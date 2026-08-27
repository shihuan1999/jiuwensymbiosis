# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""HardwareEngine —— 「硬件控制」的后台执行引擎(松力矩手动摆位)。

与 ``PerceptionEngine`` / ``CalibrationEngine`` 同构:后台线程干活,事件塞进队列,
界面用 ``ui.timer`` 轮询 ``drain()``。力矩编排一行不写在这里 —— 全在驱动的
``HandGuidingDriver.hand_guiding``,本模块只负责三件界面才需要的事:

1. **在动力矩之前设一道确认门**。支不支持松力矩取决于驱动实现了哪个端口
   (``HandGuidingDriver``),而这只有连接后才知道。所以连上先把结论报给界面
   (``mode``)并阻塞等确认,再进 ``hand_guiding`` —— 保证"力矩会掉"的警告严格早于
   力矩真的掉,且判据是端口而非机型名。
2. **把关节角推成实时读数**。松力矩期间循环取观测,连同软限位一起推给界面,操作者
   拖到哪儿、有没有拖过限位都能当场看见。
3. **在恢复力矩之前先自检限位**。驱动恢复力矩时要把实测关节角写回目标,越限会被
   拒绝并让机械臂**停在失力矩状态**(见 ``preset_current_joint_goal``)。所以按钮
   点下去先自检:越限就发 ``blocked`` 并留在手引导里(力矩仍关,臂还在操作者手上),
   而不是让它走进那条异常路径。

事件: ``mode`` / ``state`` / ``blocked`` / ``phase`` / ``log`` / ``done`` / ``error``。
"""

from __future__ import annotations

import logging
import queue
import time
from collections.abc import Callable
from dataclasses import dataclass
from threading import Thread
from typing import Any

from jiuwensymbiosis.gui.run_engine import QueueLogHandler
from jiuwensymbiosis.utils.logging import get_logger

logger = get_logger(__name__)

__all__ = ["HardwareEngine", "HardwareSetup", "limit_violations"]

# 读数刷新周期。拖拽是手上的连续动作,5fps 足够跟手,又不会让串口读满负荷。
_STATE_PERIOD_S = 0.2
# 命令队列轮询周期。远小于读数周期,保证按钮响应感觉是即时的。
_COMMAND_POLL_S = 0.05
_CMD_START = "go"
_CMD_FINISH = "done"
# 收这棵树的日志:驱动的力矩/限位告警都在 jiuwensymbiosis 下。
_LOGGER_ROOT = "jiuwensymbiosis"


@dataclass(frozen=True)
class HardwareSetup:
    """一次硬件控制会话的全部输入(界面在点「连接」时组装好)。"""

    build_session: Callable[..., Any]
    config_data: dict[str, Any]


def limit_violations(
    joints: list[float] | None,
    limits: dict[str, tuple[float, float]] | None,
) -> list[dict[str, Any]]:
    """返回越出软限位的关节明细,顺序与 ``joints`` 一致。

    ``limits`` 的键序与 ``joints`` 的下标对齐是 Env 层既有契约(SafetyRail 的
    ``q[i]`` 标签同样依赖它);长度对不上说明契约破了,此时不猜,按"无法判断"返回空。
    """
    if not joints or not limits:
        return []
    names = list(limits)
    if len(names) != len(joints):
        logger.warning("关节数 %d 与限位数 %d 不一致,跳过越限判断。", len(joints), len(names))
        return []
    out: list[dict[str, Any]] = []
    for index, (name, value) in enumerate(zip(names, joints, strict=True)):
        low, high = limits[name]
        if value < low or value > high:
            out.append(
                {
                    "index": index,
                    "name": name,
                    "value": float(value),
                    "low": float(low),
                    "high": float(high),
                    # 该往哪个方向搬:界面直接照抄,不必自己再判一次符号。
                    "direction": "调小" if value > high else "调大",
                }
            )
    return out


class HardwareEngine:
    """硬件控制后台引擎。一个实例服务一次「连接 → 松力矩 → 摆位 → 恢复」。"""

    def __init__(self, setup: HardwareSetup) -> None:
        """记下输入;线程与队列在 :meth:`start` 时才建立。"""
        self._setup = setup
        self._events: queue.Queue = queue.Queue()
        self._commands: queue.Queue[str] = queue.Queue()
        self._thread: Thread | None = None
        self._stop = False
        self._guiding = False

    # ------------------------------------------------------------------ 控制
    def start(self) -> None:
        """起线程:连接 → 报告示教方式 → 等确认 → 松力矩。幂等。"""
        if self.is_running():
            return
        self._stop = False
        _drain_queue(self._commands)
        self._thread = Thread(target=self._run, name="jiuwen-gui-hardware", daemon=True)
        self._thread.start()

    def confirm_release(self) -> None:
        """界面线程调用:用户已读过警示,放行到松力矩。"""
        self._commands.put(_CMD_START)

    def request_restore(self) -> None:
        """界面线程调用:请求恢复力矩并断开(越限时会被工作线程挡回来)。"""
        self._commands.put(_CMD_FINISH)

    def stop(self) -> None:
        """请求停止。手引导中等同于请求恢复 —— 绝不能把机械臂丢在失力矩状态。"""
        self._stop = True
        self._commands.put(_CMD_FINISH)

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def is_guiding(self) -> bool:
        """是否正处于失力矩状态(供页面切换/开跑前判断能否放开硬件)。"""
        return self._guiding

    def join(self, timeout: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def drain(self) -> list[tuple[str, Any]]:
        """非阻塞取出全部排队事件,供界面 ``ui.timer`` 周期消费。"""
        events: list[tuple[str, Any]] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                return events

    # ------------------------------------------------------------------ 内部:线程
    def _emit(self, tag: str, payload: Any) -> None:
        self._events.put((tag, payload))

    def _run(self) -> None:
        """跑完整个会话,把异常翻译成界面事件。

        ``HandGuidingRecoveryError`` 单独标 ``fatal``:它意味着机械臂可能仍处于失力矩
        状态,用户必须先用手接住,不能只当作一条普通错误滚进日志。
        """
        from jiuwensymbiosis.env.protocol import HandGuidingRecoveryError

        try:
            with self._log_capture():
                self._session()
        except HandGuidingRecoveryError as exc:
            logger.exception("硬件控制:力矩恢复失败")
            self._emit("error", {"reason": str(exc), "fatal": True})
        except Exception as exc:
            logger.exception("硬件控制运行失败")
            self._emit("error", {"reason": str(exc), "fatal": False})
        finally:
            self._guiding = False

    def _session(self) -> None:
        from jiuwensymbiosis.env.protocol import HandGuidingDriver

        self._emit("phase", {"phase": "connect", "msg": "正在连接机械臂…"})
        session = self._setup.build_session(self._setup.config_data, include_sidecars=False)
        with session:
            env = session.env
            supported = isinstance(getattr(env, "low_level", None), HandGuidingDriver)
            self._emit("mode", {"mode": "hand_guiding" if supported else "unsupported"})
            if not supported:
                # 连着也没用:没有这个端口就没有可松的力矩。直接收摊,别占着串口。
                return
            if not self._await(_CMD_START):
                return
            self._guide(env)

    def _guide(self, env: Any) -> None:
        """松力矩,推读数,等到限位合规才退出上下文恢复力矩。"""
        with env.hand_guiding(include_end_effector=True):
            self._guiding = True
            self._emit("phase", {"phase": "guiding", "msg": "已松开力矩,可用手摆位。"})
            next_state = 0.0
            while True:
                now = time.monotonic()
                if now >= next_state:
                    next_state = now + _STATE_PERIOD_S
                    self._pump_state(env)
                try:
                    command = self._commands.get(timeout=_COMMAND_POLL_S)
                except queue.Empty:
                    continue
                if command != _CMD_FINISH:
                    continue
                # 重新读一次而不是复用上一拍的读数:这一下的后果是恢复力矩,
                # 判据必须是此刻的姿态。
                violations = self._pump_state(env)
                if not violations:
                    break
                # 停止请求也一样拦:带着越限的姿态去恢复,只会换来一个机械臂
                # 仍然失力矩的异常。
                self._emit("blocked", {"violations": violations})
                next_state = time.monotonic() + _STATE_PERIOD_S
        self._guiding = False
        self._emit("done", {"phase": "guiding"})

    def _pump_state(self, env: Any) -> list[dict[str, Any]]:
        """读一次观测并推给界面,返回当前的越限明细。"""
        joints, pose = _read_observation(env)
        violations = limit_violations(joints, env.joint_limits)
        self._emit(
            "state",
            {
                "joints": joints,
                "limits": _limits_payload(env.joint_limits),
                "violations": violations,
                "pose": pose,
            },
        )
        return violations

    def _await(self, expected: str) -> bool:
        """阻塞等一条界面命令;收到停止请求或别的令牌返回 ``False``。"""
        while not self._stop:
            try:
                return self._commands.get(timeout=_COMMAND_POLL_S) == expected
            except queue.Empty:
                continue
        return False

    def _log_capture(self):
        """把驱动的力矩/限位告警接进事件队列(界面的日志区就是这些)。"""
        from contextlib import contextmanager

        @contextmanager
        def _ctx():
            handler = QueueLogHandler(self._events, level=logging.INFO)
            target = logging.getLogger(_LOGGER_ROOT)
            target.addHandler(handler)
            try:
                yield
            finally:
                target.removeHandler(handler)

        return _ctx()


def _read_observation(env: Any) -> tuple[list[float] | None, dict[str, float] | None]:
    """取一次观测;读失败不中断摆位(手还在臂上,断掉读数比断掉力矩安全得多)。"""
    try:
        obs = env.get_observation()
    except Exception as exc:
        logger.warning("硬件控制:读取观测失败: %s", exc)
        return None, None
    joints = list(obs.joints) if obs.joints is not None else None
    return joints, obs.pose


def _limits_payload(limits: dict[str, tuple[float, float]] | None) -> list[dict[str, Any]]:
    """把限位摊平成有序列表,与 ``joints`` 的下标一一对应。"""
    if not limits:
        return []
    return [{"name": name, "low": float(low), "high": float(high)} for name, (low, high) in limits.items()]


def _drain_queue(q: queue.Queue) -> None:
    """清空队列(每次开始前丢掉上一轮的残留命令)。"""
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            return
