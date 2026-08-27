# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""run_engine:后台线程 + 事件队列驱动一次模拟运行(纯逻辑,无 Qt / 无 nicegui)。"""

from __future__ import annotations

import asyncio
import logging
import queue
from types import SimpleNamespace

import jiuwensymbiosis.gui.run_engine as run_engine_module
from jiuwensymbiosis.gui import registry
from jiuwensymbiosis.gui.bridge import UIBridgeRail
from jiuwensymbiosis.gui.run_engine import (
    QueueLogHandler,
    RunEngine,
    default_workspace,
    strip_vision_services,
)
from tests.mocks.mock_session import make_mock_session

# One full identify→grasp→lift chain using only tools the hardware-free session exposes.
_SCRIPT = [
    {"tool": "home", "args": {}},
    {"tool": "get_grasp_info_simple", "args": {"object_name": "black box"}},
    {"tool": "goto_xyzr", "args": {"x": 230, "y": 0, "z": 90}},
    {"tool": "close_gripper", "args": {}},
    {"tool": "goto_xyzr", "args": {"x": 230, "y": 0, "z": 250}},
]


def _tags(events):
    return [tag for tag, _ in events]


def _use_hardware_free_session(monkeypatch, tmp_path):
    """Make ``RunEngine._build`` construct a MockArmEnv-backed session instead of a real robot."""
    stub_body = SimpleNamespace(
        build_real_session=lambda _cfg: make_mock_session(),
        config_path=lambda: tmp_path / "body.yaml",
    )
    monkeypatch.setattr(run_engine_module, "get_body", lambda _key: stub_body)


def _patch_script_runner(monkeypatch, script):
    """Drive the UI rail directly; RunEngine unit tests do not need DeepAgent."""

    def fake_run_robot_task(_session, _query, cfg, *, conversation_id, cancel_token=None):
        del conversation_id, cancel_token
        bridge = next(rail for rail in cfg.extra_rails if isinstance(rail, UIBridgeRail))

        async def drive_script():
            for step in script:
                inputs = SimpleNamespace(
                    tool_name=step["tool"],
                    tool_args=step.get("args", {}),
                    tool_result={"ok": True},
                )
                ctx = SimpleNamespace(inputs=inputs, extra={})
                await bridge.before_tool_call(ctx)
                await bridge.after_tool_call(ctx)

        asyncio.run(drive_script())
        return {"output": "模拟任务完成", "result_type": "answer"}

    monkeypatch.setattr(run_engine_module, "run_robot_task", fake_run_robot_task)


def test_run_emits_ordered_event_stream(tmp_path, monkeypatch):
    _use_hardware_free_session(monkeypatch, tmp_path)
    _patch_script_runner(monkeypatch, _SCRIPT)
    task = registry.get_task("pick_box")
    config = {
        "env": {"cfg": {"prompt": "把黑盒放到白盒上"}},
        "agent": {
            "mode": "tool",
            "exec_mode": "stepagent",
            "max_iterations": 20,
            "enable_visual_feedback": False,
            "enable_tracing": False,
        },
    }
    engine = RunEngine(task, config, workspace=str(tmp_path), body_key="piper")
    engine.start()
    engine.join(timeout=5)
    assert not engine.is_running()

    events = engine.drain()
    tags = _tags(events)
    assert tags[0] == "run_started"
    assert tags[-1] == "run_finished"

    meta = events[0][1]
    assert meta["body"] == "piper"

    finished = [payload for tag, payload in events if tag == "step_finished"]
    tools = [f["tool"] for f in finished]
    assert tools == [step["tool"] for step in _SCRIPT]  # 忠实回放脚本序列
    assert all(f["ok"] for f in finished)

    result = events[-1][1]
    assert result["ok"] is True


def test_finished_run_carries_log_tail_for_diagnosis(tmp_path, monkeypatch):
    # 无异常≠成功:fast 内层步骤失败也走这一支,诊断要拿得到日志尾佐证
    _use_hardware_free_session(monkeypatch, tmp_path)
    _patch_script_runner(monkeypatch, _SCRIPT)
    engine = RunEngine(registry.get_task("pick_box"), {}, workspace=str(tmp_path), body_key="piper")
    engine.start()
    engine.join(timeout=5)

    result = engine.drain()[-1][1]
    assert result["ok"] is True
    assert isinstance(result["log_tail"], str)


def test_run_frames_are_encoded_data_uris(tmp_path, monkeypatch):
    _use_hardware_free_session(monkeypatch, tmp_path)
    _patch_script_runner(monkeypatch, _SCRIPT)
    task = registry.get_task("pick_box")
    config = {
        "agent": {
            "mode": "tool",
            "exec_mode": "stepagent",
            "max_iterations": 20,
            "enable_visual_feedback": False,
            "enable_tracing": False,
        }
    }
    engine = RunEngine(task, config, workspace=str(tmp_path), body_key="piper")
    engine.start()
    engine.join(timeout=5)
    assert not engine.is_running()

    frames = [payload for tag, payload in engine.drain() if tag == "frame"]
    assert frames  # 初始帧 + 运动/抓取后各刷新
    assert all(isinstance(uri, str) and uri.startswith("data:image/jpeg;base64,") for uri in frames)


def test_rerun_with_keeps_body_and_task_but_takes_the_given_config(tmp_path):
    task = registry.get_task("pick_box")
    config = {"env": {"cfg": {"prompt": "把黑盒放到白盒上"}}, "agent": {"mode": "tool"}}
    engine = RunEngine(task, config, workspace=str(tmp_path), body_key="piper")

    edited = {"env": {"cfg": {"prompt": "改过的指令"}}, "agent": {"mode": "tool"}}
    twin = engine.rerun_with(edited)

    assert twin is not engine
    assert twin._task is task and twin._workspace == str(tmp_path)
    assert twin._body_key == "piper"
    # 重跑用的是传进来的配置,不是引擎开跑时那份快照。
    assert twin._config.get("env.cfg.prompt") == "改过的指令"
    assert engine._config.get("env.cfg.prompt") == "把黑盒放到白盒上"
    twin._config.set("env.cfg.prompt", "又改了")  # 深拷贝:动新引擎不回写调用方的 dict
    assert edited["env"]["cfg"]["prompt"] == "改过的指令"


def test_engine_exposes_the_body_and_task_it_ran(tmp_path):
    task = registry.get_task("pick_box")
    engine = RunEngine(task, {}, workspace=str(tmp_path), body_key="piper")
    assert engine.body_key == "piper"
    assert engine.task_key == task.key


def test_drain_is_empty_before_start(tmp_path):
    engine = RunEngine(registry.get_task("pick_box"), {}, workspace=str(tmp_path), body_key="piper")
    assert engine.drain() == []
    assert engine.is_running() is False


class _JointDriver:
    """满足 ``JointDriver`` 的假驱动(真类:runtime_checkable 用 getattr_static)。"""

    def __init__(self, *, fail: bool = False) -> None:
        self.moved: list[list[float]] = []
        self.homed = 0
        self._fail = fail

    def get_angles(self):
        return [0.0, 0.0]

    def move_joint_blocking(self, q, *, timeout_s=None):
        if self._fail:
            raise RuntimeError("out of soft limits")
        self.moved.append(list(q))

    def home(self):
        self.homed += 1


class _Session:
    """假会话。必须是真类:``with`` 的 dunder 查找在类型上,SimpleNamespace 顶不了。"""

    def __init__(self, env) -> None:
        self.env = env

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _pose_session(monkeypatch, tmp_path, *, driver, joints):
    """把 ``get_body`` 换成一个产出可控 env 的桩,并记下 build 时的关键字。"""
    seen: dict = {}
    session = _Session(
        SimpleNamespace(low_level=driver, get_observation=lambda: SimpleNamespace(joints=joints, rgb=None))
    )

    def build(_cfg, *, include_sidecars=True):
        seen["include_sidecars"] = include_sidecars
        return session

    monkeypatch.setattr(
        run_engine_module,
        "get_body",
        lambda _key: SimpleNamespace(build_real_session=build, config_path=lambda: tmp_path / "body.yaml"),
    )
    return seen


def _engine(tmp_path):
    return RunEngine(registry.get_task("pick_box"), {}, workspace=str(tmp_path), body_key="piper")


def test_start_pose_snapshot_carries_the_joints_at_connect(tmp_path, monkeypatch):
    _pose_session(monkeypatch, tmp_path, driver=_JointDriver(), joints=[1.0, 2.0])
    engine = _engine(tmp_path)

    engine._emit_start_pose(run_engine_module.get_body("piper").build_real_session({}))

    assert engine.drain() == [("start_pose", {"body": "piper", "joints": [1.0, 2.0]})]


def test_start_pose_is_skipped_without_a_joint_capable_driver(tmp_path, monkeypatch):
    """回不去的目标不如不给:按钮该保持禁用,而不是点了才失败。"""
    _pose_session(monkeypatch, tmp_path, driver=SimpleNamespace(), joints=[1.0, 2.0])
    engine = _engine(tmp_path)

    engine._emit_start_pose(run_engine_module.get_body("piper").build_real_session({}))

    assert engine.drain() == []


def test_start_pose_is_skipped_without_joint_readings(tmp_path, monkeypatch):
    _pose_session(monkeypatch, tmp_path, driver=_JointDriver(), joints=None)
    engine = _engine(tmp_path)

    engine._emit_start_pose(run_engine_module.get_body("piper").build_real_session({}))

    assert engine.drain() == []


def test_return_to_start_drives_joints_and_never_calls_home(tmp_path, monkeypatch):
    """``home_use_init_pose`` 的本体新连接时 home 就是当前姿态,home() 会原地不动。"""
    driver = _JointDriver()
    seen = _pose_session(monkeypatch, tmp_path, driver=driver, joints=[1.0, 2.0])
    engine = _engine(tmp_path)

    engine.start_return_to([10.0, 20.0])
    engine.join(timeout=5)

    assert driver.moved == [[10.0, 20.0]]
    assert driver.homed == 0
    assert seen["include_sidecars"] is False, "回位只动机械臂,不该拉起检测服务"
    assert _tags(engine.drain()) == ["pose_return_started", "pose_return_finished"]


def test_return_to_start_reports_a_rejected_move(tmp_path, monkeypatch):
    _pose_session(monkeypatch, tmp_path, driver=_JointDriver(fail=True), joints=[1.0, 2.0])
    engine = _engine(tmp_path)

    engine.start_return_to([10.0, 20.0])
    engine.join(timeout=5)

    finished = engine.drain()[-1][1]
    assert finished["ok"] is False
    assert "out of soft limits" in finished["error"]


def test_return_to_start_is_refused_while_a_run_is_in_flight(tmp_path, monkeypatch):
    driver = _JointDriver()
    _pose_session(monkeypatch, tmp_path, driver=driver, joints=[1.0, 2.0])
    engine = _engine(tmp_path)
    engine._thread = SimpleNamespace(is_alive=lambda: True)

    engine.start_return_to([10.0, 20.0])

    assert driver.moved == []


def test_strip_vision_services_removes_detector_and_camera():
    cfg = {
        "api_servers": [{"_target_": "x.grounding_dino_sam2_server.main"}],
        "env": {"cfg": {"low_level": {"camera_serial": "123", "port": "/dev/ttyUSB0"}}},
    }
    out = strip_vision_services(cfg)
    assert "api_servers" not in out  # 检测器 sidecar 不再 spawn
    assert "camera_serial" not in out["env"]["cfg"]["low_level"]  # 相机不打开
    assert out["env"]["cfg"]["low_level"]["port"] == "/dev/ttyUSB0"  # 非视觉字段保留
    assert "api_servers" in cfg  # 深拷贝:原配置不动


def test_disable_vision_strips_real_session_config(tmp_path):
    task = registry.get_task("pick_banana")
    config = {
        "gui": {"disable_vision": True},
        "api_servers": [{"_target_": "x.grounding_dino_sam2_server.main"}],
        "env": {"cfg": {"low_level": {"camera_serial": "123"}, "prompt": "抓香蕉"}},
    }
    engine = RunEngine(task, config, workspace=str(tmp_path), body_key="so101")
    real = engine._real_session_config()
    assert "api_servers" not in real
    assert "camera_serial" not in real.get("env", {}).get("cfg", {}).get("low_level", {})


def test_default_workspace_under_home():
    assert default_workspace().endswith("gui_workspace")


def test_queue_log_handler_enqueues_and_keeps_tail():
    events: queue.Queue = queue.Queue()
    handler = QueueLogHandler(events)
    record = logging.LogRecord("jiuwensymbiosis", logging.WARNING, __file__, 1, "视觉检测未就绪", None, None)
    handler.emit(record)
    tag, payload = events.get_nowait()
    assert tag == "log"
    assert payload["level"] == "WARNING"
    assert "视觉检测未就绪" in payload["msg"]
    assert "视觉检测未就绪" in handler.log_tail()


def test_step_frame_event_carries_index_and_data_uri(tmp_path):
    import numpy as np

    engine = RunEngine(registry.get_task("pick_box"), {}, workspace=str(tmp_path), body_key="piper")
    engine.step_frame(3, np.zeros((4, 4, 3), dtype=np.uint8))
    events = engine.drain()
    assert len(events) == 1
    tag, payload = events[0]
    assert tag == "step_frame"
    assert payload["index"] == 3
    assert isinstance(payload["uri"], str) and payload["uri"].startswith("data:image/jpeg;base64,")
