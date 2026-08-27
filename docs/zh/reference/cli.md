# 命令行参考

> 类别：Reference。控制台入口由 `pyproject.toml` 的 `[project.scripts]` 定义。

## piper-pick-demo

```bash
piper-pick-demo --config PATH [--query TEXT | --voice ...] [--mock]
```

`--config` 必填；非语音模式必须提供 `--query`。`--mock` 使用离线模型和 Mock 环境。常用覆盖项包括 `--model`、`--server-url`、`--api-key`、`--max-iter`、`--workspace` 和 `--debug`。

## jiuwensymbiosis-replay

```bash
jiuwensymbiosis-replay TRACE_JSON [--text]
```

默认生成自包含 HTML 回放并打印路径；`--text` 输出终端时间线。

## jiuwensymbiosis-gui

```bash
jiuwensymbiosis-gui
# 等价于
python -m jiuwensymbiosis.gui
```

启动监听 `127.0.0.1` 的 NiceGUI 浏览器界面。依赖缺失时，启动前检查会提示安装 `.[gui]`。

## 手眼标定

安装 `.[calib]` 后提供三个入口：

```bash
jiuwensymbiosis-calibrate-hand-eye --collect-poses OUTPUT --config RUNTIME_YAML
jiuwensymbiosis-calibrate-hand-eye --auto WAYPOINT_ARCHIVE --config RUNTIME_YAML --confirm-estop
jiuwensymbiosis-calibrate-hand-eye --replay STATION_ARCHIVE [--config RUNTIME_YAML]
```

`jiuwensymbiosis-calibrate-hand-eye` 是 mount-neutral 统一入口；相机安装方式来自运行时配置或 archive，不能由命令行覆盖。`jiuwensymbiosis-calibrate-eye-to-hand` 使用相同流程并强制要求 `eye_to_hand`，`jiuwensymbiosis-calibrate-eye-in-hand` 对 archive 模式强制要求 `eye_in_hand`。

退出码：`0` 成功或 dry-run 通过，`1` 执行错误，`2` preflight 契约失败，`3` 只生成不可加载的 REVIEW/candidate 报告。

图形界面的「工具 → 手眼标定」向导驱动的是同一套工作流，并额外提供标定板 PDF 生成与示教期实时角点反馈；首次标定建议从向导开始。
