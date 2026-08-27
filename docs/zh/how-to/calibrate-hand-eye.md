# 手眼标定指南（Hand-Eye Calibration）

> 类别：How-to。本文说明如何采集、求解、质检和复验手眼标定。

让装在机械臂腕部的相机「知道」自己相对机械臂的位置——标定后，相机看到的物体能被准确换算成机械臂坐标，视觉抓取才会准。

标定程序：[scripts/calibrate/calibrate_hand_eye.py](../../../scripts/calibrate/calibrate_hand_eye.py)，产出标定文件 `configs/piper/piper_calib.json`。

**什么时候需要做标定**

- 第一次装好相机/机械臂；
- 相机、镜头或相机支架被挪动过；
- 视觉抓取出现稳定的偏移（比如总是偏左、偏低同样的量）。

---

## 目录

1. [准备工作](#1-准备工作)
2. [开始标定（三步）](#2-开始标定三步)
3. [判断标定好不好](#3-判断标定好不好)
4. [复验](#4-复验)
5. [常见问题](#5-常见问题)
6. [常用参数速查](#6-常用参数速查)

---

## 1. 准备工作

### 1.1 安装依赖

```bash
pip install -e ".[calib,piper]"
```

标定代码属于根项目的 `jiuwensymbiosis.calibration` 子包；`[calib]` 提供
OpenCV 与 RealSense 支持。`[piper]` 提供 piper 机械臂驱动（如果你使用的机械臂并非
piper，需自行安装相对应的依赖）。

### 1.2 准备标定板

需要一块印有 **ChArUco**（推荐）或**棋盘格**图案的平板。

**没有现成的板？让程序生成一张可打印图：**

```bash
jiuwensymbiosis-calibrate-eye-in-hand --generate-board board.png \
    --board charuco --squares-x 5 --squares-y 7 --square-size-mm 30 --marker-size-mm 22
```

**自制标定板的建议：**

1. 用 A4 纸 **100% 原始比例**打印，关闭「适应页面/缩放」。
2. 打印后**用尺子分别测量大黑色方格和小黑色方格的实际边长**，把真实毫米数分别填给后面的 `--square-size-mm`和`--marker-size-mm`。**打印缩放是头号误差来源**。
3. **平整裱在硬板上**（KT 板/亚克力/铝板），不能弯、不能翘。
4. 表面**不要反光**（哑光纸最好）。
5. 记住你的板参数（方格数、方格边长、marker 边长）——**生成和标定必须用同一组参数**。

对于大黑色方格，可以沿一行连续测量若干个大格子的总长再除以数目，比单格量得更准。

### 1.3 配置机器人

编辑 [scripts/calibrate/calibrate.yaml](../../../scripts/calibrate/calibrate.yaml)，填入你的 RealSense 序列号：

```yaml
      camera_serial: "你的相机序列号"
```

或临时用环境变量：`export CAMERA_SERIAL=你的序列号`。

> 这份配置专为标定准备，**不要**用 `piper.yaml`——那会指向正在生成的标定文件。

---

## 2. 开始标定（三步）

把机械臂上电、CAN 接好、标定板摆在相机视野里，运行：

```bash
jiuwensymbiosis-calibrate-eye-in-hand --config scripts/calibrate/calibrate.yaml \
    --board charuco --squares-x 5 --squares-y 7 --square-size-mm 30 --marker-size-mm 22
```

请务必记得 --square-size-mm 以及 --marker-size-mm 改为你实际测量出来的那个值。

程序会**全程中文向导**带你走完。

### 第 1 步：自检与确认

程序先检查相机、机械臂是否就绪，并打印一张配置确认卡。看一眼板参数、输出文件对不对，回车继续。

### 第 2 步：采集多个角度（最关键）

手动模式下，你来摆姿势、程序来拍：

| 按键 | 作用 |
|------|------|
| 回车 | 采集当前这一帧 |
| `s`  | 采够了，开始求解 |
| `u`  | 撤销最近一帧（拍错了） |
| `q`  | 放弃退出 |

**采集要领——必须让手腕的「转动」有变化，光平移没用：**

- 每拍一帧，就**换一个姿态**：手腕往不同方向倾斜 ±20~30°、绕轴转一转、远近也变一变。
- 始终让**整块板**清楚地出现在画面里。
- 建议采 **10~15 帧**。程序顶部会实时显示「已采几帧、旋转跨度多少度」，跨度太小它会提醒你多转转。
- 每帧拍完会告诉你「✓ 采纳」还是「✗ 未采纳」及原因。

> 想让机械臂自己转着拍？加 `--auto`（会驱动机械臂运动，**确保 E-stop 在手边**；可先用 `--auto-dry-run` 看它打算去哪些位姿）。

### 第 3 步：求解并写文件

按 `s` 后程序自动求解、打印精度报告（见下一节），确认覆盖后写入 `configs/piper/piper_calib.json`（旧文件自动备份成 `.bak`）。

---

## 3. 判断标定好不好

报告里每项都带 ✅/⚠️/❌。三个核心指标：

| 指标 | 含义 | 达标（✅） | 要重做（❌） |
|------|------|-----------|-------------|
| **重投影 RMS** | 板检测得准不准 | < 1 px | > 2 px |
| **手眼一致性**（旋转 / 平移） | 各帧解出的结果一不一致 | < 0.5° / < 2~3 mm | 明显更大 |
| **板原点一致性** | 机械臂和相机对同一点的认知误差 | std < 2~3 mm | > 3 mm |

**不达标怎么办？** 绝大多数是这两个原因：

- **旋转角度不够**：回去重采，手腕多换姿态（别只平移）。
- **板尺寸填错了**：用尺子重量方格边长，确认 `--square-size-mm` 是真实值。

---

## 4. 复验

强烈建议真机验一下。在标定命令后追加 `--verify-touch`，标定完成后会立即复验——它让**指尖悬停在板中心上方约 30mm（默认不接触）**，你肉眼看指尖是否对准板中心：xy 对得上就说明标定良好。

**末端是裸法兰（没装工具）：**

```bash
jiuwensymbiosis-calibrate-eye-in-hand --config scripts/calibrate/calibrate.yaml \
    --board charuco --squares-x 5 --squares-y 7 --square-size-mm 30 --marker-size-mm 22 \
    --verify-touch
```
> 配置里 `tool_offset_mm=0` 时，程序会**强警告并要你确认**末端确实无工具——因为它会按「法兰=指尖」算悬停高度。

**末端装了工具/夹爪：** 必须用 `--verify-tool-offset-mm` 传入真实的法兰→指尖长度（mm），否则工具会比预期更低、可能撞向标定板：

```bash
... --verify-touch --verify-tool-offset-mm 95   # 例：夹爪长 95mm
```

- 悬停余量可用 `--verify-hover-mm` 调整（默认 30mm，越大越保守）。
- 不想动机械臂、只看数字：把 `--verify-touch` 换成 `--verify`，它只打印换算出的机械臂坐标供你目测（零运动）。

也可以直接跑抓取演示看整体效果：

```bash
piper-pick-demo --config configs/piper/piper.yaml
```

---

## 5. 常见问题

**Q：一直提示「未检测到标定板」？**
板要完整入镜、别太斜太远、避免反光；确认命令里的板参数和你手上的板一致。

**Q：提示「相机内参不可用」？**
非 RealSense 相机没有出厂内参，加 `--calibrate-intrinsics`（用采集到的图一起标定内参），或用 `--intrinsics fx fy ppx ppy` 手动指定。

**Q：提示「有效视图不足」？**
至少要 3 帧成功，推荐 10~15 帧。多换姿态多拍几张。

**Q：用棋盘格而不是 ChArUco？**
把 `--board charuco --marker-size-mm ...` 换成 `--board chessboard`，其余一样。ChArUco 对遮挡/模糊更鲁棒，优先用它。

**Q：没有硬件，想先熟悉一下？**
跑 `jiuwensymbiosis-calibrate-eye-in-hand --selftest`，用合成数据离线验证程序本身（无需机械臂和相机）。

---

## 6. 常用参数速查

| 参数 | 说明 |
|------|------|
| `--config <yaml>` | 机器人配置（如 `scripts/calibrate/calibrate.yaml`） |
| `--board {charuco,chessboard}` | 标定板类型（默认 charuco） |
| `--squares-x / --squares-y` | 板的方格行列数 |
| `--square-size-mm` | 方格实测边长（mm，务必准确） |
| `--marker-size-mm` | ChArUco 的 marker 边长（mm） |
| `--auto` | 自动驱动机械臂采集（注意安全） |
| `--calibrate-intrinsics` | 顺便标定相机内参（非 RealSense 用） |
| `--out <path>` | 输出标定文件（默认 `configs/piper/piper_calib.json`） |
| `--verify` / `--verify-touch` | 标定后真机复验（仅打印坐标 / 指尖悬停目视） |
| `--verify-tool-offset-mm` | verify-touch 用的法兰→指尖长度（mm，装了工具必填） |
| `--verify-hover-mm` | verify-touch 指尖悬停余量（mm，默认 30，不接触） |
| `--generate-board <png>` | 生成可打印标定板图 |
| `--selftest` | 离线自检（无需硬件） |

完整参数见 `jiuwensymbiosis-calibrate-eye-in-hand --help`。在源码仓库中仍可直接运行
`python scripts/calibrate/calibrate_hand_eye.py ...`。

上述无模式向导及 `--selftest`、`--generate-board`、`--verify` 属于明确保留的
legacy compatibility 路径，命令启动时会打印提示；`--collect-poses`、带 waypoint
archive 的 `--auto` 和 `--replay` 使用本体无关统一工作流。

---

## 7. eye-to-hand 通用标定（桌面固定相机）

上面 §1–§6 讲的是 **eye-in-hand**（相机在腕部，求 `T_flange_cam`，随法兰变），
对应 piper + `calibrate_hand_eye.py`。

使用 SO-101 和固定相机的操作者可直接阅读
[SO-101 固定相机手眼标定使用指南](calibrate-so101-eye-to-hand.md)，按完整的硬件准备、
示教、自动采集、离线重解和运行配置流程操作。

本节讲 **eye-to-hand**：相机固定在桌面/基座，求 `T_base_cam`（相机在 base
系的**常量**位姿，不随法兰变；反投影 `p_base = T_base_cam @ p_cam`，**每步不读法兰**）。
入口是本体无关的 [scripts/calibrate/eye_to_hand_calib.py](../../../scripts/calibrate/eye_to_hand_calib.py)，
安装 wheel 后使用 `jiuwensymbiosis-calibrate-eye-to-hand`，
piper 与 SO-101 都支持，新设备靠 calibration-owned adapter wrapper 实现
`jiuwensymbiosis.calibration.ports` 协议族”接入，
**零脚本改动**。

calibration 是根项目内的独立边界：它拥有模型、求解、归档、工作流和硬件 Port，
不依赖 Agent/SafetyRail，代码位于 `jiuwensymbiosis/calibration/`。
`jiuwensymbiosis.calibration.integration` 提供配置、RobotSession 生命周期、adapter
标识和制品回读验证。标定执行操作者确认的受控轨迹，不自动修改轨迹、home 或挂接
Agent Rail。可用 `make calib-test CONDA_ENV=` 和 `make calib-check CONDA_ENV=` 检查
同一根分发中的标定模块。

三个工作流可脱离 CLI 单独调用：`collect_waypoints(device, ...)`、
`execute_calibration(device, archive, ...)` 和 `replay_calibration(archive, ...)`。
它们就是标定子系统的直接调用边界。

### 7.1 两类设备、两种轨迹空间

- **SO-101**（5-DoF）：默认 `trajectory.space: joint`（关节空间）。`--collect-poses`
  用 `ManualGuidance` 同 session 手动示教（松臂力矩→托臂移动→回车记录→退出恢复力矩）。
- **Piper**（6-DoF）：可选 joint 或 cartesian。不支持 `ManualGuidance`，`--collect-poses`
  回退到外部示教器移动后 snapshot，或用 `--import-poses` 导入自描述 waypoint archive。

`camera_mount` 是相机挂载方式的**单一权威来源**：运行时权威是 adapter 配置的
`cfg.camera_mount`，再由 calibration-owned adapter wrapper 以 `device.camera_mount`
暴露给 workflow。当前 YAML loader 仍兼容旧的
`env.cfg.low_level.camera_mount` 写法；它不属于顶层 `calibration:` 段：

```yaml
env:
  cfg:
    low_level:
      camera_mount: "eye_to_hand"   # eye_in_hand | eye_to_hand
      camera_serial: "<桌面相机序列号>"
calibration:
  adapter_module: "jiuwensymbiosis.adapters.so101"
  trajectory: { space: joint }
```

### 7.2 三种模式（互斥）

```bash
# 1) 同 session 手动示教收集 waypoint（SO-101 ManualGuidance）
jiuwensymbiosis-calibrate-eye-to-hand \
    --config scripts/calibrate/so101_calibrate.yaml \
    --board charuco --squares-x 5 --squares-y 7 --square-size-mm 15.28 --marker-size-mm 11 \
    --collect-poses tmp/so101_wp.npz

# 2) 自动沿 waypoint 轨迹采集 + 求解 + 发布（live，必须有 --config）
jiuwensymbiosis-calibrate-eye-to-hand \
    --config scripts/calibrate/so101_calibrate.yaml \
    --board charuco --squares-x 5 --squares-y 7 --square-size-mm 15.28 --marker-size-mm 11 \
    --auto tmp/so101_wp.npz --n-stations 24 --confirm-estop \
    --out tmp/so101_eye_to_hand.json

# 3) 离线重解（两层语义）
#   无 --config：只输出不可被运行时加载的 REVIEW/candidate 报告（不发布正式标定）
#   有 --config：额外校验 adapter module/挂载/frame + adapter reload smoke，通过才发布
jiuwensymbiosis-calibrate-eye-to-hand --replay tmp/stations.npz
jiuwensymbiosis-calibrate-eye-to-hand --replay tmp/stations.npz \
    --config scripts/calibrate/so101_calibrate.yaml --out tmp/so101_eye_to_hand.json
```

`--collect-poses --import-poses INPUT` 不连硬件，只把自描述 waypoint archive
规范化到 OUTPUT（不猜测裸数组的单位/关节顺序/pose 约定）。

### 7.3 安全：ManualGuidance 恢复失败即中止

`ManualGuidance` 上下文退出时严格执行 **preset-before-enable**：先把当前关节角
写成 goal，再恢复力矩。preset 失败**不会**重新启动力矩（避免跳到陈旧 goal），
任一步失败抛 `ManualGuidanceRecoveryError` 中止标定——请人工托臂并归位后再继续。

### 7.4 reload 复验

正式标定（schema-2 顶层 `T_base_cam`）只有在**可观测性 ACCEPT 且通过目标
adapter `load_calibration` reload round-trip smoke test** 后才写出。REVIEW/candidate
用独立 schema（`artifact_kind=eye_to_hand_solve_report`，矩阵放在
`candidate.T_base_cam`，**无顶层 `T_base_cam`**），运行时 loader 无法加载——
`--out result.json` 时 candidate 写到 `result.candidate.json`，正式路径绝不被覆盖。

### 7.5 可观测性（固定阈值，可覆盖）

`jiuwensymbiosis.calibration.quality.observability_report` 基于相对运动评估激励质量，第一版固定阈值
（`scripts/calibrate/<adapter>_calibrate.yaml` 的 `calibration.observability` 覆盖，
同名 CLI 参数优先级更高）：相对旋转 ≥5°、≥2 根有效相对旋转轴、两轴夹角 ≥15°、
最大相对旋转 ≥20°、最大平移基线 ≥30mm、重复姿态（旋转 <2° 且平移 <5mm）即 REVIEW。
SVD 条件数只作报告项，不直接控制 ACCEPT。
