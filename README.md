# jiuwensymbiosis — OpenHarmony riscv64 移植

> 本分支（`riscv64-ohos`）只存放 **OHOS riscv64 适配层**（说明 / launcher / 打包脚本）。
> 原始代码保留在本 fork 默认分支（跟随上游 openJiuwen-ai/jiuwensymbiosis，Sync fork 即可更新）。

| 项 | 值 |
|---|---|
| 状态 | 已上机验证：mock 对拍全绿（无需机器人/LLM） |
| 上游基线 | main @ ecc9283a9b2c3aa2223d257c194674e9fd3e06bf |
| brew 包 | `hbrew/riscv/openjiuwen-symbiosis`（bottle 在 riscv-bin Release） |
### 适配方式

1. scipy 依赖以 **纯 numpy shim** 替换（设备端 scipy 无法全量交叉编译，shim 覆盖 symbiosis
   实际用到的子集，`deploy-symbiosis/scipy`）；numpy 用 riscv64 交叉编译版；
2. `configs/` 与 `examples/run_task.py` 随包分发，`--mock --no-visual-feedback` 模式离线可跑；
3. 无源码补丁。

### 冒烟

```bash
jiwensymbiosis-demo    # piper 配置 mock 1 轮迭代
```

### ohos-deploy/（设备端 launcher，随瓶分发）

- `openjiuwen-symbiosis`
- `0.1.0`
- `['jiwensymbiosis-demo']`

## 通用运行环境（OHOS riscv64 适配要点）

所有 openJiuwen 组件在设备上共享同一套运行约定（详见各 launcher）：

- `unset PYTHONHOME PYTHONPATH` 后重建 `PYTHONPATH`（hdc shell 会泄漏 /system 的 3.10 环境）；
- `LD_PRELOAD=libriscvflush.so`（riscv64 icache flush shim，openjiuwen 基础包提供）；
- `TMPDIR=/data/tmp`、`SSL_CERT_FILE=/etc/ssl/certs/cacert.pem`（musl 无系统 CA bundle）；
- 解释器优先级：**/data/python312（3.12.14）→ brew python 3.11**；3.12 优先时
  PYTHONPATH 需前置 `/data/python312/lib/python3.12/site-packages`
  （**ohos-312-patch**，否则注入的 3.11 编 pydantic_core 等二进制在 3.12 下无法加载）；
- pip 在线源：tuna（主）+ aliyun（备），musllinux_1_2_riscv64 轮可直装，
  用 `pipm` 安装（自动把 musl 后缀 .so 改成设备 gnu 后缀）；
- 运行日志写 cwd 下 `logs/`（launcher 已 cd $HOME/ojw-run 并建目录）。

## 设备实机验证（2026-09-03，K3 pico / OpenHarmony 6.1 / python 3.12.14）

- 18 个代码仓中 11 个已完成设备实机验证（含本仓）；
- 5 个源码 vendor 仓本轮验证：agent-runtime / agent-protocol / agent-tools / skillhub / agent-dx
  全部通过（边界记录见各仓章节）；
- 验证脚本与记录：openjiuwen-ohos-port 仓 + workspace/ojw-py312-musepaper2/。

## 安装与更新

```bash
# 设备上（已配置 Harmonybrew）
. /data/harmonybrew/hbrew-env.sh
brew install hbrew/riscv/<formula>     # 见下表
```

上游更新流程：Sync fork（默认分支保持上游原样）→ 在新基线上重估本分支说明与
launcher 兼容性 → 重建 bottle → 更新本 README 基线记录。
