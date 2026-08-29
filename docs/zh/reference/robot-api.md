# 机器人环境、能力与工具 API 参考

> 类别：Reference。本页以 `jiuwensymbiosis.env`、`jiuwensymbiosis.api` 和 `jiuwensymbiosis.tools` 的公开代码为基线。

## `RobotObservation`

```python
RobotObservation(
    pose: dict | None = None,
    joints: list[float] | None = None,
    rgb: numpy.ndarray | None = None,
    depth: numpy.ndarray | None = None,
    extra: dict = {},
)
```

位姿字段由机器人约定；SCARA 通常使用 `x/y/z/r`，六轴通常使用 `x/y/z/rx/ry/rz`。关节角单位必须遵循对应 Env 的约定（`env.joint_units`；未声明视为未知），深度默认以米为单位。

## `BaseRobotEnv`

子类必须实现：

```python
connect() -> None
disconnect() -> None
get_observation() -> RobotObservation
home() -> None
```

`home` 之所以是抽象方法：它背后是唯一的无条件动作（`ActionSpec(capability=None)`），若给出笛卡尔默认实现，就会把 `motion.cartesian`
悄悄塞进能力门拦不住的本体里。可选生命周期方法为 `reset()` 和 `emergency_stop()`。常用属性包括 `capabilities`、`low_level`、`z_min_safe`、`workspace_bounds`、`joint_limits`、`home_pose`、`tool_offset_mm`、`joint_units`、`default_orientation_policy`、`base_step_limits`、`lift_limits`、`waist_step_limit_rad`、`cameras`、`urdf_path`/`arm_chains`/`arm_joints`。

`has(capability)` 检查能力声明。子类创建时，未知能力会立即触发 `ValueError`。

## `BaseRobotApi` 与动作词表

`BaseRobotApi(env)` 保存 Env 引用，并在两个地方提供 `@implements`：

- `home`（唯一无条件动作，重写它委托 `env.home()`）
- 各本体加的其余动作

`capabilities` 属性**反推自本体实现的动作的 spec**（每个 `@implements` 贡献自己 `ActionSpec` 的能力），再加上声明的 marker 能力类属性（如 `motion.servo`、`planning.reachability`——它们没有对应动作，只能靠属性声明）。**实现哪个动作就自动具备哪个能力**，不会广告本体没有的能力。

动作词表在 `jiuwensymbiosis/api/actions.py`。每条动作声明：描述、能力门、参数名、结果形状、`requires`/`provides`/`invalidates`、`produces_location`/`consumes_location`/`invalidates_locations`、`planner_visible`。通用实现（转发 Env 动词就能完成）在 `api/defaults.py`，适配器按需取用。

## 已知 Capability

- `motion.cartesian`
- `motion.joint`
- `motion.servo`
- `motion.base`
- `motion.base_servo`
- `motion.lift`
- `motion.waist`
- `motion.goal`
- `motion.dual_arm`
- `grasp.suction`
- `grasp.parallel`
- `grasp.paddle`
- `vision.camera`
- `vision.depth`
- `vision.detection`
- `vision.eye_to_hand`
- `vision.search`
- `planning.reachability`
- `sorting.command`
- `speech.tts`

实际词表以 `jiuwensymbiosis.env.base.KNOWN_CAPABILITIES` 为准。

## `@implements(SPEC)`

```python
implements(spec: ActionSpec) -> Callable   # api/actions.py
```

方法变成工具的唯一入口。它把一个 `ToolMeta` 挂到方法上，内含 `spec` 与 `input_params`——后者由本体自己的签名推导（按 `spec.params` 过滤，再由 `spec.param_schema` 细化）。描述、capability、tags、requires / provides / invalidates、位置新鲜度等契约字段全部回读自 spec，所以实现方没有渠道对规划器说词表之外的话。

若签名接不下 spec 承诺的参数，导入时即抛 `ContractViolation`。

bring-up、标定与调试视图**不是动作**：不加装饰器，用 `scripts/` 下的脚本驱动。

## 动作契约（可调用，也可规划）

每个 `ActionSpec` 还携带规划契约，供两级规划器推导合法顺序：

- `result` —— 结果字段 JSON Schema，自动派生自 `TypedDict`（或并集）；权威源在 `jiuwensymbiosis/contracts.py`（归属任何层），失败/成功形状取并集
- `requires`/`provides`/`invalidates` —— 本体自身状态，基于 `api/state.py:KNOWN_STATE_TOKENS`（`payload.held`/`payload.clear`/`payload.stowed`/`body.home`）
- `produces_location`/`consumes_location`/`invalidates_locations` —— 位置新鲜度（感知目标在哪的动作→产生；移动底盘的动作→作废所有从旧视角测得的位置）

契约不编码顺序；`parse_sequence` 只拒绝前置条件不满足的排列。`WorldState.snapshot(session)` 在运行时用同一词表汇报当前状态（观测覆盖推想；缺失即未知，从不 false）。

## 工具构建

```python
build_robot_tools(api, *, env=None, allow=None, deny=None) -> list[Any]
list_tool_meta(api, *, env=None) -> list[dict]
```

传入 Env 时，有效工具按 `api.capabilities ∩ env.capabilities` 门控；`allow` 和 `deny` 使用工具名过滤。能力来自动作自身的 `ActionSpec`，从不来自哪个类声明了方法。

## 聚合工具与代码工具

```python
RobotControlTool(
    api,
    *,
    env=None,
    name="robot_control",
    description=None,
    agent_id=None,
)
```

`available_actions` 返回可派发动作；`invoke({"action": ..., "params": {...}})` 返回 `ToolOutput`。安全 Rail 透明解包 `action`/`params`。

```python
InProcessCodeTool(globals_provider)
```

`run(code)` 在进程内执行代码并返回结果字典；`as_openjiuwen_tool(**kwargs)` 构造对应的 `LocalFunction`。调用方必须提供每次执行时返回全局变量字典的 `globals_provider`。
