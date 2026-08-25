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

位姿字段由机器人约定；SCARA 通常使用 `x/y/z/r`，六轴通常使用 `x/y/z/rx/ry/rz`。关节角单位必须遵循对应 Env 的约定，深度默认以米为单位。

## `BaseRobotEnv`

子类必须实现：

```python
connect() -> None
disconnect() -> None
get_observation() -> RobotObservation
home() -> None
```

`home` 之所以是抽象方法：它背后是唯一的无条件动作，若给出笛卡尔默认实现，就会把 `motion.cartesian`
悄悄塞进能力门拦不住的本体里。可选生命周期方法为 `reset()` 和 `emergency_stop()`。常用属性包括 `capabilities`、`low_level`、`z_min_safe`、`workspace_bounds`、`joint_limits`、`home_pose` 和 `tool_offset_mm`。

`has(capability)` 检查能力声明。子类创建时，未知能力会立即触发 `ValueError`。

## `BaseRobotApi` 与 Capability Mixin

`BaseRobotApi(env)` 保存 Env 引用；`capabilities` 属性从 MRO 中所有 Mixin 的 `capability` 声明自动求并集。

| Mixin | Capability | 主要工具 |
| --- | --- | --- |
| `MotionMixin` | `motion.cartesian` | `home`、`get_pose`、`goto_xyzr`、`move_direction` |
| `JointMotionMixin` | `motion.joint` | 关节空间运动 |
| `SuctionMixin` | `grasp.suction` | 吸附与释放 |
| `ParallelGripperMixin` | `grasp.parallel` | 夹爪开合 |
| `VisionMixin` | `vision.detection` | 图像、检测、坐标投影和场景分析 |

运动、关节、抓取和 `get_image` 提供委托 Env 的默认实现。`VisionMixin` 还完整实现
`get_grasp_info_simple` 与 `pixel_to_base_xyz`；适配器只需提供 `_project_pixel_to_base_raw`，
并在需要场景分析时实现 `analyze_scene`。

## 已知 Capability

- `motion.cartesian`
- `motion.joint`
- `motion.servo`
- `grasp.suction`
- `grasp.parallel`
- `vision.camera`
- `vision.depth`
- `vision.detection`
- `vision.eye_to_hand`
- `sorting.command`
- `speech.tts`

实际词表以 `jiuwensymbiosis.env.base.KNOWN_CAPABILITIES` 为准。

## `@implements(SPEC)`

```python
implements(spec: ActionSpec) -> Callable   # api/actions.py
```

方法变成工具的唯一入口。它把一个 `ToolMeta` 挂到方法上，内含 `spec` 与 `input_params`——后者由本体自己的签名推导
（按 `spec.params` 过滤，再由 `spec.param_schema` 细化）。描述、capability、tags、requires / provides / invalidates、
位置新鲜度等契约字段全部回读自 spec，所以实现方没有渠道对规划器说词表之外的话。

若签名接不下 spec 承诺的参数，导入时即抛 `ContractViolation`。

bring-up、标定与调试视图**不是动作**：不加装饰器，用 `scripts/` 下的脚本驱动。

## 工具构建

```python
build_robot_tools(api, *, env=None, allow=None, deny=None) -> list[Any]
list_tool_meta(api, *, env=None) -> list[dict]
```

传入 Env 时，有效工具按 `api.capabilities ∩ env.capabilities` 门控；`allow` 和 `deny` 使用工具名过滤。

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

`available_actions` 返回可派发动作；`invoke({"action": ..., "params": {...}})` 返回 `ToolOutput`。

```python
InProcessCodeTool(globals_provider)
```

`run(code)` 在进程内执行代码并返回结果字典；`as_openjiuwen_tool(**kwargs)` 构造对应的 `LocalFunction`。调用方必须提供每次执行时返回全局变量字典的 `globals_provider`。
