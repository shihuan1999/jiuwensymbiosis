# Robot Environment, Capability, and Tool API Reference

> Category: Reference. The [Chinese source](../../zh/reference/robot-api.md) is authoritative. This page is based on the
> public code of `jiuwensymbiosis.env`, `jiuwensymbiosis.api`, and `jiuwensymbiosis.tools`.

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

Pose fields follow the adapter convention: SCARA commonly uses `x/y/z/r`, while a six-axis body uses `x/y/z/rx/ry/rz`. Joint units must match the Env's convention (`env.joint_units`; unstated means unknown). Camera depth is normally represented in metres at acquisition boundaries.

## `BaseRobotEnv`

Subclasses implement:

```python
connect() -> None
disconnect() -> None
get_observation() -> RobotObservation
home() -> None
```

`home` is abstract because it backs the one unconditional action (`ActionSpec(capability=None)`): a Cartesian default would smuggle `motion.cartesian` into bodies the capability gate cannot stop. Optional lifecycle hooks are `reset()` and `emergency_stop()`. Common contract properties include `capabilities`, `low_level`, `z_min_safe`, `workspace_bounds`, `joint_limits`, `home_pose`, `tool_offset_mm`, `joint_units`, `default_orientation_policy`, `base_step_limits`, `lift_limits`, `waist_step_limit_rad`, `cameras`, and `urdf_path`/`arm_chains`/`arm_joints`.

`has(capability)` checks the declaration. An unknown capability raises `ValueError` when the subclass is created.

## `BaseRobotApi` and the action vocabulary

`BaseRobotApi(env)` retains the Env reference and carries an `@implements` in two places:

- `home` (the one unconditional action; overrides to delegate `env.home()`)
- each body's additional actions

The `capabilities` property is **derived from the specs of the actions a body implements** (each `@implements` contributes its `ActionSpec`'s capability), plus any declared marker-capability class attribute (`motion.servo`, `planning.reachability` — which have no corresponding action). **Implementing an action automatically grants its capability**, and never advertises a capability the body has not got.

The action vocabulary is in `jiuwensymbiosis/api/actions.py`. Each action declares its description, capability gate, parameter names, result shape, `requires`/`provides`/`invalidates`, `produces_location`/`consumes_location`/`invalidates_locations`, and `planner_visible`. Generic implementations (one line of delegation to an Env verb) live in `api/defaults.py`, which an adapter takes as needed.

## Known capabilities

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

The source of truth is `jiuwensymbiosis.env.base.KNOWN_CAPABILITIES`.

## `@implements(SPEC)`

```python
implements(spec: ActionSpec) -> Callable   # api/actions.py
```

The one way a method becomes a tool. It attaches a `ToolMeta` holding `spec` plus `input_params`, the call schema derived from this body's signature (filtered to `spec.params`, refined by `spec.param_schema`). Every contract field — description, capability, tags, requires / provides / invalidates, location freshness — is read back off the spec, so an implementation has no channel for saying something the vocabulary does not.

A signature that cannot accept a parameter the spec promises raises `ContractViolation` at import time.

Bring-up, calibration and debug views are **not** actions: leave them undecorated and drive them from `scripts/`.

## The action contract (callable, and plannable)

Each `ActionSpec` additionally carries a planning contract that the two-tier planner uses to derive a legal order:

- `result` — JSON Schema of result fields, auto-derived from a `TypedDict` (or a union); the authoritative source is `jiuwensymbiosis/contracts.py` (owned by no layer), with success/failure shapes merged
- `requires`/`provides`/`invalidates` — robot self-state, over `api/state.py:KNOWN_STATE_TOKENS` (`payload.held`/`payload.clear`/`payload.stowed`/`body.home`)
- `produces_location`/`consumes_location`/`invalidates_locations` — location freshness (an action that senses where something is *produces*; one that moves the base *invalidates* every prior location)

A contract never encodes an order; `parse_sequence` rejects only permutations whose pre-conditions do not hold. `WorldState.snapshot(session)` reports the same vocabulary at runtime (observation overrides belief; an absent token is unknown, never false).

## Tool generation

```python
build_robot_tools(api, *, env=None, allow=None, deny=None) -> list[Any]
list_tool_meta(api, *, env=None) -> list[dict]
```

When an Env is supplied, generated tools are gated by `api.capabilities ∩ env.capabilities`. `allow` and `deny` filter by tool name. The capability comes from the action's own `ActionSpec`, never from whichever class declares the method.

## Aggregated and code tools

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

`available_actions` lists dispatchable actions. `invoke({"action": ..., "params": {...}})` returns a `ToolOutput`. Safety rails transparently unwrap `action`/`params`.

```python
InProcessCodeTool(globals_provider)
```

`run(code)` executes Python in process and returns a result dictionary. `as_openjiuwen_tool(**kwargs)` creates a `LocalFunction`. `globals_provider` must return a fresh globals mapping for each execution.
