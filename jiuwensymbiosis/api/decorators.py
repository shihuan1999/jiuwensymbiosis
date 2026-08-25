# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""What an action promises (`ActionSpec`), what a method carries (`ToolMeta`), and the
JSON-Schema generation both rely on.

Two types, one field list:

* `ActionSpec` — the contract: name, capability gate, parameters, result shape,
  pre-conditions and effects. Body-agnostic, so one action has exactly one of these.
  The 30 that make up the shared vocabulary are declared in `api/actions.py`.
* `ToolMeta` — what `@implements(SPEC)` attaches to a method: that contract plus
  `input_params`, the call schema derived from *this body's* signature. The call schema
  cannot live on the spec, because three bodies implement the same action with different
  signatures. Everything else is forwarded from the spec, so the contract is declared
  once and cannot drift from what the planner reads.

`@implements(SPEC)` (api/actions.py) is the only thing that attaches a `ToolMeta`, so a
method either fulfils a declared action or is not a tool at all. `build_robot_tools(api)`
walks the api instance, picks the decorated methods up, and binds them into
`LocalFunction`s using `ToolMeta.input_params`.

Type → JSON-Schema mapping is intentionally minimal; if you need richer schemas (enums,
regex, nested objects), refine them with `ActionSpec.param_schema`.

Beyond the call schema, a tool carries its **planning contract** — pre-conditions
and effects, never an order, so a planner can *derive* a legal order instead of
following one written down in prose:

* `result` — what fields the result exposes, so a later step can read `<bind>.field`.
  A `TypedDict` (or a union of them), turned into a schema by `result_schema()`.
* `requires` / `provides` / `invalidates` — robot self-state, over the closed
  vocabulary in `jiuwensymbiosis.api.state`.
* `produces_location` / `consumes_location` / `invalidates_locations` — location
  freshness, scoped to the step's `bind` (see `api.state`). An action that senses
  where something is *produces*; one that moves the base *invalidates* every prior
  location, since they were measured from the old standpoint.

  `consumes_location` is only for actions that read a location **implicitly**,
  off a cache, without naming it in their params. A step that passes
  `<bind>.field` already declares its dependency through that reference, and the
  sequence validator checks its freshness from the reference alone — annotating
  it would wrongly force a location on the same action when it is called with
  plain literal coordinates.
"""

from __future__ import annotations

import inspect
import logging
import types
import typing
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any, get_args, get_origin, get_type_hints

from jiuwensymbiosis.api.state import validate_tokens
from jiuwensymbiosis.env.base import KNOWN_CAPABILITIES

logger = logging.getLogger(__name__)


class UnknownCapability(ValueError):
    """A spec declared a capability outside ``KNOWN_CAPABILITIES``."""


@dataclass(frozen=True)
class ActionSpec:
    """What one action is: its name, gate, parameters, result shape and contract.

    Body-agnostic, and the ONLY place these fields are declared — ``ToolMeta`` holds a
    spec rather than copying it, so what a planner reads cannot drift from what the
    vocabulary promises.

    Attributes:
        name: the action name a plan writes as ``op``. Unique across the vocabulary.
        description: what the action does, what it needs and what it establishes.
            This is the ONLY text a planner sees, so it must stay true on every body:
            an implementation has no channel for adding prose of its own.
        capability: the capability gate. ``None`` = every body has it (``home``).
        params: parameter names a planner may pass. Every vocabulary entry states this,
            so a body that adds a parameter of its own does not advertise it and no plan
            can come to depend on something the next robot lacks. ``None`` = advertise
            whatever the implementation's signature takes, which is only right for a
            one-off spec declared inline next to its single implementation.
        required_params: the subset that must be passed. Everything else is optional.
        param_schema: per-parameter JSON Schema refining what the signature can express
            — e.g. the field names inside a ``pose`` dict, which every 6-DoF body agrees
            on and a bare ``dict`` annotation would lose. Merged over the derived schema.
        result: a ``TypedDict`` (or a union of them) describing the result fields, so
            a later step can read ``<bind>.field`` and the validator can reject a
            field that does not exist. ``None`` = shape unknown → field checking is
            skipped rather than everything being rejected.
        requires / provides / invalidates: robot self-state, over the closed
            vocabulary in ``api/state.py``.
        produces_location / consumes_location / invalidates_locations: location
            freshness (see ``api/state.py``).
        opens_access / closes_access: this action clears (or restores) whatever blocks
            the thing it acts on — a door, a lid, a stacked crate, a lock. Only the
            barrier side declares anything; which steps reach *through* a barrier is
            derived from the sequence (see ``api/state.py``).
        tags: rail triggers (``VisualFeedbackRail`` / ``RecoveryRail`` watch these).
            NOT a visibility mechanism — that is ``planner_visible``.
        planner_visible: whether the action enters the planner's vocabulary. ``False``
            for bring-up / calibration / diagnostic actions, which stay callable
            through ``robot_control`` and from scripts but must not compete for the
            planner's attention.
    """

    name: str
    description: str
    capability: str | None = None
    params: tuple[str, ...] | None = None
    required_params: tuple[str, ...] = ()
    param_schema: Mapping[str, Any] | None = None
    result: Any = None
    requires: tuple[str, ...] = ()
    provides: tuple[str, ...] = ()
    invalidates: tuple[str, ...] = ()
    produces_location: bool = False
    consumes_location: bool = False
    invalidates_locations: bool = False
    opens_access: bool = False
    closes_access: bool = False
    tags: tuple[str, ...] = ()
    planner_visible: bool = True

    def __post_init__(self) -> None:
        """Validate the spec against the closed vocabularies it draws on."""
        if self.capability is not None and self.capability not in KNOWN_CAPABILITIES:
            raise UnknownCapability(
                f"action {self.name!r} declares unknown capability {self.capability!r}. "
                f"Add it to KNOWN_CAPABILITIES in jiuwensymbiosis/env/base.py first."
            )
        for field_name in ("requires", "provides", "invalidates"):
            validate_tokens(getattr(self, field_name), field=field_name, owner=f"action {self.name}")
        unknown = set(self.required_params) - set(self.params or ())
        if self.params is not None and unknown:
            raise ValueError(f"action {self.name!r}: required_params {sorted(unknown)} are not in params")

    def result_schema(self) -> dict[str, Any]:
        """JSON Schema for ``result``; ``{}`` when the shape is unknown.

        A ``TypedDict`` is the form to prefer — the type checker sees it too. A mapping is
        taken as an already-written schema, for a result whose shape no annotation captures.
        """
        if isinstance(self.result, Mapping):
            return dict(self.result)
        return schema_from_typeddict(self.result)


@dataclass
class ToolMeta:
    """What ``@implements(SPEC)`` attaches to one body's method: the contract it fulfils
    plus this body's call schema.

    Every contract field is forwarded from ``spec`` rather than copied, so there is one
    field list in the codebase. ``input_params`` is the only thing that is genuinely
    per-method: three bodies implement the same action with different signatures.
    An empty ``returns`` means "output shape unknown" — consumers must degrade to no
    field checking rather than reject.
    """

    spec: ActionSpec
    input_params: dict[str, Any]

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def description(self) -> str:
        return self.spec.description

    @property
    def capability(self) -> str | None:
        return self.spec.capability

    @property
    def tags(self) -> list[str]:
        """Rail triggers. A list, because rails and tests read it as one."""
        return list(self.spec.tags)

    @property
    def returns(self) -> dict[str, Any]:
        return self.spec.result_schema()

    @property
    def requires(self) -> tuple[str, ...]:
        return self.spec.requires

    @property
    def provides(self) -> tuple[str, ...]:
        return self.spec.provides

    @property
    def invalidates(self) -> tuple[str, ...]:
        return self.spec.invalidates

    @property
    def produces_location(self) -> bool:
        return self.spec.produces_location

    @property
    def consumes_location(self) -> bool:
        return self.spec.consumes_location

    @property
    def invalidates_locations(self) -> bool:
        return self.spec.invalidates_locations

    @property
    def opens_access(self) -> bool:
        return self.spec.opens_access

    @property
    def closes_access(self) -> bool:
        return self.spec.closes_access

    @property
    def planner_visible(self) -> bool:
        return self.spec.planner_visible

    def result_fields(self) -> frozenset[str]:
        """Field names ``returns`` advertises; empty when the shape is unknown."""
        props = self.returns.get("properties")
        return frozenset(props) if isinstance(props, dict) else frozenset()

    def full_description(self) -> str:
        """What a planner reads. Every word of it comes from the ActionSpec: a body has no
        channel for adding prose of its own, because a fact that changes a plan and is true
        of only one robot means the action is not the same action there.
        """
        return self.description


_BASIC_TYPES = {
    int: "integer",
    float: "number",
    str: "string",
    bool: "boolean",
    list: "array",
    dict: "object",
}


def _annotation_to_schema(ann: Any) -> dict[str, Any]:
    """Best-effort conversion of a Python type annotation to a JSON Schema fragment."""
    if ann is inspect.Parameter.empty or ann is Any:
        return {}
    if ann in _BASIC_TYPES:
        return {"type": _BASIC_TYPES[ann]}

    origin = get_origin(ann)
    args = get_args(ann)

    # ``Literal[...]`` → an enum. This is how one shared parameter carries per-body legal
    # values: the contract names the parameter, and each body's own signature says which
    # values IT accepts, so a planner reads the truth for the robot in front of it.
    if origin is typing.Literal:
        schema: dict[str, Any] = {"enum": list(args)}
        types_seen = {_BASIC_TYPES[type(a)] for a in args if type(a) in _BASIC_TYPES}
        if len(types_seen) == 1:
            schema["type"] = types_seen.pop()
        return schema

    if origin in (typing.Union, types.UnionType):
        non_none = [a for a in args if a is not type(None)]
        if len(non_none) == 1:
            return _annotation_to_schema(non_none[0])
        return {"oneOf": [_annotation_to_schema(a) for a in non_none]}

    if origin in (list, tuple):
        if args:
            return {"type": "array", "items": _annotation_to_schema(args[0])}
        return {"type": "array"}

    if origin is dict:
        return {"type": "object"}

    return {}


def _resolve_hints(func: Callable) -> dict[str, Any]:
    """Resolve ``func``'s annotations to real type objects.

    Handles ``from __future__ import annotations`` (which makes annotations
    string literals at runtime) by calling ``typing.get_type_hints`` with
    progressively wider namespaces. Falls back to whatever
    ``func.__annotations__`` already holds (may be strings) if everything
    else fails — the schema for those params will degrade to ``{}``.
    """
    # First try the standard call — works for fully-qualified annotations.
    try:
        return get_type_hints(func)
    except Exception as e:
        logger.debug("get_type_hints(%s) failed: %s; trying fallback.", func.__name__, e)
    # Try with the function's own module globals + builtins (helps for
    # locally defined functions whose forward refs reference globals).
    try:
        mod = inspect.getmodule(func)
        ns = getattr(mod, "__dict__", {}) if mod is not None else {}
        return get_type_hints(func, globalns=ns)
    except Exception as e:
        logger.debug(
            "get_type_hints(%s, globalns=...) failed: %s; falling back to raw __annotations__.",
            func.__name__,
            e,
        )
    # Last resort: raw __annotations__ (may still be strings).
    return getattr(func, "__annotations__", {}) or {}


def schema_from_signature(func: Callable) -> dict[str, Any]:
    """Public alias of :func:`_schema_from_signature` for the ``ActionSpec`` layer."""
    return _schema_from_signature(func)


def schema_from_typeddict(annotation: Any) -> dict[str, Any]:
    """JSON Schema for a ``TypedDict`` (or a union of them); ``{}`` when the shape is unknown.

    ``{}`` means "output shape unknown", which consumers must treat as "skip field
    checking" rather than "declares no fields".
    """
    if annotation is None:
        return {}
    members = (
        [a for a in get_args(annotation) if a is not type(None)]
        if get_origin(annotation) in (typing.Union, types.UnionType)
        else [annotation]
    )
    schemas = [_typeddict_schema(m) for m in members if typing.is_typeddict(m)]
    if not schemas:
        return {}
    return schemas[0] if len(schemas) == 1 else _merge_schemas(schemas)


def _merge_schemas(schemas: list[dict[str, Any]]) -> dict[str, Any]:
    """Union of several result shapes: properties merge, ``required`` intersects.

    A field is only *guaranteed* when every branch declares it — the success and the
    failure shape of one action rarely agree on more than ``ok``.
    """
    properties: dict[str, Any] = {}
    for s in schemas:
        properties.update(s.get("properties", {}))
    merged: dict[str, Any] = {"type": "object", "properties": properties}
    required = set(schemas[0].get("required", []))
    for s in schemas[1:]:
        required &= set(s.get("required", []))
    if required:
        merged["required"] = sorted(required)
    return merged


def _schema_from_signature(func: Callable) -> dict[str, Any]:
    """Build a JSON-Schema-style ``input_params`` from a function signature.

    Returns ``{"type":"object","properties":{...},"required":[...]}``.
    """
    sig = inspect.signature(func)
    hints = _resolve_hints(func)
    properties: dict[str, Any] = {}
    required: list[str] = []
    for name, param in sig.parameters.items():
        if name == "self":
            continue
        if param.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD):
            continue
        ann = hints.get(name, param.annotation)
        prop = _annotation_to_schema(ann)
        if param.default is not inspect.Parameter.empty:
            prop["default"] = param.default
        else:
            required.append(name)
        # Fall back to string only when we truly couldn't infer anything;
        # never overwrite a populated schema (e.g. {"default": None}).
        if not prop:
            prop = {"type": "string"}
        properties[name] = prop
    schema: dict[str, Any] = {"type": "object", "properties": properties}
    if required:
        schema["required"] = required
    return schema


def _typeddict_schema(td: Any) -> dict[str, Any]:
    """JSON-Schema fragment for one ``TypedDict`` class."""
    hints = get_type_hints(td)
    properties = {key: _annotation_to_schema(ann) or {"type": "string"} for key, ann in hints.items()}
    schema: dict[str, Any] = {"type": "object", "properties": properties}
    required = sorted(getattr(td, "__required_keys__", frozenset()))
    if required:
        schema["required"] = required
    return schema
