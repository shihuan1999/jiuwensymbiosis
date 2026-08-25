# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""The shared vocabulary must not fork to track a body.

``ActionSpec`` exists so one action name means one thing on every robot. A body difference
is allowed to show up in exactly two ways:

1. **a vocabulary subset** — which actions this body offers (the capability gate decides);
2. **parameter values** — the same action name, with per-body enums / units / limits /
   defaults (``orientation_policy``'s ``Literal``, ``joint_units``, ``joint_limits``).

It is NOT allowed to show up as a **new action name**. When it does, a planner that "wants to
move a joint" must first know which robot it is standing in front of just to pick the verb —
which is the thing the shared vocabulary was built to remove.

The signature of a fork is easy to spot mechanically. Take one capability and the bodies that
declare it; compare the action names each implements under it:

* one set is a SUBSET of the other → a coverage gap. One name for the concept; some body
  simply has not implemented it. The gate tells a planner, and nothing is ambiguous.
* NEITHER is a subset of the other → each body has a name the other lacks under the same
  capability. That is the fork, and it is what this test refuses.

A failure here is not automatically a bug — two genuinely different actions can share a
capability. It is a demand for an explicit answer: either they are one concept and must
converge on one name, or they are two and the gate is too coarse to separate them.
"""

from __future__ import annotations

import importlib
import itertools

import pytest

from jiuwensymbiosis.api.actions import ACTIONS

# The shipped bodies. A new adapter belongs here the day it ships.
_BODIES = (("piper", "PiperApi", "PiperEnv"), ("so101", "So101Api", "So101Env"), ("cruzr", "CruzrApi", "CruzrEnv"))

# Known forks, each with the decision that has been made about it. Kept as an explicit
# waiver list rather than a silent skip: the entry is where the reasoning lives, and
# deleting it is what "we fixed it" looks like.
_ACCEPTED_FORKS: dict[str, str] = {
    "vision.detection": (
        "get_grasp_info_simple (piper/so101) vs locate_for_grasp/locate_for_place (cruzr). "
        "The contracts genuinely differ — one returns a ready-to-descend grasp_z for a single "
        "gripper, the other returns face normals and a landing footprint for a body that must "
        "square up and clamp. Under review: this may be a gate too coarse (the cruzr pair only "
        "makes sense with motion.base + grasp.dual_arm) rather than a naming fork."
    ),
}


def _bodies() -> dict[str, tuple[frozenset[str], set[str]]]:
    """``{body: (declared capabilities, implemented action names)}``."""
    out: dict[str, tuple[frozenset[str], set[str]]] = {}
    for name, api_cls, env_cls in _BODIES:
        api = getattr(importlib.import_module(f"jiuwensymbiosis.adapters.{name}.api"), api_cls)
        env = getattr(importlib.import_module(f"jiuwensymbiosis.adapters.{name}.env"), env_cls)
        implemented = {
            meta.name
            for cls in api.__mro__
            for value in vars(cls).values()
            if (meta := getattr(value, "__tool_meta__", None)) is not None
        }
        out[name] = (frozenset(env.capabilities), implemented)
    return out


def _capabilities_under_test() -> list[str]:
    """Capabilities at least two shipped bodies declare — only there can a fork exist."""
    bodies = _bodies()
    declared = {s.capability for s in ACTIONS.values() if s.capability}
    return sorted(cap for cap in declared if sum(1 for caps, _ in bodies.values() if cap in caps) >= 2)


@pytest.mark.parametrize("capability", _capabilities_under_test())
def test_no_body_specific_action_names_under_a_shared_capability(capability: str) -> None:
    bodies = _bodies()
    holders = {name: impl for name, (caps, impl) in bodies.items() if capability in caps}
    under_cap = {
        name: {a for a in impl if ACTIONS[a].capability == capability if a in ACTIONS}
        for name, impl in holders.items()
    }

    forks = [
        (a, b, sorted(under_cap[a] - under_cap[b]), sorted(under_cap[b] - under_cap[a]))
        for a, b in itertools.combinations(sorted(under_cap), 2)
        if not (under_cap[a] <= under_cap[b] or under_cap[b] <= under_cap[a])
    ]
    if not forks:
        assert capability not in _ACCEPTED_FORKS, (
            f"'{capability}' is on the accepted-forks list but no longer forks — delete the entry."
        )
        return

    detail = "; ".join(
        f"{a} has {only_a} that {b} lacks, {b} has {only_b} that {a} lacks" for a, b, only_a, only_b in forks
    )
    if capability in _ACCEPTED_FORKS:
        pytest.xfail(f"known fork under '{capability}': {_ACCEPTED_FORKS[capability]} ({detail})")
    pytest.fail(
        f"the vocabulary forks under '{capability}': {detail}.\n"
        "Two names for one concept, chosen by which robot you are standing in front of. Either "
        "converge them onto one action name (body difference belongs in the parameter values), or "
        "split the capability so the gate — not the name — carries the difference. If they really "
        "are two different actions, record why in _ACCEPTED_FORKS."
    )
