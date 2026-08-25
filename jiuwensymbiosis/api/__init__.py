# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""The action layer: what a robot can be asked to do, and where the doing comes from.

**Two things live here: the contract, and implementations of it.**

===========================================  ==========================================
I want to…                                   Use
===========================================  ==========================================
declare an action                            add an ``ActionSpec`` to ``actions.py``
implement one                                ``@implements(SPEC)`` on YOUR adapter method,
                                             forwarding to ``defaults.<action>(self, ...)``
                                             when the body has nothing of its own to say
bring-up, calibration, a debug view          none of the above — it is not an action.
                                             A plain method plus a script in ``scripts/``
===========================================  ==========================================

**Every action reaches its implementation the same way**: adapter method → ``defaults`` →
the shared code (``perception/`` or ``motion/``). There is no second path. Three approach
actions used to take one — through an ``Approach`` component the adapter held, which the
drive loops then read as a fifteen-member interface — and being the only three that did was
the whole cost: a reader had to learn two shapes to follow thirty actions.

There is no component layer left. What an adapter once held now lives beside the algorithm
it drives — 3-D sensing in ``perception/scene3d.py``, base approach in ``motion/approach.py``
— and each resolves its own body hooks off the ``api`` it is handed. A body customises one
by defining the hook (``detector_seg_fn``, ``base_driver``, …), not by subclassing or
holding anything.

**One contract, one carrier.** ``ActionSpec`` (declared in ``decorators.py``, next to the
carrier) says what an action is; ``ToolMeta`` is what ``@implements`` pins to a method —
that spec plus ``input_params``, the call schema derived from *this* body's signature.
``ToolMeta`` holds its spec rather than copying it, so the contract fields are written
once and a planner cannot be reading a different answer from the vocabulary.

**There is no second decorator.** A method with no ``ActionSpec`` is not a tool: the two
agent paths both build their tool list with ``planner_only=True``, so a "body-specific
tool" was never reachable by anything but a hand-written script — which is what a
bring-up or calibration routine should be anyway. If a body genuinely needs one, decide
first how it becomes reachable; do not reach for a decorator that hides the question.

**Nothing here is a base class.** ``defaults`` are free functions the adapter calls
explicitly and a component is an object it holds, so taking one action never drags in
its neighbours — and since ``BaseRobotApi.capabilities`` is derived from the actions a
body implements, it also never advertises hardware the body has not got.

**Only the adapter declares a contract.** ``defaults`` and the components are both pure
implementation: a component is HELD, so it is never in an Api's MRO and ``build_robot_tools``
never sees it. An ``@implements`` on a component method is therefore a second copy of the
contract that nothing reads. The state the components appear to hold (``last_detection`` /
``last_surface``) lives on ``BaseRobotApi`` — they read it through properties, because the
approach loops and the dual-arm grasp read it there too.
"""

from jiuwensymbiosis.api import defaults
from jiuwensymbiosis.api.actions import ACTIONS, ActionSpec, implements, planner_vocabulary
from jiuwensymbiosis.api.base import BaseRobotApi
from jiuwensymbiosis.api.decorators import ToolMeta
from jiuwensymbiosis.api.reachability import Reachability

__all__ = [
    "BaseRobotApi",
    # The shared action vocabulary — what an action IS.
    "ACTIONS",
    "ActionSpec",
    "implements",
    "planner_vocabulary",
    # What @implements pins to a method: the spec + this body's call schema.
    "ToolMeta",
    # Reusable implementations an adapter CALLS. Not a base class: a body takes one
    # action without taking its neighbours.
    "defaults",
    # Not an action implementation at all: a planning-time judge the planner reads
    # directly (api/reachability.py).
    "Reachability",
]
