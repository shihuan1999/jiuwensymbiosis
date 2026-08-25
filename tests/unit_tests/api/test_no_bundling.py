# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Taking one sensing action must not mean taking its neighbours.

While the 3-D sensing was a base class, wanting ``locate_for_grasp`` meant inheriting
``locate_for_place`` and ``analyze_scene`` too — and, because capabilities are derived from
the actions a body implements, it also meant advertising a detector the body might not have.
That bundling is why piper and so101 never took the 3-D sensing at all. Holding the component
instead of inheriting it is what removes the bundle: the body declares what it offers, one
action at a time, and its api file is the whole list.
"""

from __future__ import annotations

from jiuwensymbiosis.api import defaults
from jiuwensymbiosis.api.actions import LOCATE_FOR_GRASP, implements
from jiuwensymbiosis.api.base import BaseRobotApi
from jiuwensymbiosis.env.mock import MockArmEnv


class OneActionBody(BaseRobotApi):
    """A body that wants the 3-D measurement and nothing else."""

    def __init__(self, env):
        super().__init__(env)

    @implements(LOCATE_FOR_GRASP)
    def locate_for_grasp(self, object_name: str = "box", reference: str | None = None,
                         relation: str = "on") -> dict:
        return defaults.locate_for_grasp(self, object_name, reference, relation)


def test_one_action_can_be_taken_alone():
    api = OneActionBody(MockArmEnv())
    assert hasattr(api, "locate_for_grasp")
    assert not hasattr(api, "locate_for_place"), "taking one must not drag in its neighbour"
    assert not hasattr(api, "analyze_scene")


def test_the_api_file_is_the_whole_list():
    """Every action a planner can see is declared in the class itself — none arrive by
    inheritance, so reading the adapter tells you exactly what the robot offers."""
    declared = {n for n in vars(OneActionBody) if hasattr(getattr(OneActionBody, n, None), "__tool_meta__")}
    inherited = {
        n
        for cls in OneActionBody.__mro__[1:]
        for n in vars(cls)
        if hasattr(getattr(cls, n, None), "__tool_meta__")
    }
    assert declared == {"locate_for_grasp"}
    assert inherited == {"home"}, f"only home is inherited (every body owes one); got {inherited}"


def test_capability_follows_the_action_not_the_component():
    """Reaching the shared pipeline is not evidence of a detector — implementing an action is."""
    api = OneActionBody(MockArmEnv())
    assert api.capabilities == frozenset({"vision.detection"})

    class HoldsButDeclaresNothing(BaseRobotApi):
        def __init__(self, env):
            super().__init__(env)

    assert HoldsButDeclaresNothing(MockArmEnv()).capabilities == frozenset()


class TestSharedImplementationsDeclareNoContract:
    """A shared implementation is REACHED, never inherited, so it is not in any Api's MRO and
    ``build_robot_tools`` never sees it. An ``@implements`` down there was therefore a second
    copy of the contract that nothing read — the same "declared twice" problem the ActionSpec
    vocabulary exists to remove, surviving in the last place.

    The contract is declared once, on the adapter method that forwards to ``api.defaults``."""

    def test_no_shared_implementation_carries_a_tool_meta(self):
        from jiuwensymbiosis.api import defaults
        from jiuwensymbiosis.motion import approach
        from jiuwensymbiosis.perception import scene3d

        offenders = [
            f"{mod.__name__}.{name}"
            for mod in (defaults, approach, scene3d)
            for name, value in vars(mod).items()
            if getattr(value, "__tool_meta__", None) is not None
        ]
        assert not offenders, (
            f"{offenders} declare a contract they cannot expose. Move the @implements onto "
            "the adapter method that forwards here."
        )

    def test_the_holder_declares_every_action_it_forwards(self):
        """What the component would have advertised must still be advertised — by the body."""
        from jiuwensymbiosis.adapters.cruzr.api import CruzrApi

        declared = {
            meta.name
            for value in vars(CruzrApi).values()
            if (meta := getattr(value, "__tool_meta__", None)) is not None
        }
        forwarded = {
            "locate_for_grasp", "locate_for_place", "analyze_scene",
            "search_target", "approach_for_grasp", "approach_for_place",
        }
        assert forwarded <= declared
