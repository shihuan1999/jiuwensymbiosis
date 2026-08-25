# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""``Chain.limits()`` is what a body's ``joint_limits`` is built from, and therefore what
decides whether SafetyRail range-checks a joint command at all.

It is a METHOD, not a property: reading ``chain.limits`` instead of calling it yields a bound
method, which is truthy, non-empty-looking, and silently useless as a limits mapping — an
adapter that made that mistake reported "limits available" while every range check was
skipped. These tests run against the SO-101 description that ships in the package, so the
contract is verified on any machine, with no robot description checked out.
"""

from __future__ import annotations

from pathlib import Path

from jiuwensymbiosis.kinematics.urdf_chain import parse_chain

URDF = str(Path(__file__).resolve().parents[3]
           / "jiuwensymbiosis" / "adapters" / "so101" / "description" / "so101_new_calib.urdf")
REVOLUTE = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}


class TestChainLimits:
    def test_returns_a_mapping_not_a_bound_method(self):
        limits = parse_chain(URDF, "base_link", "gripper_frame_link").limits()
        assert isinstance(limits, dict)
        assert limits, "empty limits would disable every range check downstream"

    def test_covers_every_movable_joint(self):
        limits = parse_chain(URDF, "base_link", "gripper_frame_link").limits()
        assert set(limits) == REVOLUTE

    def test_omits_fixed_joints(self):
        """A fixed joint has no range to state, so naming one would invent a limit."""
        limits = parse_chain(URDF, "base_link", "gripper_frame_link").limits()
        assert "gripper_frame_joint" not in limits

    def test_each_entry_is_an_ordered_lo_hi_pair(self):
        limits = parse_chain(URDF, "base_link", "gripper_frame_link").limits()
        for name, bounds in limits.items():
            lo, hi = bounds
            assert lo < hi, f"{name}: lower {lo} is not below upper {hi}"
