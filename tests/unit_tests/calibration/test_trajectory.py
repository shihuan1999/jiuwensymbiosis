# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""§11 acceptance cases #25-#26: trajectory semantics (§7.2).

``shortest_arc`` and ``stored_coordinates`` behave differently on a periodic
joint, and step count uses ``max(abs(delta_deg))`` over all joints.
"""

from __future__ import annotations

import inspect

import numpy as np

from jiuwensymbiosis.calibration.domain.trajectory import interpolate_joint_path, interpolate_joint_sequence


def test_interpolation_api_does_not_accept_safety_limits():
    """Soft-limit enforcement belongs to the Driver/controller boundary."""
    for function in (interpolate_joint_path, interpolate_joint_sequence):
        assert "limits" not in inspect.signature(function).parameters


class TestPeriodicPolicy:
    """#25: shortest_arc wraps; stored_coordinates does not."""

    def _pair(self, a_deg: float, b_deg: float, periodic: bool):
        q0 = np.array([a_deg], dtype=float)
        q1 = np.array([b_deg], dtype=float)
        return q0, q1

    def test_shortest_arc_picks_short_wrap(self):
        # 10deg -> 350deg on a periodic joint: shortest_arc delta = -20deg (wrap),
        # not +340deg.
        q0, q1 = self._pair(10.0, 350.0, periodic=True)
        pts = interpolate_joint_path(
            q0,
            q1,
            unit="deg",
            order=("j1",),
            periodic=(True,),
            periodic_policy="shortest_arc",
            max_step_deg=5.0,
        )
        # The final point is 10 + (-20) = -10 (≡ 350 mod 360).
        assert len(pts) >= 2
        final = float(pts[-1][0])
        assert abs(final - (-10.0)) < 1e-9 or abs((final - 350.0) % 360.0) < 1e-9

    def test_stored_coordinates_no_wrap(self):
        # Same pair under stored_coordinates: delta = +340deg (no wrap).
        q0, q1 = self._pair(10.0, 350.0, periodic=True)
        pts = interpolate_joint_path(
            q0,
            q1,
            unit="deg",
            order=("j1",),
            periodic=(True,),
            periodic_policy="stored_coordinates",
            max_step_deg=5.0,
        )
        assert len(pts) >= 2
        # stored_coordinates walks linearly 10 -> 350, so there are ~68 steps (340/5).
        assert len(pts) > 60
        assert abs(float(pts[-1][0]) - 350.0) < 1e-9

    def test_non_periodic_ignores_policy(self):
        # On a non-periodic joint, both policies give the signed delta.
        q0, q1 = self._pair(0.0, 50.0, periodic=False)
        for policy in ("shortest_arc", "stored_coordinates"):
            pts = interpolate_joint_path(
                q0,
                q1,
                unit="deg",
                order=("j1",),
                periodic=(False,),
                periodic_policy=policy,  # type: ignore[arg-type]
                max_step_deg=5.0,
            )
            assert abs(float(pts[-1][0]) - 50.0) < 1e-9


class TestStepCountMaxAbsDelta:
    """#26: step count uses max(abs(delta_deg)) over all joints."""

    def test_all_negative_delta_densifies_same_as_positive(self):
        # -100deg path and +100deg path must yield the same step count.
        order = ("j1",)
        periodic = (False,)
        pos = interpolate_joint_path(
            np.array([0.0]),
            np.array([100.0]),
            unit="deg",
            order=order,
            periodic=periodic,
            max_step_deg=5.0,
        )
        neg = interpolate_joint_path(
            np.array([0.0]),
            np.array([-100.0]),
            unit="deg",
            order=order,
            periodic=periodic,
            max_step_deg=5.0,
        )
        assert len(pos) == len(neg)

    def test_multi_joint_uses_largest_abs(self):
        # joint0 delta=10deg, joint1 delta=50deg -> steps driven by 50deg.
        q0 = np.array([0.0, 0.0])
        q1 = np.array([10.0, 50.0])
        pts = interpolate_joint_path(
            q0,
            q1,
            unit="deg",
            order=("j1", "j2"),
            periodic=(False, False),
            max_step_deg=5.0,
        )
        # 50deg / 5deg-per-step = 10 steps -> 11 points.
        assert len(pts) == 11
