# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Deterministic trajectory expansion (joint + cartesian interpolation).

Calibration replays an operator-confirmed sparse trajectory by *deterministic
interpolation* — this is NOT motion-safety planning. The calibration layer does
not read soft limits, workspace bounds or reachability; the Driver/controller is
the sole authority on hardware motion safety and may refuse a segment, in which
case the run aborts (no auto-homing, no direction retry).

Joint interpolation:

* No ``limits`` parameter — :func:`interpolate_joint_path` /
  :func:`interpolate_joint_sequence` accept ``periodic_policy`` instead.
* ``shortest_arc`` (default): pick the wrap direction with the smallest
  magnitude; an exact half-rev resolves to the positive direction (deterministic
  tie-break).
* ``stored_coordinates``: always use ``b - a`` even for periodic joints.
* Step count uses ``max(abs(delta_deg))`` over all joints — never signed maximum
  (an all-negative delta still densifies correctly).

Cartesian interpolation: Slerp on rotation + linear translation; pure SE(3) data
validation only (shape/finiteness/homogeneous/SO(3)) — no workspace or IK.

The same inputs + algorithm version + parameters MUST produce the same dense
trajectory (reproducibility is a data contract, not a safety claim).
"""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation

from jiuwensymbiosis.calibration.domain.models import require_se3_mm
from jiuwensymbiosis.utils.geometry import make_transform

PeriodicPolicy = Literal["shortest_arc", "stored_coordinates"]
TrajectorySpace = Literal["joint", "cartesian"]


def _joint_period(unit: str) -> float:
    """Full revolution in the joint's native unit (deg or rad)."""
    if unit == "deg":
        return 360.0
    if unit == "rad":
        return 2.0 * math.pi
    raise ValueError(f"unit must be 'deg' or 'rad', got {unit!r}")


def _to_deg(value: float, unit: str) -> float:
    """Convert a native-unit joint quantity to degrees (for step sizing)."""
    if unit == "deg":
        return float(value)
    if unit == "rad":
        return float(np.degrees(value))
    raise ValueError(f"unit must be 'deg' or 'rad', got {unit!r}")


def _joint_candidate_deltas(a: float, b: float, *, periodic: bool, unit: str) -> list[float]:
    """Signed candidate deltas from a to b for one joint (native unit).

    Non-periodic: the single delta ``b - a``.
    Periodic: the half-open wrap ``wrap(b-a)`` always maps an exact half-rev to
    ``-period/2``; the other direction is then ``+period/2``. Both half-rev
    candidates are returned so the tie-break rule applies deterministically.
    """
    a_f, b_f = float(a), float(b)
    if not periodic:
        return [b_f - a_f]
    period = _joint_period(unit)
    half = period / 2.0
    d = (b_f - a_f + half) % period - half  # half-open: exact half-rev -> -period/2
    if d == 0.0:
        return [0.0]
    d_alt = d - math.copysign(period, d)  # the other wrap direction
    return [d, d_alt]


def _select_joint_delta(
    a: float,
    b: float,
    *,
    name: str,
    periodic: bool,
    unit: str,
    policy: PeriodicPolicy,
) -> float:
    """Pick the candidate delta per ``periodic_policy``.

    ``shortest_arc``: smallest magnitude (positive tie-break on a half-rev).
    ``stored_coordinates``: always ``b - a`` even for periodic joints.
    """
    if policy == "stored_coordinates":
        return float(b) - float(a)
    candidates = _joint_candidate_deltas(a, b, periodic=periodic, unit=unit)
    # Smallest magnitude; positive direction wins a half-rev tie deterministically.
    candidates.sort(key=lambda d: (abs(d), -d))
    return candidates[0]


def interpolate_joint_path(
    q0: np.ndarray,
    q1: np.ndarray,
    *,
    unit: str,
    order: tuple[str, ...],
    periodic: tuple[bool, ...],
    periodic_policy: PeriodicPolicy = "shortest_arc",
    max_step_deg: float,
) -> list[np.ndarray]:
    """Densify a single joint-space segment into a list of waypoint vectors.

    No ``limits`` parameter; no soft-limit read. ``periodic_policy`` selects
    the wrap rule for periodic joints. Step count uses ``max(abs(delta_deg))``
    over all joints (signed maximum would under-densify all-negative deltas).
    """
    if unit not in ("deg", "rad"):
        raise ValueError(f"unit must be 'deg' or 'rad', got {unit!r}")
    a = np.asarray(q0, dtype=np.float64)
    b = np.asarray(q1, dtype=np.float64)
    if a.shape != b.shape:
        raise ValueError(f"joint vectors must match shape, got {a.shape} and {b.shape}")
    if len(a) != len(order) or len(a) != len(periodic):
        raise ValueError(f"joint vector length {len(a)} != order {len(order)} / periodic {len(periodic)}")

    deltas = np.empty(len(a), dtype=np.float64)
    for i, name in enumerate(order):
        deltas[i] = _select_joint_delta(
            float(a[i]), float(b[i]), name=name, periodic=periodic[i], unit=unit, policy=periodic_policy
        )

    # Step sizing in a single degree frame: convert native-unit deltas to degrees,
    # then take max(abs(...)) over all joints (signed maximum would under-densify
    # all-negative deltas — §7.2).
    max_delta_deg = float(max((abs(_to_deg(float(d), unit)) for d in deltas), default=0.0))
    steps = max(1, int(math.ceil(max_delta_deg / max(float(max_step_deg), 1e-9))))
    out = [a.copy()]
    for s in range(1, steps + 1):
        alpha = s / steps
        out.append(a + deltas * alpha)
    return out


def interpolate_joint_sequence(
    points: np.ndarray,
    *,
    unit: str,
    order: tuple[str, ...],
    periodic: tuple[bool, ...],
    periodic_policy: PeriodicPolicy = "shortest_arc",
    max_step_deg: float,
) -> list[np.ndarray]:
    """Densify an ordered taught joint path into a dense waypoint list.

    Walks consecutive pairs via :func:`interpolate_joint_path`, concatenating
    the intermediate points (each pair's start is the previous pair's end, so
    we drop the duplicated start except for the very first).
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 2:
        raise ValueError(f"points must be (N, n_joints) with N>=2, got {pts.shape}")
    dense: list[np.ndarray] = []
    for i in range(len(pts) - 1):
        seg = interpolate_joint_path(
            pts[i],
            pts[i + 1],
            unit=unit,
            order=order,
            periodic=periodic,
            periodic_policy=periodic_policy,
            max_step_deg=max_step_deg,
        )
        dense.extend(seg if i == 0 else seg[1:])
    return dense


def interpolate_cartesian_path(
    tf_a: np.ndarray,
    tf_b: np.ndarray,
    *,
    max_step_mm: float,
    max_step_deg: float,
) -> list[np.ndarray]:
    """Densify a cartesian SE(3) segment: Slerp on rotation + linear translation.

    Steps = ceil(max(translation_mm / max_step_mm, rotation_deg / max_step_deg));
    translation in mm, rotation in deg (dimensioned separately, never mixed).
    No workspace/IK/collision check — pure SE(3) data expansion.
    """
    a = require_se3_mm(tf_a, field="tf_a")
    b = require_se3_mm(tf_b, field="tf_b")
    ra = Rotation.from_matrix(a[:3, :3])
    rb = Rotation.from_matrix(b[:3, :3])
    rel = ra.inv() * rb
    rot_deg = float(np.linalg.norm(rel.as_rotvec(degrees=True)))
    trans_mm = float(np.linalg.norm(b[:3, 3] - a[:3, 3]))
    steps = max(
        1,
        int(
            math.ceil(
                max(
                    trans_mm / max(float(max_step_mm), 1e-9),
                    rot_deg / max(float(max_step_deg), 1e-9),
                )
            )
        ),
    )
    out = [a.copy()]
    rel_rotvec_deg = rel.as_rotvec(degrees=True)
    for s in range(1, steps + 1):
        alpha = s / steps
        scaled = Rotation.from_rotvec(rel_rotvec_deg * alpha, degrees=True)
        rot = (ra * scaled).as_matrix()
        t = a[:3, 3] + (b[:3, 3] - a[:3, 3]) * alpha
        out.append(make_transform(rot, t))
    return out


__all__ = [
    "PeriodicPolicy",
    "TrajectorySpace",
    "_joint_candidate_deltas",
    "_joint_period",
    "_select_joint_delta",
    "_to_deg",
    "interpolate_cartesian_path",
    "interpolate_joint_path",
    "interpolate_joint_sequence",
]
