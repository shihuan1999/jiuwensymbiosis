# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Hardware ports owned by the in-tree calibration subsystem.

Calibration is the consumer of these interfaces, so it owns their vocabulary.
JiuwenSymbiosis Env/adapter implementations satisfy the protocols structurally;
the calibration workflow never reaches through them to ``env.low_level``.

The ports define the boundary inside the JiuwenSymbiosis distribution: the
calibration subsystem may depend on core utilities and adapter implementations,
while core modules must not import calibration.  Keeping that dependency
direction explicit lets the workflows stay hardware-agnostic without implying
that calibration is a separately installable package.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Protocol, runtime_checkable

import numpy as np

# Teaching-time hand guiding is a hardware capability, not a calibration concept:
# core owns the failure mode so an Env/driver can raise it without importing
# calibration. The calibration-facing name stays an alias so both sides of the
# boundary catch the same class.
from jiuwensymbiosis.env.protocol import HandGuidingRecoveryError as ManualGuidanceRecoveryError


@dataclass(frozen=True)
class CalibrationCameraFrame:
    """Camera observation required by calibration.

    Adapters may pre-resolve the target pose.  Otherwise the workflow uses its
    injected board detector.  Arrays are expressed in the camera convention;
    transform translations use millimetres throughout the subsystem.
    """

    rgb: np.ndarray
    intrinsics: np.ndarray
    distortion: np.ndarray | None
    captured_at_ns: int
    tf_cam_target: np.ndarray | None = None
    reproj_rms_px: float | None = None


@runtime_checkable
class CalibrationCaptureSource(Protocol):
    """Port that supplies calibration frames and declares camera topology."""

    @property
    def camera_mount(self) -> Literal["eye_in_hand", "eye_to_hand"]:
        pass

    def capture_calibration_frame(self) -> CalibrationCameraFrame:
        pass


@dataclass(frozen=True)
class JointState:
    """Body-agnostic joint snapshot with explicit coordinate metadata."""

    values: np.ndarray
    unit: Literal["deg", "rad"]
    order: tuple[str, ...]
    periodic: tuple[bool, ...]
    limits: dict[str, tuple[float, float]] | None = None

    def __post_init__(self) -> None:
        arr = np.asarray(self.values, dtype=np.float64).copy()
        if arr.ndim != 1:
            raise ValueError(f"JointState.values must be 1-D, got shape {arr.shape}.")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"JointState.values has non-finite value: {arr!r}.")
        arr.setflags(write=False)
        object.__setattr__(self, "values", arr)

        if self.unit not in ("deg", "rad"):
            raise ValueError(f"JointState.unit must be 'deg' or 'rad', got {self.unit!r}.")
        if not isinstance(self.order, tuple) or not all(isinstance(name, str) and name for name in self.order):
            raise ValueError(f"JointState.order must be a tuple of non-empty strings, got {self.order!r}.")
        if len(self.order) != len(arr):
            raise ValueError(f"JointState order length {len(self.order)} != values length {len(arr)}.")
        if not isinstance(self.periodic, tuple) or not all(isinstance(value, bool) for value in self.periodic):
            raise ValueError(f"JointState.periodic must be a tuple of bools, got {self.periodic!r}.")
        if len(self.periodic) != len(arr):
            raise ValueError(f"JointState periodic length {len(self.periodic)} != values length {len(arr)}.")
        if self.limits is None:
            return
        if not isinstance(self.limits, dict):
            raise ValueError("JointState.limits must be a dict or None.")
        if set(self.limits) != set(self.order):
            raise ValueError(
                f"JointState.limits keys {sorted(self.limits)} must match order keys {sorted(self.order)}."
            )
        for name, pair in self.limits.items():
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                raise ValueError(f"JointState.limits['{name}'] must be a (lo, hi) pair, got {pair!r}.")
            lo, hi = pair
            if not (np.isfinite(lo) and np.isfinite(hi)) or float(lo) > float(hi):
                raise ValueError(f"JointState.limits['{name}'] must be ordered finite (lo<=hi), got {pair!r}.")


@runtime_checkable
class CalibrationPoseSource(Protocol):
    """Port that reads flange-in-base SE(3), with translation in millimetres."""

    def get_flange_transform_mm(self) -> np.ndarray:
        pass


@runtime_checkable
class JointCalibrationMotion(CalibrationPoseSource, Protocol):
    """Joint-space controlled calibration motion.

    ``move_joint_path`` is OPTIONAL: when a device exposes it, the workflow
    streams a dense waypoint path and settles only at the end (avoiding
    per-point settle on slew-limit intermediates).  Devices without it fall
    back to per-point ``move_joint_vector`` — detected via ``getattr`` in the
    workflow, not via this Protocol, so existing adapters need no change.
    """

    def get_joint_state(self) -> JointState:
        pass

    def move_joint_vector(self, q: np.ndarray) -> None:
        pass


@runtime_checkable
class CartesianCalibrationMotion(CalibrationPoseSource, Protocol):
    """Cartesian controlled calibration motion in base-frame millimetre SE(3)."""

    def move_to_flange_transform_mm(self, tf: np.ndarray) -> None:
        pass


@runtime_checkable
class ManualGuidance(Protocol):
    """Optional context-managed hand-guidance port used while teaching poses."""

    def manual_guidance(self):
        """Release the arm for teaching and restore a controllable state on exit."""
        pass


@runtime_checkable
class GuidanceHold(Protocol):
    """Optional pause port: hold / release the arm *inside* a manual-guidance context.

    Hand-guiding is tiring — the operator must support a limp arm the whole time.
    A device that can re-engage torque at the current pose lets them let go and rest
    without ending the teaching round. ``hold_arm`` must raise
    ``ManualGuidanceRecoveryError`` if torque does not come back, exactly like leaving
    the ``manual_guidance`` context: the arm is still limp and the operator must catch it.
    """

    def hold_arm(self) -> None:
        """Re-engage torque at the current pose so the operator can let go."""
        pass

    def release_arm(self) -> None:
        """Drop torque again to resume hand-guiding."""
        pass


__all__ = [
    "CalibrationCameraFrame",
    "CalibrationCaptureSource",
    "CalibrationPoseSource",
    "CartesianCalibrationMotion",
    "GuidanceHold",
    "JointCalibrationMotion",
    "JointState",
    "ManualGuidance",
    "ManualGuidanceRecoveryError",
]
