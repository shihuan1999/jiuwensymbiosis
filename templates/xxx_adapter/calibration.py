# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Minimal calibration-integration wrapper example for a new arm.

Copy this file to ``jiuwensymbiosis/calibration/adapters/xxx.py`` (NOTE: the
calibration wrapper lives under ``jiuwensymbiosis.calibration.adapters``, NOT
under the core adapter package, so the core adapter never imports calibration).
It is discovered by name from the core adapter package ``jiuwensymbiosis.adapters.xxx``
and exposes ``CALIBRATION_ADAPTER_SPEC``.

The generic plumbing (connected-driver guard, camera-frame capture, joint-move
forwarding, ``JointState`` construction, camera-mount resolution) is shared in
``jiuwensymbiosis.calibration.adapters._common``. This wrapper only expresses
per-machine semantics: pose conversion, joint order, and (optional) manual
guidance.

Requirement checklist (see docs/hardware-porting-guide.md §8.7):
  1. config.py declares ``camera_mount`` (default = this body's historical value).
  2. ``get_flange_transform_mm -> 4x4 SE(3), mm``.
  3. joint metadata via ``build_joint_state(order/unit/periodic/limits)``.
  4. optional cartesian motion + ManualGuidance.
  5. a runtime artifact loader ``load_calibration_artifact(path, *, mount)``.
  6. passes tests/unit_tests/calibration/test_adapter_conformance.py and
     ``python scripts/validate_adapter.py --module jiuwensymbiosis.adapters.xxx``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from jiuwensymbiosis.calibration.adapters._common import (
    build_joint_state,
    camera_mount_from_env,
    capture_camera_frame,
    move_joint_vector,
    require_connected_driver,
)
from jiuwensymbiosis.calibration.domain.ports import CalibrationCameraFrame, JointState
from jiuwensymbiosis.calibration.integration.integration import CalibrationAdapterSpec

if TYPE_CHECKING:
    from jiuwensymbiosis.adapters.xxx.env import XxxEnv

# Replace with the actual joint order of your body (J1..JN).
_XXX_JOINT_ORDER = tuple(f"J{index}" for index in range(1, 7))


def _pose_to_tf_mm(pose: Any) -> np.ndarray:
    """Convert the vendor pose to a flange-in-base SE(3) matrix in mm."""
    raise NotImplementedError("Implement the vendor pose-to-SE(3) conversion for this adapter.")


@dataclass(frozen=True)
class XxxCalibrationDevice:
    """Calibration-owned adapter wrapper over a connected :class:`XxxEnv`."""

    env: XxxEnv
    camera_mount: str = "eye_to_hand"  # default = this body's historical mount

    def __post_init__(self) -> None:
        # Freeze the mount once; later config mutation cannot diverge it.
        object.__setattr__(
            self,
            "camera_mount",
            camera_mount_from_env(self.env, default=self.camera_mount),
        )

    def get_flange_transform_mm(self) -> np.ndarray:
        driver = require_connected_driver(self.env, "xxx")
        return _pose_to_tf_mm(driver.get_pose())

    def get_joint_state(self) -> JointState:
        driver = require_connected_driver(self.env, "xxx")
        return build_joint_state(
            driver.get_angles(),
            unit="deg",
            order=_XXX_JOINT_ORDER,
            limits=self.env.joint_limits,
        )

    def move_joint_vector(self, q: np.ndarray) -> None:
        move_joint_vector(self.env, q)

    # --- optional: implement only if the body supports this space / guidance ---
    # def move_to_flange_transform_mm(self, tf: np.ndarray) -> None:
    #     driver = require_connected_driver(self.env, "xxx")
    #     driver.move_to_pose_blocking(matrix_mm_to_pose(tf))

    # @contextmanager
    # def manual_guidance(self):
    #     driver = require_connected_driver(self.env, "xxx")
    #     driver.disable_torque()
    #     try:
    #         yield
    #     finally:
    #         driver.restore_torque()

    def capture_calibration_frame(self) -> CalibrationCameraFrame:
        return capture_camera_frame(self.env, "xxx", distortion=None)


# Replace with your adapter's session factory, artifact loader and device class.
CALIBRATION_ADAPTER_SPEC = CalibrationAdapterSpec(
    package="jiuwensymbiosis.adapters.xxx",
    session_factory=build_xxx_session,  # noqa: F821 - from your session.py
    load_calibration_artifact=load_calibration,  # noqa: F821 - your runtime loader
    make_calibration_device=XxxCalibrationDevice,
)


__all__ = ["CALIBRATION_ADAPTER_SPEC", "XxxCalibrationDevice"]
