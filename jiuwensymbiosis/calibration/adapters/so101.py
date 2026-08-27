# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""SO-101's calibration hardware-port view and integration specification."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from jiuwensymbiosis.adapters.so101._calibration import load_calibration_artifact
from jiuwensymbiosis.adapters.so101.geometry import pose_mm_deg_to_matrix_m
from jiuwensymbiosis.adapters.so101.session import build_so101_session
from jiuwensymbiosis.calibration.adapters._common import (
    build_joint_state,
    camera_mount_from_env,
    capture_camera_frame,
    move_joint_path,
    move_joint_vector,
    require_connected_driver,
)
from jiuwensymbiosis.calibration.domain.ports import CalibrationCameraFrame, JointState
from jiuwensymbiosis.calibration.integration.integration import CalibrationAdapterSpec

if TYPE_CHECKING:
    from collections.abc import Iterator

    from jiuwensymbiosis.adapters.so101.env import So101Env


@dataclass(frozen=True)
class So101CalibrationDevice:
    """Calibration-owned adapter wrapper over a connected :class:`So101Env`."""

    env: So101Env
    camera_mount: str = "eye_to_hand"

    def __post_init__(self) -> None:
        # Snapshot the mount once at construction so later mutation of the
        # original config cannot diverge Env/driver/calibration frame semantics
        # within this session.
        object.__setattr__(
            self,
            "camera_mount",
            camera_mount_from_env(self.env, default=self.camera_mount),
        )

    def get_flange_transform_mm(self) -> np.ndarray:
        driver = require_connected_driver(self.env, "SO-101")
        transform = np.asarray(pose_mm_deg_to_matrix_m(driver.get_pose()), dtype=np.float64).copy()
        transform[:3, 3] *= 1000.0
        return transform

    def get_joint_state(self) -> JointState:
        from jiuwensymbiosis.adapters.so101.lowlevel import ARM_JOINT_ORDER

        driver = require_connected_driver(self.env, "SO-101")
        order = tuple(ARM_JOINT_ORDER)
        return build_joint_state(
            list(driver.get_angles()),
            unit="deg",
            order=order,
            limits=self.env.joint_limits,
        )

    def move_joint_vector(self, q: np.ndarray) -> None:
        move_joint_vector(self.env, q)

    def move_joint_path(self, waypoints: np.ndarray) -> None:
        # SO-101's driver exposes ``move_joint_path_blocking``: stream the
        # dense path at trajectory_hz, settle only on the final waypoint.  This
        # avoids N_dense per-point settle cycles on slew-limit intermediates.
        move_joint_path(self.env, waypoints)

    # Teaching keeps gripper torque on: the eye-to-hand board is clamped in it, so
    # releasing the end effector would drop the board mid-session.
    @contextmanager
    def manual_guidance(self) -> Iterator[None]:
        """Disable arm torque for teaching and restore it without a stale-goal jump."""
        with require_connected_driver(self.env, "SO-101").hand_guiding():
            yield

    def release_arm(self) -> None:
        """Drop arm torque for hand-guiding (``GuidanceHold``)."""
        require_connected_driver(self.env, "SO-101").release_for_hand_guiding()

    def hold_arm(self) -> None:
        """Re-engage torque at the current pose so the operator can let go (``GuidanceHold``)."""
        require_connected_driver(self.env, "SO-101").restore_torque_at_current_pose()

    def capture_calibration_frame(self) -> CalibrationCameraFrame:
        return capture_camera_frame(self.env, "SO-101", distortion=None)


CALIBRATION_ADAPTER_SPEC = CalibrationAdapterSpec(
    package="jiuwensymbiosis.adapters.so101",
    session_factory=build_so101_session,
    load_calibration_artifact=load_calibration_artifact,
    make_calibration_device=So101CalibrationDevice,
)


__all__ = ["CALIBRATION_ADAPTER_SPEC", "So101CalibrationDevice"]
