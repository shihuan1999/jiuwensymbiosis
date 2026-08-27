# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Piper's calibration hardware-port view and integration specification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from jiuwensymbiosis.adapters.piper._calibration import load_calibration
from jiuwensymbiosis.adapters.piper.geometry import FlangePose
from jiuwensymbiosis.adapters.piper.session import build_piper_session
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
    from jiuwensymbiosis.adapters.piper.env import PiperEnv

_PIPER_JOINT_ORDER = tuple(f"J{index}" for index in range(1, 7))


@dataclass(frozen=True)
class PiperCalibrationDevice:
    """Calibration-owned adapter wrapper over a connected :class:`PiperEnv`."""

    env: PiperEnv
    camera_mount: str = "eye_in_hand"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "camera_mount",
            camera_mount_from_env(self.env, default=self.camera_mount),
        )

    def get_flange_transform_mm(self) -> np.ndarray:
        driver = require_connected_driver(self.env, "Piper")
        pose = driver.get_pose()
        return FlangePose(pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz).to_tf_base_flange()

    def get_joint_state(self) -> JointState:
        driver = require_connected_driver(self.env, "Piper")
        angles = driver.get_angles()
        return build_joint_state(
            angles.as_tuple(),
            unit="deg",
            order=_PIPER_JOINT_ORDER,
            limits=self.env.joint_limits,
        )

    def move_joint_vector(self, q: np.ndarray) -> None:
        move_joint_vector(self.env, q)

    def move_to_flange_transform_mm(self, tf: np.ndarray) -> None:
        driver = require_connected_driver(self.env, "Piper")
        from jiuwensymbiosis.adapters.piper.geometry import matrix_mm_to_flange_pose

        driver.move_to_pose_blocking(matrix_mm_to_flange_pose(np.asarray(tf, dtype=np.float64)))

    def capture_calibration_frame(self) -> CalibrationCameraFrame:
        return capture_camera_frame(self.env, "Piper", distortion=None)


CALIBRATION_ADAPTER_SPEC = CalibrationAdapterSpec(
    package="jiuwensymbiosis.adapters.piper",
    session_factory=build_piper_session,
    load_calibration_artifact=load_calibration,
    make_calibration_device=PiperCalibrationDevice,
)


__all__ = ["CALIBRATION_ADAPTER_SPEC", "PiperCalibrationDevice"]
