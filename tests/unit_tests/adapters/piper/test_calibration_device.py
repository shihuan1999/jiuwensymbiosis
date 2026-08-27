# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Contract tests for the Piper calibration device bridge."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from jiuwensymbiosis.adapters.piper.config import PiperConfig
from jiuwensymbiosis.adapters.piper.env import PiperEnv
from jiuwensymbiosis.calibration.domain.ports import (
    CalibrationCaptureSource,
    CartesianCalibrationMotion,
    JointCalibrationMotion,
)
from jiuwensymbiosis.calibration.integration.integration import load_adapter_spec
from tests.unit_tests.calibration.test_adapter_conformance import (
    AdapterConformanceCase,
    assert_calibration_conformance,
)


class _Driver:
    intrinsics = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])

    def __init__(self) -> None:
        self.joint_command = None
        self.pose_command = None

    def get_pose(self):
        return SimpleNamespace(x=100.0, y=20.0, z=300.0, rx=10.0, ry=20.0, rz=30.0)

    def get_angles(self):
        return SimpleNamespace(as_tuple=lambda: (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

    def move_joint_blocking(self, values):
        self.joint_command = values

    def move_to_pose_blocking(self, pose):
        self.pose_command = pose

    def grab_frames(self):
        return np.zeros((8, 8, 3), dtype=np.uint8), None


def test_piper_spec_builds_calibration_owned_ports():
    env = PiperEnv(PiperConfig(camera_mount="eye_to_hand"))
    driver = _Driver()
    env._inner = driver
    device = load_adapter_spec("jiuwensymbiosis.adapters.piper").make_calibration_device(env)

    assert isinstance(device, JointCalibrationMotion)
    assert isinstance(device, CartesianCalibrationMotion)
    assert isinstance(device, CalibrationCaptureSource)
    assert device.camera_mount == "eye_to_hand"
    state = device.get_joint_state()
    assert state.unit == "deg"
    assert state.order == ("J1", "J2", "J3", "J4", "J5", "J6")
    np.testing.assert_allclose(device.get_flange_transform_mm()[:3, 3], [100.0, 20.0, 300.0])

    device.move_joint_vector(np.arange(6, dtype=np.float64))
    assert driver.joint_command == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert device.capture_calibration_frame().intrinsics.shape == (3, 3)


def test_piper_calibration_conformance(tmp_path):
    def build_device():
        env = PiperEnv(PiperConfig())
        env._inner = _Driver()
        spec = load_adapter_spec("jiuwensymbiosis.adapters.piper")
        return spec.make_calibration_device(env), env, spec

    assert_calibration_conformance(
        AdapterConformanceCase(
            package="jiuwensymbiosis.adapters.piper",
            build_device=build_device,
            expected_mount="eye_in_hand",
            joint_order=("J1", "J2", "J3", "J4", "J5", "J6"),
            supports_cartesian=True,
        ),
        tmp_path,
    )
