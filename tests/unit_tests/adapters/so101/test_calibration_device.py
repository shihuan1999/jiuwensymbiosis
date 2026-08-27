# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Contract tests for the SO-101 calibration device bridge."""

from __future__ import annotations

from contextlib import contextmanager

import numpy as np
import pytest

from jiuwensymbiosis.adapters.so101.config import So101Config
from jiuwensymbiosis.adapters.so101.env import So101Env
from jiuwensymbiosis.adapters.so101.geometry import So101Pose
from jiuwensymbiosis.adapters.so101.lowlevel import ARM_JOINT_ORDER
from jiuwensymbiosis.calibration.domain.ports import (
    CalibrationCaptureSource,
    JointCalibrationMotion,
    ManualGuidance,
    ManualGuidanceRecoveryError,
)
from jiuwensymbiosis.calibration.integration.integration import load_adapter_spec
from tests.unit_tests.calibration.test_adapter_conformance import (
    AdapterConformanceCase,
    assert_calibration_conformance,
)


class _Driver:
    """Fake of the driver's ``HandGuidingDriver`` surface — the orchestration it
    stands in for is covered by ``test_lowlevel.TestHandGuiding``."""

    intrinsics = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])

    def __init__(self) -> None:
        self.events: list[str] = []
        self.fail_restore = False

    def get_pose(self):
        return So101Pose(10.0, 20.0, 30.0, 0.0, 0.0, 0.0)

    def get_angles(self):
        return [1.0, 2.0, 3.0, 4.0, 5.0]

    def move_joint_blocking(self, values):
        self.events.append(f"move:{list(values)}")

    def grab_frames(self):
        return np.zeros((8, 8, 3), dtype=np.uint8), None

    @contextmanager
    def hand_guiding(self, *, include_end_effector: bool = False):
        self.release_for_hand_guiding(include_end_effector=include_end_effector)
        try:
            yield
        finally:
            self.restore_torque_at_current_pose(include_end_effector=include_end_effector)

    def release_for_hand_guiding(self, *, include_end_effector: bool = False):
        self.events.append("release_all" if include_end_effector else "release")

    def restore_torque_at_current_pose(self, *, include_end_effector: bool = False, cause=None):
        self.events.append("restore")
        if self.fail_restore:
            raise ManualGuidanceRecoveryError("restore failed")


def _device():
    cfg = So101Config(
        port="/dev/fake",
        home_joints_deg=[0.0] * 5,
        joint_limits=dict.fromkeys(ARM_JOINT_ORDER, (-180.0, 180.0)),
        safety_validated=True,
        camera_serial="fake-camera",
    )
    env = So101Env(cfg)
    driver = _Driver()
    env._inner = driver
    return load_adapter_spec("jiuwensymbiosis.adapters.so101").make_calibration_device(env), driver, env


def test_so101_spec_builds_calibration_owned_ports():
    device, _driver, _env = _device()
    assert isinstance(device, JointCalibrationMotion)
    assert isinstance(device, CalibrationCaptureSource)
    assert isinstance(device, ManualGuidance)
    assert device.camera_mount == "eye_to_hand"
    state = device.get_joint_state()
    assert state.unit == "deg"
    assert state.order == tuple(ARM_JOINT_ORDER)
    np.testing.assert_allclose(device.get_flange_transform_mm()[:3, 3], [10.0, 20.0, 30.0])


def test_so101_calibration_conformance(tmp_path):
    def build_device():
        device, _driver, env = _device()
        spec = load_adapter_spec("jiuwensymbiosis.adapters.so101")
        return device, env, spec

    assert_calibration_conformance(
        AdapterConformanceCase(
            package="jiuwensymbiosis.adapters.so101",
            build_device=build_device,
            expected_mount="eye_to_hand",
            joint_order=tuple(ARM_JOINT_ORDER),
            supports_manual_guidance=True,
        ),
        tmp_path,
    )


def test_manual_guidance_delegates_to_the_driver_keeping_the_gripper_powered():
    """Teaching must not release the end effector — the eye-to-hand board is clamped in it."""
    device, driver, _env = _device()
    with device.manual_guidance():
        driver.events.append("body")
    assert driver.events == ["release", "body", "restore"]


def test_manual_guidance_propagates_a_failed_recovery():
    device, driver, _env = _device()
    driver.fail_restore = True
    with pytest.raises(ManualGuidanceRecoveryError, match="restore failed"):
        with device.manual_guidance():
            pass


def test_guidance_hold_pauses_without_ending_the_context():
    device, driver, _env = _device()
    with device.manual_guidance():
        device.hold_arm()
        device.release_arm()
    assert driver.events == ["release", "restore", "release", "restore"]
