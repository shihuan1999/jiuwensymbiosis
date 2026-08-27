# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Reusable calibration-adapter conformance assertions.

Adapter-specific test modules provide their own connected-driver double and
call :func:`assert_calibration_conformance`.  Keeping the assertions here,
without a central list of adapter names, means adding a body only requires a
test next to that body's adapter.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import numpy as np

from jiuwensymbiosis.calibration.domain.ports import (
    CalibrationCameraFrame,
    CalibrationCaptureSource,
    CalibrationPoseSource,
    CartesianCalibrationMotion,
    JointCalibrationMotion,
    JointState,
    ManualGuidance,
)
from jiuwensymbiosis.calibration.integration.integration import (
    CalibrationAdapterSpec,
    SolvedCalibration,
    validate_adapter_reload,
)


@dataclass(frozen=True)
class AdapterConformanceCase:
    """Adapter-owned inputs needed by the shared calibration contract test."""

    package: str
    build_device: Callable[[], tuple[Any, Any, CalibrationAdapterSpec]]
    expected_mount: Literal["eye_in_hand", "eye_to_hand"]
    joint_order: tuple[str, ...]
    supports_cartesian: bool = False
    supports_manual_guidance: bool = False
    joint_unit: Literal["deg", "rad"] = "deg"
    periodic: tuple[bool, ...] | None = None


def assert_calibration_conformance(case: AdapterConformanceCase, tmp_path: Path) -> None:
    """Assert Env/wrapper separation, ports, geometry, motion and artifacts."""
    device, env, spec = case.build_device()
    assert isinstance(spec, CalibrationAdapterSpec)
    assert spec.package == case.package
    assert callable(spec.session_factory)
    assert callable(spec.session_factory.from_yaml)
    assert callable(spec.session_factory.from_dict)
    assert callable(spec.load_calibration_artifact)
    assert callable(spec.make_calibration_device)

    # Calibration owns these ports. The core Env remains the single generic
    # hardware contract; only the calibration-owned wrapper may satisfy them.
    for port in (
        CalibrationCaptureSource,
        CalibrationPoseSource,
        JointCalibrationMotion,
        CartesianCalibrationMotion,
        ManualGuidance,
    ):
        assert not isinstance(env, port), f"{type(env).__name__} unexpectedly satisfies {port.__name__}"

    assert isinstance(device, CalibrationCaptureSource)
    assert isinstance(device, CalibrationPoseSource)
    assert isinstance(device, JointCalibrationMotion)
    assert device.camera_mount == case.expected_mount
    if case.supports_cartesian:
        assert isinstance(device, CartesianCalibrationMotion)
        device.move_to_flange_transform_mm(np.eye(4))
    if case.supports_manual_guidance:
        assert isinstance(device, ManualGuidance)

    state = device.get_joint_state()
    assert isinstance(state, JointState)
    assert state.unit == case.joint_unit
    assert state.order == case.joint_order
    assert len(state.periodic) == len(case.joint_order)
    expected_periodic = case.periodic if case.periodic is not None else tuple(False for _ in case.joint_order)
    assert state.periodic == expected_periodic
    assert state.values.shape == (len(case.joint_order),)
    assert np.all(np.isfinite(state.values))
    device.move_joint_vector(np.zeros(len(case.joint_order), dtype=np.float64))

    transform = np.asarray(device.get_flange_transform_mm(), dtype=np.float64)
    assert transform.shape == (4, 4)
    assert np.all(np.isfinite(transform))
    np.testing.assert_allclose(transform[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-6)
    rotation = transform[:3, :3]
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)

    frame = device.capture_calibration_frame()
    assert isinstance(frame, CalibrationCameraFrame)
    assert frame.rgb.ndim == 3
    assert frame.intrinsics.shape == (3, 3)
    assert np.all(np.isfinite(frame.intrinsics))
    if frame.distortion is not None:
        assert np.all(np.isfinite(frame.distortion))
    assert isinstance(frame.captured_at_ns, int)

    reload_path = tmp_path / f"{case.package.rsplit('.', 1)[-1]}-reload.json"
    transform_for_reload = np.eye(4)
    transform_for_reload[:3, 3] = [15.0, -20.0, 100.0]
    validate_adapter_reload(
        spec,
        reload_path,
        SolvedCalibration(transform_for_reload, np.eye(3) * 800.0, case.expected_mount),
        [],
    )


__all__ = ["AdapterConformanceCase", "assert_calibration_conformance"]
