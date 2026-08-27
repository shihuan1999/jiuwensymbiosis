# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Body-agnostic calibration subsystem and its small stable public surface.

The package owns its domain model, workflows and hardware ports. Framework
integration (session/config/adapter discovery) is intentionally outside the
domain boundary. Public symbols are loaded lazily so importing a hardware
port does not import the solver or optional OpenCV dependency.

Quality, trajectory, profile and preflight helpers belong to their defining
modules. Import those helpers explicitly rather than growing this package
facade; ``__all__`` is the frozen use-case/domain contract.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

from jiuwensymbiosis.calibration.domain.ports import (
    CalibrationCameraFrame,
    CalibrationCaptureSource,
    CalibrationPoseSource,
    CartesianCalibrationMotion,
    JointCalibrationMotion,
    JointState,
    ManualGuidance,
    ManualGuidanceRecoveryError,
)

_LAZY_EXPORTS: dict[str, str] = {}


def _register(module: str, *names: str) -> None:
    for name in names:
        _LAZY_EXPORTS[name] = module


_register(
    "jiuwensymbiosis.calibration.artifacts.archives",
    "dump_station_archive",
    "dump_waypoint_archive",
    "load_station_archive",
    "load_waypoint_archive",
)
_register(
    "jiuwensymbiosis.calibration.artifacts.artifacts",
    "save_candidate_report",
    "save_eye_to_hand_calibration",
)
_register(
    "jiuwensymbiosis.calibration.domain.models",
    "CalibrationDecision",
    "CalibrationQualityReport",
    "EyeInHandResult",
    "EyeToHandResult",
    "Station",
    "ViewDetection",
)
_register("jiuwensymbiosis.calibration.workflows.collect", "collect_waypoints")
_register("jiuwensymbiosis.calibration.workflows.execute", "execute_calibration")
_register("jiuwensymbiosis.calibration.workflows.replay", "replay_calibration")
_register(
    "jiuwensymbiosis.calibration.workflows.workflows",
    "CalibrationRunOptions",
    "RunOutcome",
)
_register(
    "jiuwensymbiosis.calibration.domain.solver",
    "save_calibration",
    "solve_eye_in_hand",
    "solve_eye_to_hand",
)


def __getattr__(name: str) -> Any:
    module_name = _LAZY_EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(module_name), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(_LAZY_EXPORTS) | set(__all__))


__all__ = [
    "CalibrationCameraFrame",
    "CalibrationCaptureSource",
    "CalibrationDecision",
    "CalibrationPoseSource",
    "CalibrationQualityReport",
    "CalibrationRunOptions",
    "CartesianCalibrationMotion",
    "EyeInHandResult",
    "EyeToHandResult",
    "JointCalibrationMotion",
    "JointState",
    "ManualGuidance",
    "ManualGuidanceRecoveryError",
    "RunOutcome",
    "Station",
    "ViewDetection",
    "collect_waypoints",
    "dump_station_archive",
    "dump_waypoint_archive",
    "execute_calibration",
    "load_station_archive",
    "load_waypoint_archive",
    "replay_calibration",
    "save_calibration",
    "save_candidate_report",
    "save_eye_to_hand_calibration",
    "solve_eye_in_hand",
    "solve_eye_to_hand",
]

del _register
