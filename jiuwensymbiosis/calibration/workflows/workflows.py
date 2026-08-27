# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Shared types for the in-tree calibration workflows."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from jiuwensymbiosis.calibration.domain.ports import CalibrationCameraFrame
from jiuwensymbiosis.calibration.domain.quality import ObservabilityThresholds, TargetConsistencyThresholds


class ArtifactReloadValidator(Protocol):
    """Injected integration callback that validates a temporary formal artifact."""

    def __call__(
        self,
        temporary_json: Path,
        tf_base_cam: np.ndarray,
        intrinsics: np.ndarray,
        mount: str,
        stations: list,
        *,
        t_flange_target: np.ndarray | None = None,
    ) -> None:
        pass


@dataclass(frozen=True)
class CalibrationRunOptions:
    """Explicit workflow inputs constructed at the CLI or integration boundary."""

    config: str | Path | None = None
    out: str | Path | None = None
    dry_run: bool = False
    n_stations: int = 20
    max_joint_step_deg: float = 5.0
    max_cartesian_step_mm: float = 10.0
    max_cartesian_step_deg: float = 5.0
    method: str = "PARK"
    cross_check: bool = False

    def __post_init__(self) -> None:
        if self.n_stations <= 0:
            raise ValueError("n_stations must be positive")
        for field_name in ("max_joint_step_deg", "max_cartesian_step_mm", "max_cartesian_step_deg"):
            value = float(getattr(self, field_name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field_name} must be a positive finite value, got {value!r}")
        if not self.method:
            raise ValueError("method must be non-empty")


@dataclass
class RunOutcome:
    """Calibration result, acceptance decision and emitted artifact."""

    result: Any
    decision: Any
    artifact_path: Path | None
    candidate: bool


@dataclass(frozen=True)
class WorkflowDependencies:
    """Non-device dependencies shared by execute and replay workflows."""

    options: CalibrationRunOptions
    adapter_package: str | None = None
    reload_validator: ArtifactReloadValidator | None = None
    observability_thresholds: ObservabilityThresholds | None = None
    target_consistency_thresholds: TargetConsistencyThresholds | None = None
    detect_fn: Callable[[CalibrationCameraFrame], Any] | None = None


__all__ = [
    "ArtifactReloadValidator",
    "CalibrationRunOptions",
    "RunOutcome",
]
