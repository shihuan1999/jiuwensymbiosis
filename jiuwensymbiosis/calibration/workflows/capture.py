# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Station capture: CalibrationCaptureSource protocol + CaptureResult.

The execution workflow captures stations through the calibration-owned
``CalibrationCaptureSource`` protocol. Generic calibration code never reaches
into ``low_level``; a calibration-owned adapter wrapper turns Driver data into
the unified ``CalibrationCameraFrame`` type.

``CaptureResult`` is the fixed-shape snapshot solve/publish reads intrinsics +
distortion from. After the device disconnects, only this snapshot is consumed;
the workflow never reads the device or Driver again.

Dependency direction: ``CalibrationCameraFrame`` and ``CalibrationCaptureSource``
are owned by :mod:`jiuwensymbiosis.calibration.domain.ports`; adapter calibration
devices implement them structurally. Only ``CaptureResult`` is defined here.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from jiuwensymbiosis.calibration.domain.models import Station
from jiuwensymbiosis.calibration.domain.ports import (
    CalibrationCameraFrame,
    CalibrationCaptureSource,
)

__all__ = [
    "CalibrationCameraFrame",
    "CalibrationCaptureSource",
    "CaptureResult",
]


@dataclass
class CaptureResult:
    """Fixed-shape snapshot: stations + intrinsics + capture/joint metadata.

    ``dry_run=True`` means no stations were captured (trajectory validation
    only); solve/publish reads ``intrinsics`` from this object after the env
    has disconnected, never from the Env or Driver.
    """

    stations: list[Station]
    intrinsics: np.ndarray | None
    dry_run: bool = False
    capture_meta: list[dict] = field(default_factory=list)
    joint_meta: dict | None = None
    distortion: np.ndarray | None = None
