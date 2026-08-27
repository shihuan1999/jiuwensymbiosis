# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Piper calibration JSON loader.

Thin wrapper over ``perception.calibration.load_calibration`` with
Piper-specific defaults: the camera pose lives in ``tf_flange_cam`` (6-DoF
eye-in-hand on the wrist).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from jiuwensymbiosis.calibration_schema import EYE_IN_HAND_FRAME_FIELD, PIPER_LEGACY_FRAME_FIELD
from jiuwensymbiosis.perception.calibration import (
    CURRENT_SCHEMA_VERSION,
    LegacyCalibrationError,
)
from jiuwensymbiosis.perception.calibration import (
    load_calibration as _generic_load_calibration,
)

__all__ = ["load_calibration", "LegacyCalibrationError", "CURRENT_SCHEMA_VERSION"]


def load_calibration(
    path: str | Path,
    *,
    mount: Literal["eye_in_hand"] = "eye_in_hand",
) -> dict[str, Any]:
    """Load a Piper calibration JSON. See ``perception.calibration`` for the schema.

    ``mount`` satisfies the calibration subsystem's loader contract, which passes
    the session's camera topology so the loader reads the matching frame field.
    Piper's camera is wrist-mounted, so only ``eye_in_hand`` (``T_flange_cam``)
    is accepted; a base-fixed Piper camera would need its own calibration and
    projection path rather than a silent reinterpretation of this transform.
    """
    if mount != "eye_in_hand":
        raise ValueError(f"Piper camera mount must be 'eye_in_hand' (the camera is wrist-mounted); got {mount!r}.")
    return _generic_load_calibration(
        path,
        frame_field=EYE_IN_HAND_FRAME_FIELD,
        legacy_field=PIPER_LEGACY_FRAME_FIELD,
        env_var="JIUWEN_PIPER_ALLOW_LEGACY_CALIB",
    )
