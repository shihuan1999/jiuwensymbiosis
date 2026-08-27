# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""SO-101 calibration-artifact loader.

Encapsulates the generic ``perception.calibration.load_calibration`` with the
SO-101 eye-to-hand frame field (``T_base_cam``) and SO-101 legacy/env
conventions, so a calibration that passes the reload smoke is the one the
runtime would actually load.

Lives in ``_calibration.py`` (not ``__init__.py``) so the calibration-owned
adapter spec can import the loader directly, without a reverse import from the
package ``__init__`` (which would create a cycle).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

from jiuwensymbiosis.calibration_schema import EYE_TO_HAND_FRAME_FIELD


def load_calibration_artifact(
    path: str | Path,
    *,
    mount: Literal["eye_in_hand", "eye_to_hand"] = "eye_to_hand",
) -> dict[str, Any]:
    """Adapter-owned runtime calibration loader (for replay validation).

    Delegates to the SAME generic loader ``So101Driver._load_calibration`` uses
    at runtime, with the eye-to-hand frame field (``T_base_cam``) and the SO-101
    legacy/env conventions, so a calibration that passes this hook is the one
    the runtime would actually load.
    """
    from jiuwensymbiosis.perception.calibration import load_calibration

    if mount != "eye_to_hand":
        raise ValueError(f"SO-101 is eye-to-hand only; got mount={mount!r}.")
    return load_calibration(
        path,
        frame_field=EYE_TO_HAND_FRAME_FIELD,
        legacy_field="T_base_cam_legacy",
        env_var="JIUWEN_SO101_ALLOW_LEGACY_CALIB",
    )


__all__ = ["load_calibration_artifact"]
