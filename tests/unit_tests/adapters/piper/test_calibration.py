# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Tests for Piper's mount-aware calibration artifact loader."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from jiuwensymbiosis.adapters.piper._calibration import load_calibration


def _write_calibration(path: Path, *, frame_field: str, version: int = 2, legacy: bool = False) -> None:
    data = {
        "schema_version": version,
        "intrinsics": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
        frame_field: {"matrix_4x4": np.eye(4).tolist()},
    }
    if legacy:
        data["object"] = {"xyz_base_mm": [100, 200, 300]}
    path.write_text(json.dumps(data), encoding="utf-8")


@pytest.mark.parametrize("mount", [None, [], "", "ceiling", "eye-in-hand"])
def test_invalid_mount_fails_closed_before_file_access(tmp_path, mount):
    with pytest.raises(ValueError, match="camera mount"):
        load_calibration(tmp_path / "does-not-exist.json", mount=mount)  # type: ignore[arg-type]


def test_eye_to_hand_is_rejected_rather_than_reinterpreting_the_wrist_transform(tmp_path):
    """Piper's camera is wrist-mounted; a base-fixed Piper camera is not supported.

    The failure must be a rejection, not a silent read of ``T_base_cam``: loading
    a flange-frame transform as a constant base-camera pose corrupts every
    projection while looking completely healthy.
    """
    path = tmp_path / "eye_to_hand.json"
    _write_calibration(path, frame_field="T_base_cam")

    with pytest.raises(ValueError, match="camera mount"):
        load_calibration(path, mount="eye_to_hand")  # type: ignore[arg-type]


def test_eye_in_hand_retains_opt_in_legacy_compatibility(tmp_path, monkeypatch):
    path = tmp_path / "legacy.json"
    _write_calibration(path, frame_field="T_TCP_cam", version=1, legacy=True)
    monkeypatch.setenv("JIUWEN_PIPER_ALLOW_LEGACY_CALIB", "1")

    result = load_calibration(path, mount="eye_in_hand")

    assert result["T_flange_cam"]["matrix_4x4"].shape == (4, 4)
    assert result["T_flange_cam"]["_legacy_remap_from"] == "T_TCP_cam"
