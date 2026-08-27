# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Neutral hand-eye calibration JSON schema vocabulary.

This module deliberately lives outside :mod:`jiuwensymbiosis.calibration` so
both the runtime loader and the calibration publisher can share the wire
contract without making core runtime code depend on the calibration subsystem.
"""

from __future__ import annotations

from typing import Literal

CURRENT_SCHEMA_VERSION = 2
EYE_IN_HAND_FRAME_FIELD = "T_flange_cam"
EYE_TO_HAND_FRAME_FIELD = "T_base_cam"
PIPER_LEGACY_FRAME_FIELD = "T_TCP_cam"


def frame_field_for_mount(mount: Literal["eye_in_hand", "eye_to_hand"]) -> str:
    """Return the schema-2 camera-pose field for a validated mount."""
    return EYE_TO_HAND_FRAME_FIELD if mount == "eye_to_hand" else EYE_IN_HAND_FRAME_FIELD


__all__ = [
    "CURRENT_SCHEMA_VERSION",
    "EYE_IN_HAND_FRAME_FIELD",
    "EYE_TO_HAND_FRAME_FIELD",
    "PIPER_LEGACY_FRAME_FIELD",
    "frame_field_for_mount",
]
