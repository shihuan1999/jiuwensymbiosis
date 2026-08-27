#!/usr/bin/env python3
# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Eye-to-hand calibration CLI (compatibility alias).

This entry point is a thin facade over the unified ``hand_eye_calib.main()``
application flow. It shares the exact same Python application flow as the
unified entry point, but requires the device camera mount to be ``eye_to_hand``.

The orchestration lives in the calibration ``collect_waypoints``,
``execute_calibration`` and ``replay_calibration`` workflows. This module owns
nothing but the mount guard; it does not implement its own station collection,
solve, quality gating, or artifact writing.
"""

from __future__ import annotations

from scripts.calibrate.hand_eye_calib import main as _unified_main


def main(argv: list[str] | None = None) -> int:
    """Delegate to the unified flow, pinned to eye-to-hand."""
    prog = "eye_to_hand_calib.py"
    return _unified_main(argv, prog=prog, require_mount="eye_to_hand")


if __name__ == "__main__":
    raise SystemExit(main())
