# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Calibration profile + trajectory space resolution.

``_resolve_space`` is NOT a hardware preflight — it reads the YAML
``calibration.trajectory.space`` value only. ``trajectory.space`` is
authoritative; a missing value is a configuration error (no inference from
``joint_limits`` presence, which conflated "has limits" with "wants joint
space"). Lives in the profile module, not ``preflight.py``, so the hardware
preflight module stays free of YAML concerns.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, cast

from jiuwensymbiosis.calibration.domain.trajectory import TrajectorySpace

try:
    import yaml
except ImportError:  # pragma: no cover - yaml is a core dep, but be safe
    yaml = None


def load_profile(yaml_path: str | Path | None) -> dict[str, Any]:
    """Read the top-level ``calibration:`` block from a YAML (or return {}).

    ``camera_mount`` is intentionally NOT read here — it is owned by the
    calibration-owned adapter wrapper (typically resolved from ``cfg.camera_mount``)
    and exposed to the workflow as ``device.camera_mount``.
    """
    if yaml_path is None:
        return {}
    if yaml is None:
        raise RuntimeError("PyYAML not installed; cannot read calibration profile YAML.")
    data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}
    return dict(data.get("calibration") or {})


def resolve_space(profile: dict[str, Any] | None) -> TrajectorySpace:
    """trajectory.space is authoritative: ``calibration.trajectory.space`` only.

    No guessing from joint_limits presence. A missing trajectory.space is a
    configuration error.
    """
    traj = (profile or {}).get("trajectory") or {}
    space = str(traj.get("space") or "").strip().lower()
    if space not in ("joint", "cartesian"):
        raise ValueError(
            "calibration.trajectory.space must be 'joint' or 'cartesian'; set it explicitly in the YAML (no inference)."
        )
    return cast(TrajectorySpace, space)


def resolve_out_path(args_out: str | Path | None, config: str | Path | None) -> Path:
    """formal output: --out > calibration.output > tmp/eye_to_hand_calib.json."""
    if args_out:
        return Path(args_out)
    profile = load_profile(config) if config else {}
    cfg_out = (profile.get("output") or "").strip()
    if cfg_out:
        return Path(cfg_out)
    return Path("tmp/eye_to_hand_calib.json")


__all__ = [
    "load_profile",
    "resolve_out_path",
    "resolve_space",
]
