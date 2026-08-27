# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Shared plumbing for calibration-owned adapter wrappers.

These helpers handle only the generic glue every calibration device needs:
the connected-driver guard, camera-frame capture, joint-value motion
forwarding, ``JointState`` construction and camera-mount resolution.  They
deliberately do NOT cover pose conversion, joint order or manual guidance —
those are real per-machine semantics and stay in the concrete wrapper modules
(``piper.py`` / ``so101.py``).
"""

from __future__ import annotations

import time
from typing import Any, Literal

import numpy as np

from jiuwensymbiosis.calibration.domain.ports import CalibrationCameraFrame, JointState
from jiuwensymbiosis.calibration.workflows.preflight import validate_mount


def require_connected_driver(env: Any, body_name: str):
    """Return ``env.low_level`` or raise a clear connected-Env error.

    The error message names the body (``body_name``) so an operator connecting
    the wrong adapter sees which robot is expected.
    """
    driver = getattr(env, "low_level", None)
    if driver is None:
        raise RuntimeError(
            f"{body_name} calibration device requires a connected {body_name} env "
            "(session.env.low_level); connect the RobotSession first."
        )
    return driver


def capture_camera_frame(env: Any, body_name: str, *, distortion: np.ndarray | None = None) -> CalibrationCameraFrame:
    """Capture one calibration frame from the env's camera and wrap it.

    Assumes the driver exposes ``grab_frames() -> (rgb, depth)`` and an
    ``intrinsics`` 3x3 matrix.  Raises a body-specific ``RuntimeError`` when the
    camera or intrinsics are unavailable.
    """
    driver = require_connected_driver(env, body_name)
    frames = driver.grab_frames()
    if frames is None:
        raise RuntimeError(f"{body_name} calibration camera returned no frames.")
    rgb, _depth = frames
    intrinsics = getattr(driver, "intrinsics", None)
    if intrinsics is None:
        raise RuntimeError(f"{body_name} calibration camera intrinsics are unavailable.")
    return CalibrationCameraFrame(
        rgb=np.asarray(rgb),
        intrinsics=np.asarray(intrinsics, dtype=np.float64),
        distortion=distortion,
        captured_at_ns=time.time_ns(),
    )


def move_joint_vector(env: Any, q: np.ndarray) -> None:
    """Forward a joint vector to the driver's blocking joint move."""
    driver = require_connected_driver(env, "adapter")
    driver.move_joint_blocking([float(value) for value in np.asarray(q, dtype=np.float64)])


def move_joint_path(env: Any, waypoints: np.ndarray) -> None:
    """Stream a dense joint waypoint path, settling only at the end.

    When the driver exposes ``move_joint_path_blocking`` (e.g. SO-101), forward
    to it so intermediate points are streamed at ``trajectory_hz`` without
    per-point settling — settling each dense waypoint wastes ``N_dense`` settle
    cycles when only the capture stations need convergence.  Otherwise fall
    back to per-point ``move_joint_blocking`` so every adapter still works.
    """
    driver = require_connected_driver(env, "adapter")
    path = np.asarray(waypoints, dtype=np.float64)
    if path.ndim == 1:
        path = path.reshape(1, -1)
    streaming = getattr(driver, "move_joint_path_blocking", None)
    if callable(streaming):
        streaming(list(path))
        return
    for wp in path:
        driver.move_joint_blocking([float(v) for v in wp])


def build_joint_state(
    values: Any,
    *,
    unit: Literal["deg", "rad"],
    order: tuple[str, ...],
    periodic: tuple[bool, ...] | None = None,
    limits: dict[str, tuple[float, float]] | None = None,
) -> JointState:
    """Build a validated :class:`JointState` from a raw angle sequence.

    ``periodic`` defaults to all-false for ``order`` when omitted.
    """
    if periodic is None:
        periodic = tuple(False for _ in order)
    return JointState(
        values=np.asarray(values, dtype=np.float64),
        unit=unit,
        order=order,
        periodic=periodic,
        limits=limits,
    )


def camera_mount_from_env(env: Any, *, default: str) -> str:
    """Resolve and validate the session's calibration camera mount.

    An Env-level snapshot wins over its mutable config. The config and the
    concrete wrapper's historical default remain compatibility fallbacks for
    adapters that have not yet exposed a session-level property.
    """
    mount = getattr(env, "camera_mount", None)
    if mount is None:
        cfg = getattr(env, "cfg", None)
        mount = getattr(cfg, "camera_mount", default)
    return validate_mount(str(mount), source=f"{type(env).__name__}.camera_mount")


__all__ = [
    "build_joint_state",
    "camera_mount_from_env",
    "capture_camera_frame",
    "move_joint_path",
    "move_joint_vector",
    "require_connected_driver",
]
