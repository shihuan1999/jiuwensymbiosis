# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Cross-vendor SE(3) and pinhole-projection primitives.

Pure math. Knows nothing about any specific robot's kinematics or any
specific camera model beyond the pinhole intrinsics. Per-vendor geometry
lives in the per-vendor ``geometry.py`` and composes these.

Frame conventions used by the helpers:
  cam  — RealSense color-stream optical frame (CV convention):
           x_cam = image right (u +)
           y_cam = image down  (v +)
           z_cam = optical axis (into the scene)
  Any other frame is fully described by a 4x4 homogeneous transform.
"""

from __future__ import annotations

import math
from typing import Any, NamedTuple

import numpy as np
from scipy.spatial.transform import Rotation  # scipy is a core dependency (see pyproject)


def make_transform(rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and a 3-vector translation."""
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = t
    return transform


def apply_transform(transform: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Apply a 4x4 SE(3) transform to a (3,) point or (N,3) array of points."""
    p = np.asarray(p)
    if p.ndim == 1:
        result: np.ndarray = transform[:3, :3] @ p + transform[:3, 3]
        return result
    result = p @ transform[:3, :3].T + transform[:3, 3]
    return result


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Closed-form inverse of an SE(3) transform: (R, t) → (Rᵀ, -Rᵀ t)."""
    rot = transform[:3, :3]
    t = transform[:3, 3]
    transform_inv = np.eye(4)
    transform_inv[:3, :3] = rot.T
    transform_inv[:3, 3] = -rot.T @ t
    return transform_inv


def _rot_z(deg: float) -> np.ndarray:
    """Rotation about base Z by ``deg`` degrees."""
    c = math.cos(math.radians(deg))
    s = math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def pixel_and_depth_to_camera_xyz(uv: tuple[float, float], depth_m: float, intrinsics: np.ndarray) -> np.ndarray:
    """Back-project a single pixel + metric depth to camera-frame XYZ (in mm).

    Note: ``depth_m`` is in meters; output is in mm to match some robot's base-frame
    convention (so callers can compose with mm-valued SE(3) transforms without
    a unit jump).
    """
    u, v = uv
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    ppx, ppy = intrinsics[0, 2], intrinsics[1, 2]
    z_mm = float(depth_m) * 1000.0
    x_mm = (u - ppx) * z_mm / fx
    y_mm = (v - ppy) * z_mm / fy
    return np.array([x_mm, y_mm, z_mm], dtype=np.float64)


def pixels_and_depths_to_camera_xyz(
    us: np.ndarray, vs: np.ndarray, depths_m: np.ndarray, intrinsics: np.ndarray
) -> np.ndarray:
    """Vectorized :func:`pixel_and_depth_to_camera_xyz` over ``(N,)`` pixels + depths.

    Returns an ``(N, 3)`` array of camera-frame points in mm (same pinhole model
    and mm-output convention as the single-pixel helper). Used for projecting a
    whole segmentation mask to a point cloud instead of one centroid pixel.
    """
    us = np.asarray(us, dtype=np.float64)
    vs = np.asarray(vs, dtype=np.float64)
    z_mm = np.asarray(depths_m, dtype=np.float64) * 1000.0
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    ppx, ppy = intrinsics[0, 2], intrinsics[1, 2]
    x_mm = (us - ppx) * z_mm / fx
    y_mm = (vs - ppy) * z_mm / fy
    return np.stack([x_mm, y_mm, z_mm], axis=-1)


# =============================================================================
# Pose / rotation helpers (migrated from scripts/calibrate/handeye_core.py).
# Body-agnostic: duck-typed on vendor pose objects (x/y/z/rx/ry/rz or r).
# These live here (not in scripts/) so env/base + adapters can import them
# after a wheel install (scripts/ is not in the packaged tree).
# =============================================================================

# Intrinsic XYZ Euler axis order, matching the Piper adapter's _RPY_AXES and the
# SO-101 adapter's _EULER_AXES so rx/ry/rz mean the same thing framework-wide.
_RPY_AXES = "xyz"


def rpy_deg_to_rot(rx_deg: float, ry_deg: float, rz_deg: float, axes: str = _RPY_AXES) -> np.ndarray:
    """RPY (degrees) -> 3x3 rotation matrix, matching the runtime FlangePose convention."""
    rot = Rotation.from_euler(axes, [rx_deg, ry_deg, rz_deg], degrees=True).as_matrix()
    return np.asarray(rot, dtype=np.float64)


class XyzRpy(NamedTuple):
    """A pose as millimetres + degrees. Unpacks like the plain tuple it replaces."""

    x_mm: float
    y_mm: float
    z_mm: float
    rx_deg: float
    ry_deg: float
    rz_deg: float


def matrix_mm_to_xyzrpy(
    matrix: np.ndarray,
    *,
    axes: str = _RPY_AXES,
) -> XyzRpy:
    """4x4 SE(3) with millimetre translation -> ``x,y,z,rx,ry,rz``.

    This is the body-agnostic half of converting a calibration/workflow matrix
    into an adapter pose value object. Adapters only need to pass the returned
    values to their own dataclass constructor.
    """
    arr = np.asarray(matrix, dtype=np.float64)
    if arr.shape != (4, 4):
        raise ValueError(f"matrix must be a 4x4 array, got shape {arr.shape}.")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"matrix has non-finite value: {matrix!r}.")

    rx_deg, ry_deg, rz_deg = Rotation.from_matrix(arr[:3, :3]).as_euler(axes, degrees=True)
    x_mm, y_mm, z_mm = arr[:3, 3]
    return XyzRpy(
        float(x_mm),
        float(y_mm),
        float(z_mm),
        float(rx_deg),
        float(ry_deg),
        float(rz_deg),
    )


def _first_attr(obj: Any, *names: str) -> Any:
    """Return the first existing attribute value; raise AttributeError if none exist."""
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    raise AttributeError(f"pose missing attributes {names}")


def _opt_attr(obj: Any, *names: str) -> float | None:
    for n in names:
        if hasattr(obj, n):
            val = getattr(obj, n)
            return None if val is None else float(val)
    return None


def pose_to_tf_base_flange(pose: Any, *, axes: str = _RPY_AXES) -> np.ndarray:
    """Vendor pose object -> 4x4 base<-flange SE(3) (mm/deg). Duck-typed, cross-body.

    Reads ``x/y/z`` (mm; ``*_mm`` variant preferred) and ``rx/ry/rz`` (deg). A
    4-DoF SCARA exposing only ``r`` is mapped to ``rz`` with ``rx=ry=0``. The
    default ``axes="xyz"`` reproduces piper ``FlangePose.to_tf_base_flange()``.
    """
    x = float(_first_attr(pose, "x_mm", "x"))
    y = float(_first_attr(pose, "y_mm", "y"))
    z = float(_first_attr(pose, "z_mm", "z"))
    rx = _opt_attr(pose, "rx_deg", "rx")
    ry = _opt_attr(pose, "ry_deg", "ry")
    rz = _opt_attr(pose, "rz_deg", "rz")
    if rx is None and ry is None:
        # 4-DoF SCARA: only a base-Z rotation r
        r = _opt_attr(pose, "r_deg", "r")
        if r is None:
            raise ValueError("pose missing rotation fields (need rx/ry/rz or r)")
        rx, ry, rz = 0.0, 0.0, float(r)
    else:
        rx = float(rx) if rx is not None else 0.0
        ry = float(ry) if ry is not None else 0.0
        rz = float(rz) if rz is not None else 0.0
    return make_transform(rpy_deg_to_rot(rx, ry, rz, axes=axes), np.array([x, y, z], dtype=np.float64))


def orthonormalize(rot: np.ndarray) -> np.ndarray:
    """Project a near-rotation matrix onto SO(3) via SVD (det=+1)."""
    u, _, vt = np.linalg.svd(np.asarray(rot, dtype=np.float64))
    rn = u @ vt
    if np.linalg.det(rn) < 0:
        u = u.copy()
        u[:, -1] *= -1.0
        rn = u @ vt
    return np.asarray(rn, dtype=np.float64)


def rotation_angle_deg(ra: np.ndarray, rb: np.ndarray) -> float:
    """Geodesic angle (degrees) between two rotation matrices."""
    m = ra.T @ rb
    c = np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))
