# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Data validation shared by calibration workflows."""

from __future__ import annotations

import numpy as np


def validate_intrinsics(intrinsics: np.ndarray | None, *, source: str) -> np.ndarray:
    """Return a finite camera matrix with positive focal lengths."""
    if intrinsics is None:
        raise RuntimeError(f"camera intrinsics unavailable ({source}); cannot solve without a real K.")
    matrix = np.asarray(intrinsics, dtype=np.float64).copy()
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise RuntimeError(
            f"camera intrinsics ({source}) invalid: shape={matrix.shape}, finite={np.isfinite(matrix).all()}."
        )
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    if fx <= 0.0 or fy <= 0.0:
        raise RuntimeError(f"camera intrinsics ({source}) non-positive focal length: fx={fx}, fy={fy}.")
    return matrix


__all__ = ["validate_intrinsics"]
