# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Shared camera-mount vocabulary for calibration boundaries.

The archive and workflow boundaries deliberately raise different exception
types, but they must agree on the accepted mount values.  This module owns the
small, dependency-free canonicalisation primitive so those boundary adapters
cannot drift or introduce an import cycle.
"""

from __future__ import annotations

from typing import Any, Literal, cast

Mount = Literal["eye_in_hand", "eye_to_hand"]
MOUNT_VALUES: tuple[Mount, ...] = ("eye_in_hand", "eye_to_hand")


def canonical_mount(value: Any) -> Mount:
    """Return a canonical camera-mount value or raise ``ValueError``.

    String coercion preserves the archive boundary's historical handling of
    scalar-like values while keeping the vocabulary check in one place.
    Callers add their own source/context and public exception type.
    """
    mount = str(value)
    if mount not in MOUNT_VALUES:
        raise ValueError(f"{value!r} is not a valid camera mount")
    return cast(Mount, mount)


__all__ = ["MOUNT_VALUES", "Mount", "canonical_mount"]
