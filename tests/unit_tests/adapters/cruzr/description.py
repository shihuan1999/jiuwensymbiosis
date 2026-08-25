# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Where the Cruzr robot description lives, for the tests that need the real one.

The description is a ROS package that lives outside this repo, so there is no path that is
right on more than one machine and CI has none at all. Defining it once here means a moved
workspace is a one-line change rather than a sweep across eight files, and
``requires_description`` is the single gate those tests skip behind.

Point ``$CRUZR_WS`` at your workspace when it is not ``~/Robot/Cruzr_ws``.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

WORKSPACE = Path(os.environ.get("CRUZR_WS", "~/Robot/Cruzr_ws")).expanduser()
_PACKAGE = WORKSPACE / "cruzr_s2_description"

URDF = str(_PACKAGE / "cruzr_s2_description" / "urdf" / "cruzr_s2_v1" / "cruzr_s2_v1.urdf")
PACKAGE_DIR = str(_PACKAGE)
MESHES = str(_PACKAGE / "cruzr_s2_description" / "meshes")

requires_description = pytest.mark.skipif(
    not Path(URDF).exists(), reason="cruzr description not checked out (set $CRUZR_WS)"
)


def config(**overrides):
    """A ``CruzrConfig`` pointing at the checked-out description.

    The config itself defaults these to None — no path is right on every machine — so a test
    that needs the real robot description says so here, and pairs this with
    ``@requires_description``.
    """
    from jiuwensymbiosis.adapters.cruzr.config import CruzrConfig

    return CruzrConfig(urdf_path=URDF, urdf_package_dir=PACKAGE_DIR, **overrides)
