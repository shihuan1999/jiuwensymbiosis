# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""The algorithm libraries stay below the api layer.

``perception/`` and ``motion/`` are body-agnostic algorithms the api layer composes —
``api/components.py`` imports both. An import back the other way closes that loop, and
the result shapes the two sides share live in ``jiuwensymbiosis/contracts.py`` (owned by
neither) precisely so it never has to happen.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

import jiuwensymbiosis

_ROOT = Path(jiuwensymbiosis.__file__).parent
_BELOW_THE_API_LAYER = ("perception", "motion")
_MODULES = sorted(p for d in _BELOW_THE_API_LAYER for p in (_ROOT / d).rglob("*.py"))


def _imported_modules(path: Path) -> set[str]:
    """Absolute module names imported anywhere in ``path`` — including inside functions."""
    names: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            names.add(node.module)
    return names


@pytest.mark.parametrize("path", _MODULES, ids=lambda p: f"{p.parent.name}/{p.name}")
def test_no_import_of_the_api_layer(path):
    offenders = sorted(m for m in _imported_modules(path) if m.startswith("jiuwensymbiosis.api"))
    assert not offenders, (
        f"{path.relative_to(_ROOT.parent)} imports {offenders}, but perception/ and motion/ sit BELOW "
        "the api layer, which imports them back through api/components.py. A shape both sides need "
        "belongs in jiuwensymbiosis/contracts.py."
    )
