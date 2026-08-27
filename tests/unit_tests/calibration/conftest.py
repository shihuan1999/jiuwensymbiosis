"""Calibration test bootstrap for the in-tree JiuwenSymbiosis package."""

from __future__ import annotations

import sys
from pathlib import Path

# Keep the repository root importable when this subtree is run directly.
_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
