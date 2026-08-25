# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Cruzr-only debug view: what did the waist detector actually see, and where does it land in base?

``CruzrApi.detect`` is deliberately **not** an action — it answers "what is in the frame", which is
a question a human asks while bringing a body up, not one a planner can act on. The plannable
answer is ``locate_for_grasp`` / ``locate_for_place``, which return a graspable pose rather than a
2-D box. So ``detect`` carries no ``ActionSpec``, is never emitted as a tool, and this script is
its only entry point.

It prints, for one noun, the detector's 2-D box + score, the depth read at the centroid, and the
base-frame XYZ that depth projects to — the three numbers that separate "the detector missed it"
from "the detector saw it but the depth is a hole" from "both are fine but the extrinsics are
wrong". A ``position`` of ``None`` names which of the two calibrations was missing.

Run at the robot's current pose (needs ROS + waist RGBD topics live; the detector sidecar is
started by the session when ``detector.spawn`` is set in the YAML)::

    python -m scripts.cruzr.debug_detect --config configs/cruzr/cruzr.yaml --object "white box"
    python -m scripts.cruzr.debug_detect --config configs/cruzr/cruzr.yaml --object box --json

For the RGB↔mask↔cloud *alignment* picture rather than these scalars, use
``scripts/cruzr/debug_align.py``.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from typing import Any

from jiuwensymbiosis.utils.proxy import clear_proxy_env  # noqa: E402 - call it before the package imports below

clear_proxy_env()

from jiuwensymbiosis.adapters.cruzr import build_cruzr_session  # noqa: E402 - after clear_proxy_env() (proxy hygiene)

logger = logging.getLogger(__name__)


def _report(result: dict[str, Any]) -> None:
    """Print the detection as the three numbers a bring-up actually reads."""
    if not result.get("ok"):
        logger.error("detect failed: reason=%s camera=%s",
                     result.get("reason"), result.get("camera_name"))
        return
    box = result.get("box_2d") or []
    u, v = result.get("pixel_uv") or (None, None)
    logger.info("object   : %s (camera=%s)", result.get("object"), result.get("camera_name"))
    logger.info("box_2d   : %s  score=%.3f", [round(float(b), 1) for b in box], result.get("score", 0.0))
    logger.info("centroid : u=%s v=%s  depth=%.3f m", u, v, result.get("depth_m", float("nan")))
    position = result.get("position")
    if position is None:
        logger.error("position : None — %s; the box and depth above are still good, so this is a "
                     "calibration gap, not a detection failure", result.get("position_reason"))
    else:
        logger.info("position : x=%.1f y=%.1f z=%.1f mm (base frame)", *position)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Cruzr waist-camera detection debug view")
    p.add_argument("--config", default="configs/cruzr/cruzr.yaml", help="Cruzr YAML config")
    p.add_argument("--object", default="box", help="target noun (English) to detect")
    p.add_argument("--camera-name", default="waist_rgbd", help="camera name echoed in the result")
    p.add_argument("--json", action="store_true", help="print the raw result dict instead")
    a = p.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    # The session owns connect/disconnect and the detector sidecar; __exit__ stops both even
    # when the detect below raises, so a failed bring-up never leaves the sidecar running.
    with build_cruzr_session.from_yaml(a.config) as session:
        result = session.api.detect(object_name=a.object, camera_name=a.camera_name)

    if a.json:
        # The machine-readable payload IS this program's output, not a log line, so it is
        # serialised straight to stdout for a caller to pipe; _report() below — the
        # human-readable view — is what goes through logging. No trailing newline: json.dump
        # writes the document and nothing else, which is what a consumer parses.
        json.dump(result, sys.stdout, indent=2, ensure_ascii=False, default=str)
    else:
        _report(result)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
