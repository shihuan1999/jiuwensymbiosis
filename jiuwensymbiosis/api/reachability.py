# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""``Reachability`` — can this arm reach that point, from where the body is standing now.

Kept out of :mod:`jiuwensymbiosis.api.components` because it is not one: a component is an
implementation an adapter holds and forwards an ACTION to, and this answers no action at
all. The planner reads ``check_reachable`` / ``describe_reach`` directly (see
``agent/run.py``); the LLM never sees them, and there is no ``ActionSpec`` behind them.

What it is instead is a planning-time judge, gated by ``planning.reachability`` — which is
itself derived from a body holding this (or its own) judge AND its Env shipping the URDF the
judge reads.
"""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

__all__ = ["Reachability"]


class Reachability:
    """URDF-based reachability prior, HELD by a body that wants the generic judge.

    Reads ``env.urdf_path`` + ``env.arm_chains`` and runs the single-arm reach judge
    (``kinematics.reach``), degrading to ``None`` when the body exposes no URDF. NOT an
    action — the planner reads ``check_reachable`` directly, the LLM never calls it.

    A body whose geometry needs a different judge (cruzr weighs both arms plus an adaptive
    lifter) simply writes its own ``check_reachable`` and holds nothing here. Either way
    ``planning.reachability`` is DERIVED, never declared — holding a judge is the Api half,
    shipping the URDF it reads is the Env half, and only the intersection is true.
    """

    def __init__(self, api: Any) -> None:
        self.api = api

    @property
    def env(self) -> Any:
        return self.api.env

    def check_reachable(self, target: Any) -> bool | None:
        """Whether the end effector can reach ``target`` (a scene object dict with ``center_mm``, or an
        xyz-mm sequence) from the current body pose. ``None`` when the body has no URDF (caller skips).
        Any arm chain reaching the point counts as reachable.
        """
        urdf = getattr(self.env, "urdf_path", None)
        chains = getattr(self.env, "arm_chains", None)
        if not urdf or not chains:
            return None
        xyz = target.get("center_mm") if isinstance(target, dict) else target
        if not (isinstance(xyz, (list, tuple)) and len(xyz) == 3):
            return None
        from jiuwensymbiosis.kinematics.reach import reachable_point
        from jiuwensymbiosis.kinematics.urdf_chain import parse_chain

        q = self._reach_joint_positions()
        for root, leaf in chains.values():
            try:
                if reachable_point(parse_chain(urdf, root, leaf), xyz, q):
                    return True
            except Exception as exc:  # a chain we cannot solve is not a chain that reaches
                logger.warning("check_reachable: chain %s→%s failed: %s", root, leaf, exc)
        return False

    def describe_reach(self) -> dict | None:
        """Coarse reachable-workspace envelope (forward/lateral/height ranges, m) of one arm at the
        current pose — a no-target planning prior. ``None`` when the body has no URDF.
        """
        urdf = getattr(self.env, "urdf_path", None)
        chains = getattr(self.env, "arm_chains", None)
        if not urdf or not chains:
            return None
        from jiuwensymbiosis.kinematics.reach import reach_envelope
        from jiuwensymbiosis.kinematics.urdf_chain import parse_chain

        root, leaf = next(iter(chains.values()))  # one representative arm
        try:
            return reach_envelope(parse_chain(urdf, root, leaf), self._reach_joint_positions())
        except Exception as exc:  # no envelope is a missing prior, never a wrong one
            logger.warning("describe_reach: envelope for %s→%s unavailable: %s", root, leaf, exc)
            return None

    def _reach_joint_positions(self) -> dict[str, float]:
        """Current joint angles for the reachability IK, read from the env observation. Adapters with a
        richer/faster joint source may override.
        """
        try:
            obs = self.env.get_observation()
            jp = (obs.extra or {}).get("joint_positions") if obs is not None else None
            return dict(jp) if jp else {}
        except Exception as exc:  # an unreadable pose falls back to the chain's zero pose
            logger.warning("reachability: joint state unreadable, assuming zero pose: %s", exc)
            return {}
