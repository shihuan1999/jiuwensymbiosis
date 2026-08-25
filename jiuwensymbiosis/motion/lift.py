# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Raising a held payload clear of what it was picked off — ``motion.lift``.

A body that can raise and lower whatever carries its shoulders owes one thing after a grasp:
get the payload up to a height it can travel at. That is the same job whatever the torso
mechanism is (a three-link pitch lifter, a prismatic column) and whatever the end effector is
— which is why it lives here rather than in an adapter.

Two things stay with the body:

* **where the contact points end up** at the transit height (the ``lift_plan`` hook) — the
  same end-effector axis ``motion/dual_arm.py`` draws for grasping;
* **the transit height itself** and the ramp, which are tuning.

Duck-typed ``api``: this module must not import the api layer
(``tests/unit_tests/test_layering.py`` enforces it).
"""

from __future__ import annotations

import logging
from typing import Any

from jiuwensymbiosis.motion import dual_arm
from jiuwensymbiosis.motion.dual_arm import ARMS, _geometry_of, _torso_state, arm_setup, both

logger = logging.getLogger(__name__)


def lift_plan(api: Any, box: Any, chains: dict, q_upright: dict, target_z_m: float) -> dict:
    """Where each contact point should be once the payload is up at ``target_z_m``.

    An END-EFFECTOR fact: it is computed from where the contacts are RIGHT NOW (so the grip
    the body actually achieved is preserved) with only the height replaced. Plates must keep
    their gap bit-for-bit; a gripper only has to stay closed.
    """
    override = getattr(api, "lift_plan", None)
    if override is None:
        raise NotImplementedError(
            f"{type(api).__name__} implements lift_to_clearance but defines no lift_plan hook — "
            "where its contact points sit depends on the end effector."
        )
    return override(box, chains, q_upright, target_z_m)


def lift_to_clearance(api: Any, box: dict | None = None, upright_tol_rad: float = 0.05) -> dict:
    """Stand the torso up and raise the held payload to the body's transit height.

    Plans against the UPRIGHT torso rather than the current one, so the command is never
    aimed at a pose the body is about to leave: standing up holds the arm joints rigid, so
    forward kinematics with the current arm angles and the torso at zero says exactly where
    each contact will be *after* standing. Keeping that x/y and replacing only z sends both
    contacts to the same absolute height, which is what preserves the grip and keeps the
    payload level.

    Arms and torso are commanded in ONE move so they ramp together. The endpoints are exact;
    the grip is not held constant mid-ramp, which is the price of a single faster lift.
    """
    env = api.env
    cfg = getattr(env, "cfg", None)
    payload = box if isinstance(box, dict) else getattr(api, "last_grasped", None)
    if not payload:
        return {"ok": False, "reason": "no_box_to_lift"}
    geo = _geometry_of(payload)
    if geo is None:
        return {"ok": False, "reason": "incomplete_box_payload"}

    chains, arm_joints = arm_setup(api)
    q_fixed = _torso_state(api)
    if q_fixed is None:
        return {"ok": False, "reason": "no_joint_state"}
    q_all = env.low_level.get_joint_positions() or {}
    cur = {a: {j: q_all.get(j, 0.0) for j in arm_joints[a]} for a in ARMS}

    lift_joints = list(env.lift_limits or ())
    upright = dict.fromkeys(lift_joints, 0.0)
    lifter_from = {j: float(q_fixed[j]) for j in lift_joints}
    stand_needed = any(abs(v) > upright_tol_rad for v in lifter_from.values())
    q_upright = {**q_all, **upright}          # FK: arms held, torso at zero
    q_fixed_upright = {**q_fixed, **upright}  # IK: solve against the upright torso

    target_z = float(getattr(cfg, "transit_lift_z_m", 0.0))
    lifted = lift_plan(api, geo, chains, q_upright, target_z)

    ik = {}
    for arm, chain, tgt, c in both(chains, lifted, cur):
        # Resolved through the module so a test patching the shared seam reaches this too.
        ik[arm] = dual_arm.solve_arm_ik(chain, q_fixed_upright, arm_joints[arm], tgt, q_init=c,
                                         check_collision=True,
                                         package_dir=getattr(cfg, "urdf_package_dir", None))
    if not all(r.converged for r in ik.values()):
        return {"ok": False, "reason": "lift_unreachable",
                "pos_err_m": {a: r.pos_err_m for a, r in ik.items()}}

    cmd: dict[str, float] = {}
    for r in ik.values():
        cmd.update(r.q)
    if stand_needed:
        cmd.update(upright)
    logger.info("lift_to_clearance -> z=%.3f%s (one move)",
                target_z, f", torso to 0 from {lifter_from}" if stand_needed else "")
    env.move_named_joints(cmd, ramp_duration_s=getattr(cfg, "lifter_ramp_duration_s", None))
    return {"ok": True, "target_z_m": target_z, "stood_up": stand_needed,
            "lifter_from": lifter_from}
