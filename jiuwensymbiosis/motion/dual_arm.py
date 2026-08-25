# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Two arms driven in coordination — the part that is the same whatever they carry.

``motion.dual_arm`` is a TOPOLOGY capability: it says the body has two arms it can command
together, and it decides which ACTION a plan calls (``dual_arm_grasp`` / ``dual_arm_place``).
What the arms hold — plates that clamp a face each side, a gripper per arm, a hand — is the
separate ``grasp.*`` axis, and it is the only part that differs.

So the split this module draws is:

* **here (topology)** — solve both arms onto a set of contact waypoints, check self-collision,
  drive the approach → descend → contact sequence, confirm by force. Identical for any
  two-arm body.
* **the body (end effector)** — WHERE those waypoints go. Supplied through the ``grasp_plan``
  hook, which returns ``(approach, descend, clamp)``, one :class:`ArmTarget` per arm. There is
  no generic default and there must not be one: a body that does not say how it makes contact
  has not been described, and guessing would put metal on an object.

``ArmTarget`` carries its own tool-frame axes (``approach_local`` / ``paddle_local`` /
``tcp_offset_local``) precisely so this module needs nothing from the body's own geometry
module — those are facts about the end effector, and the end effector is what produced the
target.

The body supplies two things through the Env contract, both already there for other reasons:
``arm_chains`` (root→leaf per arm) and ``arm_joints`` (which joints each arm actuates — NOT
derivable from the chain, since a chain rooted at ``base_link`` runs through the torso the IK
must hold fixed).

Duck-typed ``api`` on purpose: this module must not import the api layer
(``tests/unit_tests/test_layering.py`` enforces it).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

from jiuwensymbiosis.kinematics.ik import IKResult, ik_solve_pose
from jiuwensymbiosis.kinematics.urdf_chain import Chain

logger = logging.getLogger(__name__)

ARMS: tuple[str, str] = ("left", "right")


@dataclass
class ArmTarget:
    """One arm's contact waypoint, self-contained.

    The three ``*_local`` fields are the END EFFECTOR's frame convention — which local axis
    points along the approach, which along the contact face, and where the contact point sits
    relative to the chain leaf. They travel with the target so the shared solve below needs no
    knowledge of what is bolted to the wrist.
    """

    arm: str
    pos_m: tuple[float, float, float]              # contact target (base frame, m)
    approach: tuple[float, float, float]           # base dir the tool approach axis aligns to
    paddle: tuple[float, float, float]             # base dir the tool contact-face axis aligns to
    tcp_offset_local: tuple[float, float, float]   # contact point offset from leaf, tool frame (m)
    approach_local: tuple[float, float, float] = (0.0, 0.0, 1.0)  # tool axis that is "forward"
    paddle_local: tuple[float, float, float] = (1.0, 0.0, 0.0)    # tool axis normal to the face


@dataclass
class GraspPlan:
    ok: bool
    reason: str
    lifter_center_z_mm: float
    approach: dict[str, ArmTarget]   # clear of the object, above the contact height
    descend: dict[str, ArmTarget]    # straight down to contact height, still clear
    clamp: dict[str, ArmTarget]      # in onto the object
    ik: dict[str, IKResult]          # IK at the contact pose


def both(*maps):
    """Yield ``(arm, *values)`` for both arms, reading each mapping once.

    The single place that assumes a two-arm mapping is keyed by exactly left/right: a missing
    arm raises KeyError HERE, with the mapping in hand, instead of letting a half-planned move
    reach the hardware.
    """
    for arm in ARMS:
        yield (arm, *(m[arm] for m in maps))


def solve_arm_ik(
    chain: Chain,
    q_fixed: dict,
    joint_names: list[str],
    tgt: ArmTarget,
    *,
    pos_tol_m: float = 0.016,
    q_init: dict | None = None,
    max_iters: int = 1500,
    n_restarts: int = 4,
    check_collision: bool = False,
    package_dir: str | None = None,
) -> IKResult:
    """Solve full-orientation IK for ONE arm onto ``tgt``.

    ``joint_names`` is which joints this arm actuates — everything else in the chain (a torso
    lifter, a waist) is held at ``q_fixed``. That distinction cannot come from the chain: a
    chain rooted at ``base_link`` contains the torso joints too, and solving them here would
    let a grasp quietly re-pose the body.

    Dispatches to the pinocchio solver (analytic Jacobian + random restarts) when pinocchio is
    importable and the chain carries its URDF path/leaf; otherwise falls back to the retained
    legacy DLS. ``q_init`` warm-starts either solver.
    """
    from jiuwensymbiosis.kinematics import ik_pinocchio as _pik

    init = q_init if q_init is not None else {j: q_fixed.get(j, 0.0) for j in joint_names}
    if _pik.pin_available() and getattr(chain, "urdf_path", "") and getattr(chain, "leaf_link", ""):
        try:
            return _pik.solve_pose_ik_pin(
                chain.urdf_path, joint_names, chain.leaf_link, chain.limits(),
                tgt.pos_m, approach_target=tgt.approach, paddle_target=tgt.paddle,
                tool_approach_local=tgt.approach_local, tool_paddle_local=tgt.paddle_local,
                tcp_offset_local=tgt.tcp_offset_local, q_fixed=q_fixed, q_init=init,
                pos_tol_m=pos_tol_m, max_iters=min(max_iters, 200), n_restarts=n_restarts,
                check_collision=check_collision, package_dir=package_dir,
            )
        except Exception as exc:  # never let IK-backend errors abort a grasp; fall back
            logger.warning("pinocchio IK failed (%s); falling back to legacy DLS", exc)

    return ik_solve_pose(
        chain, q_fixed, joint_names, tgt.pos_m,
        approach_target=tgt.approach, paddle_target=tgt.paddle,
        tool_approach_local=tgt.approach_local, tool_paddle_local=tgt.paddle_local,
        tcp_offset_local=tgt.tcp_offset_local, q_init=init, pos_tol_m=pos_tol_m, max_iters=max_iters,
    )


def solve_planned_grasp(
    contact_z_mm: float,
    chains: dict[str, Chain],
    arm_joints: dict[str, list[str]],
    q_fixed: dict[str, float],
    *,
    approach: dict[str, ArmTarget],
    descend: dict[str, ArmTarget],
    clamp: dict[str, ArmTarget],
    pos_tol_m: float = 0.016,
    ik_max_iters: int = 1500,
    check_collision: bool = False,
    package_dir: str | None = None,
) -> GraspPlan:
    """Solve both arms onto an ALREADY-PLANNED set of contact waypoints.

    The waypoints come from the body's ``grasp_plan`` hook (end-effector axis); everything
    here — two-arm IK and the self-collision check — is the same job for any dual-arm body.
    """
    ik = {
        arm: solve_arm_ik(chain, q_fixed, arm_joints[arm], tgt, pos_tol_m=pos_tol_m,
                          max_iters=ik_max_iters, check_collision=check_collision,
                          package_dir=package_dir)
        for arm, chain, tgt in both(chains, clamp)
    }
    ok = all(r.converged for r in ik.values())
    return GraspPlan(ok, "" if ok else "ik_no_converge", contact_z_mm, approach, descend, clamp, ik)


def grasp_plan(api: Any, target: Any, **kwargs: Any) -> tuple[dict, dict, dict]:
    """The END-EFFECTOR hook: where the two contact points go, for THIS body's end effector.

    No generic default, deliberately. Plates, a gripper per arm and a hand put their contact
    points in different places for the same object, and a body that has not said which it has
    is a body nobody has described — guessing would drive metal into an object.
    """
    override = getattr(api, "grasp_plan", None)
    if override is None:
        raise NotImplementedError(
            f"{type(api).__name__} implements a dual-arm action but defines no grasp_plan hook. "
            "The shared coordination cannot know where THIS end effector should make contact "
            "(plates clamp a face each side; a gripper per arm does not). Define grasp_plan("
            "target, **kw) -> (approach, descend, clamp)."
        )
    return override(target, **kwargs)


def arm_setup(api: Any) -> tuple[dict[str, Chain], dict[str, list[str]]]:
    """``(chains, arm_joints)`` read off the Env contract, or a clear error naming what is missing."""
    from jiuwensymbiosis.kinematics.urdf_chain import parse_chain

    env = api.env
    chains_spec = getattr(env, "arm_chains", None)
    joints = getattr(env, "arm_joints", None)
    if not chains_spec or not joints:
        missing = [n for n, v in (("arm_chains", chains_spec), ("arm_joints", joints)) if not v]
        raise NotImplementedError(
            f"{getattr(env, 'name', type(env).__name__)}: a dual-arm action needs {missing} on the "
            "Env — the chains to solve and which joints each arm actuates (the rest of the chain "
            "is held fixed)."
        )
    urdf = getattr(env, "urdf_path", None)
    chains = {arm: parse_chain(urdf, root, leaf) for arm, (root, leaf) in chains_spec.items()}
    return chains, {arm: list(joints[arm]) for arm in chains}


# ---------------------------------------------------------------------------
# The shared two-arm grasp.
# ---------------------------------------------------------------------------
# Steps that only some bodies have are gated on the CAPABILITY, not stubbed out by the body:
# a fixed-base two-arm robot declares no ``motion.lift`` and the lifter step simply does not
# run — the same way ``motion/approach.py`` asks whether the body can turn before turning it.
# What is left behind a hook is only what genuinely differs BETWEEN bodies that all have the
# capability: where the end effector makes contact, and how a grip is confirmed.


def ready_plan(api: Any) -> dict[str, ArmTarget] | None:
    """Transit pose to hold the arms in before descending — an END-EFFECTOR fact.

    Where two paddles wait clear of a table is not where two grippers wait. ``None`` (no hook)
    means the body wants no transit waypoint, which is legal: it just descends from wherever
    the arms are.
    """
    override = getattr(api, "ready_plan", None)
    return override() if override else None


def lifter_for_object(api: Any, box: Any, current: dict, waist_yaw: float) -> Any:
    """Which torso pose best reaches ``box`` — only asked of a body with ``motion.lift``.

    Returns an object with ``found`` / ``reason`` / ``improves`` / ``q_lifter``. A body with a
    lifter but no search declares none and the caller keeps the current pose.
    """
    override = getattr(api, "lifter_for_object", None)
    return override(box, current, waist_yaw) if override else None


def contact_confirmed(api: Any) -> tuple[bool, Any]:
    """``(held, detail)`` — did BOTH arms actually make contact.

    SAFETY: the caller must not report a grasp, and must not let a lift follow, without this.
    A body that cannot sense contact says so by not defining the hook, and gets ``(False,
    reason)`` rather than an optimistic default: "I could not tell" is not "I am holding it".
    """
    override = getattr(api, "contact_confirmed", None)
    if override is None:
        return False, {"reason": "no_contact_sensing"}
    return override()


def _ramp(api: Any, name: str) -> float | None:
    """Per-move ramp override off the body config; None = the driver's own default."""
    return getattr(getattr(api.env, "cfg", None), name, None)


def _torso_state(api: Any) -> dict[str, float] | None:
    """Current angles of the joints the arm solve holds fixed, or None if unreadable."""
    names = list(getattr(api.env, "torso_joints", ()) or ())
    try:
        q = api.env.low_level.get_joint_positions()
    except Exception as exc:  # unreadable state aborts the grasp, never guesses
        logger.warning("dual-arm grasp: joint state unreadable: %s", exc)
        return None
    if not q or any(n not in q for n in names):
        return None
    return {n: q[n] for n in names}


def dual_arm_grasp(api: Any, target: Any = None, object_name: str = "box") -> dict:
    """Both arms take hold of an already-detected target and confirm the grip by force.

    Order matters and is the same for any two-arm body: reach the transit pose BEFORE moving
    the torso (so the arms are clear while the body leans), then descend outside the object,
    then close. Descending before closing is what keeps the arms from sweeping up into a
    table from below.
    """
    from jiuwensymbiosis.perception.object_geometry import ObjectGeometry3D

    env = api.env
    caps = getattr(env, "capabilities", frozenset())
    # A new grasp invalidates every prior belief about what is held or where things are.
    api.last_detection = api.last_detection  # (kept; the detection below is read from it)
    api.last_surface = None
    env.holding_payload = False

    det = target if isinstance(target, dict) else api.last_detection
    if not det or not det.get("ok"):
        return {"ok": False, "reason": "no_detection"}
    box = ObjectGeometry3D(
        True, "", tuple(det["center_mm"]), det["width_mm"], det["height_mm"],
        det["front_x_mm"], det["top_z_mm"], det["n_points"], back_x_mm=det.get("back_x_mm", 0.0),
    )

    chains, arm_joints = arm_setup(api)
    q_fixed = _torso_state(api)
    if q_fixed is None:
        return {"ok": False, "reason": "no_joint_state"}

    # A body that can raise/lower its shoulders picks the torso pose that best reaches the
    # object first. No motion.lift → this whole step is absent, not stubbed.
    lp = None
    if "motion.lift" in caps:
        lp = lifter_for_object(api, box, q_fixed, q_fixed.get(getattr(env, "waist_joint", ""), 0.0))
    if lp is not None and not lp.found:
        return {"ok": False, "reason": lp.reason or "unreachable_any_lifter"}

    transit_ramp = _ramp(api, "arm_transit_ramp_duration_s")
    contact_ramp = _ramp(api, "arm_contact_ramp_duration_s")

    # Transit pose BEFORE the torso moves — the arms must be clear while the body leans.
    ready_t = ready_plan(api)
    ready_q: dict[str, dict] = {}
    if ready_t:
        for arm, chain, tgt in both(chains, ready_t):
            r = solve_arm_ik(chain, q_fixed, arm_joints[arm], tgt)
            if r.converged:
                ready_q[arm] = r.q
        _move(env, ready_q, transit_ramp)

    if lp is not None and getattr(lp, "improves", False):
        env.set_lifter(lp.q_lifter)
        # Do NOT re-detect: the object's base-frame coordinate is invariant to the torso move
        # (the base frame sits below it), and leaning often carries the object out of the
        # camera's view — a re-detect would then return partial geometry. Only the torso
        # angles changed, so re-read just those.
        q_fixed = _torso_state(api)
        if q_fixed is None:
            return {"ok": False, "reason": "no_joint_state"}

    _join_warmup(env)
    approach_t, descend_t, clamp_t = grasp_plan(api, box)
    plan = solve_planned_grasp(
        box.center_mm[2], chains, arm_joints, q_fixed,
        approach=approach_t, descend=descend_t, clamp=clamp_t,
        check_collision=True, package_dir=getattr(getattr(env, "cfg", None), "urdf_package_dir", None))
    if not plan.ok:
        return {"ok": False, "reason": plan.reason,
                "ik": {a: plan.ik[a].pos_err_m for a in plan.ik}}

    # Top-down: descend to contact height while still clear of the object, then close in.
    # The descend solve warm-starts from the transit pose.
    de = {}
    for arm, chain, tgt in both(chains, plan.descend):
        r = solve_arm_ik(chain, q_fixed, arm_joints[arm], tgt, q_init=ready_q.get(arm))
        if r.converged:
            de[arm] = r.q
    _move(env, de, transit_ramp)
    _move(env, {a: plan.ik[a].q for a in ARMS}, contact_ramp)

    held, detail = contact_confirmed(api)
    if not held:
        return {"ok": False, "reason": "no_contact", "ft": detail}

    api.last_detection = det          # what dual_arm_place will act on if called bare
    env.holding_payload = True        # RecoveryRail: retreat without opening, i.e. without dropping
    return {"ok": True, "object": object_name, "box": det, "ft": detail}


def _move(env: Any, q_by_arm: dict, ramp_s: float | None) -> None:
    """Command every named joint in ONE message so the arms move together."""
    combined: dict = {}
    for arm_q in q_by_arm.values():
        combined.update(arm_q)
    if combined:
        env.move_named_joints(combined, ramp_duration_s=ramp_s)


def _join_warmup(env: Any) -> None:
    """Wait for a background self-collision model build, when the body does one at connect.

    Best-effort and body-agnostic: a body without the warm-up has no thread and this is a
    no-op. Joining matters because rebuilding the model here would pay a multi-second cost in
    exactly the gap between arriving at the object and closing on it.
    """
    warm = getattr(env, "_warm_thread", None)
    if warm is not None and getattr(warm, "is_alive", lambda: False)():
        warm.join()


# ---------------------------------------------------------------------------
# The shared two-arm place.
# ---------------------------------------------------------------------------
_SURFACE_FIELDS = ("front_x_mm", "back_x_mm", "center_mm", "width_mm", "surface_z_mm")


def landing_xy(box: Any, surface: dict | None, margin_mm: float,
               carried_xy: tuple[float, float],
               default_z_mm: float | None = None) -> tuple[float, float, float | None] | dict:
    """Where on the surface the object should land, or a failure dict if it cannot fit.

    Pure geometry, no arms in it. With a sensed footprint the object lands FULLY on the
    surface clear of the edges — Y centred, X just inside the near edge so the whole object
    sits on it and the arms reach the least far. Landing at the CARRIED xy instead is what
    drops things on the rim or in mid-air, because the carried position was never checked
    against the surface. Without a surface there is nothing to check against, so the carried
    xy is all there is.

    The fit checks return a reason BEFORE anything moves: an object too wide or too deep for
    the surface is a plan that should fail standing still.
    """
    if surface is None:
        # Nothing to check against, so the carried xy is all there is; the landing height is
        # whatever the caller stated (None = leave the height to the plan).
        return carried_xy[0], carried_xy[1], default_z_mm
    if any(f not in surface for f in _SURFACE_FIELDS):
        return {"ok": False, "reason": "incomplete_surface_payload"}
    near_x, far_x = float(surface["front_x_mm"]), float(surface["back_x_mm"])
    centre_y, width = float(surface["center_mm"][1]), float(surface["width_mm"])
    # Depth along x; fall back to the width when the far face was never detected.
    depth = (box.back_x_mm - box.front_x_mm) if box.back_x_mm > box.front_x_mm else box.width_mm
    if box.width_mm / 2.0 + margin_mm > width / 2.0:
        return {"ok": False, "reason": "box_wider_than_table"}
    if depth + 2.0 * margin_mm > (far_x - near_x):
        return {"ok": False, "reason": "box_deeper_than_table"}
    return near_x + depth / 2.0 + margin_mm, centre_y, float(surface["surface_z_mm"])


def place_plan(api: Any, box: Any, landing: tuple[float, float, float | None],
               held: dict, **kwargs: Any) -> tuple[dict, dict, dict]:
    """Waypoints for setting the object down — an END-EFFECTOR fact, like ``grasp_plan``.

    ``held`` is where each contact point is RIGHT NOW (measured, not re-derived), because how
    an end effector must keep its hold on the way down is its own business: plates have to
    preserve the gap they actually achieved, a gripper just stays closed.
    """
    override = getattr(api, "place_plan", None)
    if override is None:
        raise NotImplementedError(
            f"{type(api).__name__} implements dual_arm_place but defines no place_plan hook."
        )
    return override(box, landing, held, **kwargs)


def lower_and_release(api: Any, chains: dict, arm_joints: dict, q_fixed: dict, *,
                      plan: tuple[dict, dict, dict], held_q: dict, **kwargs: Any) -> dict:
    """Descend to the landing height and let go — the two coupled END-EFFECTOR steps.

    Coupled on purpose: how you keep hold on the way down and how you let go afterwards are
    the same fact about the end effector. Plates must descend without their gap widening and
    then open outward below the rim; a gripper descends straight and opens its fingers.

    Returns ``{"ok": bool, ...}``. The caller clears ``holding_payload`` on success BEFORE the
    raise, so a failure during the raise cannot leave a phantom payload behind.
    """
    override = getattr(api, "lower_and_release", None)
    if override is None:
        raise NotImplementedError(
            f"{type(api).__name__} implements dual_arm_place but defines no lower_and_release hook."
        )
    return override(chains, arm_joints, q_fixed, plan, held_q, **kwargs)


def lifter_for_place(api: Any, clamp: dict, q_fixed: dict) -> Any:
    """Smallest torso lean from which both arms reach the place targets, or None."""
    override = getattr(api, "lifter_for_place", None)
    return override(clamp, q_fixed) if override else None


def dual_arm_place(api: Any, target: dict | None = None, surface: dict | None = None,
                   **kwargs: Any) -> dict:
    """Set a two-arm-held payload down on a sensed surface and withdraw.

    Order is the shared part and it matters: check the fit BEFORE moving, lean only as far as
    the arms need, descend, let go, and only then raise clear — lifting off top-down instead
    of dragging across the surface.
    """
    env = api.env
    caps = getattr(env, "capabilities", frozenset())
    box_payload = target if isinstance(target, dict) else getattr(api, "last_grasped", None)
    if not box_payload:
        return {"ok": False, "reason": "no_box_to_place"}
    if surface is None:
        surface = api.last_surface

    box = _geometry_of(box_payload)
    if box is None:
        return {"ok": False, "reason": "incomplete_box_payload"}

    chains, arm_joints = arm_setup(api)
    q_fixed = _torso_state(api)
    if q_fixed is None:
        return {"ok": False, "reason": "no_joint_state"}
    q_all = env.low_level.get_joint_positions() or {}
    held_q = {a: {j: q_all.get(j, 0.0) for j in arm_joints[a]} for a in ARMS}

    held = carried_contacts(api, chains, q_all, box, **kwargs)
    carried = (500.0 * (held["left"][0] + held["right"][0]),
               500.0 * (held["left"][1] + held["right"][1]))
    margin = float(getattr(getattr(env, "cfg", None), "place_edge_margin_mm", 0.0))
    landing = landing_xy(box, surface, margin, carried, kwargs.get("surface_z_mm"))
    if isinstance(landing, dict):
        return landing

    approach_t, descend_t, clamp_t = place_plan(api, box, landing, held, **kwargs)

    lp = lifter_for_place(api, clamp_t, q_fixed) if "motion.lift" in caps else None
    if lp is not None and not lp.found:
        return {"ok": False, "reason": lp.reason or "place_unreachable_any_lifter"}
    if lp is not None and getattr(lp, "improves", False):
        env.move_named_joints({**lp.q_lifter, **held_q["left"], **held_q["right"]})
        q_fixed = _torso_state(api)
        if q_fixed is None:
            return {"ok": False, "reason": "no_joint_state"}

    out = lower_and_release(api, chains, arm_joints, q_fixed,
                            plan=(approach_t, descend_t, clamp_t), held_q=held_q, **kwargs)
    if not out.get("ok"):
        return out
    # Cleared here, not at the return: the payload is already let go, so a failure during the
    # raise below must not make RecoveryRail preserve a phantom one.
    env.holding_payload = False

    q_fixed = _torso_state(api) or q_fixed
    up = {}
    for arm, chain, tgt in both(chains, approach_t):
        r = solve_arm_ik(chain, q_fixed, arm_joints[arm], tgt, q_init=held_q.get(arm),
                         check_collision=True,
                         package_dir=getattr(getattr(env, "cfg", None), "urdf_package_dir", None))
        if r.converged:
            up[arm] = r.q
    _move(env, up, _ramp(api, "arm_transit_ramp_duration_s"))
    return {"ok": True, "leaned": bool(lp.improves) if lp is not None else False,
            "lifter": getattr(lp, "q_lifter", None),
            "surface_z_mm": landing[2], "landing_mm": [landing[0], landing[1]],
            "carried_mm": [carried[0], carried[1]]}


def carried_contacts(api: Any, chains: dict, q_all: dict, box: Any, **kwargs: Any) -> dict:
    """Where each contact point is RIGHT NOW, by FK of the measured joints.

    Measured rather than re-derived from the object: the grip actually achieved differs from
    the planned one (compliance, a different inset at grasp time), and planning the descent
    off the re-derived spacing is what squeezes or drops the payload on the way down.
    """
    override = getattr(api, "carried_contacts", None)
    if override is not None:
        return override(chains, q_all, box, **kwargs)
    raise NotImplementedError(
        f"{type(api).__name__} implements dual_arm_place but defines no carried_contacts hook — "
        "where its contact points sit depends on the end effector."
    )


def _geometry_of(payload: dict) -> Any:
    """A measured-object payload dict → the geometry record, or None when fields are missing."""
    from jiuwensymbiosis.perception.object_geometry import ObjectGeometry3D

    try:
        return ObjectGeometry3D(
            True, "", tuple(payload["center_mm"]), payload["width_mm"], payload["height_mm"],
            payload["front_x_mm"], payload["top_z_mm"], payload["n_points"],
            back_x_mm=payload.get("back_x_mm", 0.0),
        )
    except (KeyError, TypeError):
        return None

