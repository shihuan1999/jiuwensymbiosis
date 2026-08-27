# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Observability, AX=XB residual, target-consistency and acceptance policies.

Body-agnostic quality measurements for hand-eye calibration, plus the two
mode-specific acceptance policies (eye-in-hand / eye-to-hand) that turn the
unified :class:`CalibrationQualityReport` measurements into a
:class:`CalibrationDecision`.

What lives here:

* :func:`axxb_residuals` — AX=XB consistency across station pairs; the
  mode-specific A/B convention is locked in :func:`_axxb_relative_pair` so the
  eye-in-hand and eye-to-hand engines never drift.
* :func:`board_origin_points` / :func:`board_origin_in_base` — eye-in-hand
  rigidity invariant ``T_base_target`` (constant across stations as the arm
  moves with the board clamped to it). The eye-to-hand invariant
  ``T_flange_target`` is computed by :func:`t_flange_target_consistency` and
  :func:`target_consistency_report`, which use the ``inv(T_base_flange)`` form.
* :func:`observability_report` — relative-motion excitation quality of the
  flange AND camera sides (a degenerate camera side is the classic eye-to-hand
  failure).
* :func:`evaluate_calibration_quality` — the single entry point that assembles a
  :class:`CalibrationQualityReport` from solved stations + the hand-eye
  transform, computing the correct target-consistency invariant per mode.
* :class:`EyeInHandAcceptancePolicy` / :class:`EyeToHandAcceptancePolicy` —
  thresholds -> decision. The eye-in-hand policy preserves the Piper legacy CLI
  thresholds (reproj/axxb/board-origin spread); the eye-to-hand policy requires
  observability + target consistency + (caller-checked) reload smoke.

Quality sub-reports hold measurements only; ``accept``/``reasons`` live in the
:class:`CalibrationDecision` the policies produce.
"""

from __future__ import annotations

import itertools
from collections.abc import Sequence
from dataclasses import dataclass, fields
from typing import Any, Literal

import numpy as np
from scipy.spatial.transform import Rotation

from jiuwensymbiosis.calibration.domain.models import (
    AxxbResidualReport,
    CalibrationDecision,
    CalibrationQualityReport,
    CrossCheckReport,
    ReprojectionReport,
    Station,
    TargetConsistencyReport,
    VerifyStat,
    _stat,
)
from jiuwensymbiosis.utils.geometry import (
    invert_transform,
    make_transform,
    orthonormalize,
    rotation_angle_deg,
)

# =============================================================================
# AX=XB residual — the SINGLE A/B convention source
# =============================================================================
CalibrationMode = Literal["eye_in_hand", "eye_to_hand"]


def _validate_mode(mode: CalibrationMode) -> None:
    """Reject an unknown calibration mode instead of silently using ETH semantics."""
    if mode not in ("eye_in_hand", "eye_to_hand"):
        raise ValueError(f"mode must be 'eye_in_hand' or 'eye_to_hand', got {mode!r}")


def _validate_finite_thresholds(value: Any) -> None:
    """Validate every numeric field of a threshold dataclass.

    A NaN threshold makes every comparison in the acceptance policy false and
    therefore fails open.  Keep this check at construction time so profile/CLI
    values cannot silently disable a gate.
    """
    for field in fields(value):
        raw = getattr(value, field.name)
        try:
            finite = bool(np.isfinite(float(raw)))
        except (TypeError, ValueError):
            finite = False
        if not finite:
            raise ValueError(f"{type(value).__name__}.{field.name} must be finite, got {raw!r}")


def _reprojection_invalid(report: ReprojectionReport) -> bool:
    """Return whether a reprojection report is missing or contains bad values."""
    if not report.per_view_rms_px:
        return True
    try:
        values = np.asarray(report.per_view_rms_px, dtype=np.float64)
    except (TypeError, ValueError):
        return True
    if values.ndim != 1 or not np.all(np.isfinite(values)) or np.any(values < 0.0):
        return True
    summary = report.summary
    return not all(np.isfinite(float(x)) for x in (summary.mean, summary.max, summary.std))


def _all_finite(*values: float) -> bool:
    """Return whether every quality value is numeric and finite."""
    try:
        return all(bool(np.isfinite(float(value))) for value in values)
    except (TypeError, ValueError):
        return False


def _axxb_relative_pair(
    ti: np.ndarray,
    tj: np.ndarray,
    ci: np.ndarray,
    cj: np.ndarray,
    *,
    mode: CalibrationMode = "eye_in_hand",
) -> tuple[np.ndarray, np.ndarray]:
    """Relative transforms for one station pair (the SINGLE source of the A/B convention).

    Returns ``(A_ij, B_ij)`` where ``B_ij`` is always
    ``T_cam_target_i @ inv(T_cam_target_j)``.  The flange-side convention is
    mode-specific because OpenCV's eye-to-hand solve consumes
    ``T_base_flange⁻¹``:

      * eye-in-hand: ``A_ij = inv(T_base_flange_i) @ T_base_flange_j``;
      * eye-to-hand: ``A_ij = T_base_flange_i @ inv(T_base_flange_j)``.

    :func:`axxb_residuals` (solve-quality) and :func:`observability_report`
    both call THIS helper so the A/B convention is locked in one place — two
    independently-coded formulas would drift.
    """
    _validate_mode(mode)
    ti_arr = np.asarray(ti, dtype=np.float64)
    tj_arr = np.asarray(tj, dtype=np.float64)
    if mode == "eye_to_hand":
        ti_arr = invert_transform(ti_arr)
        tj_arr = invert_transform(tj_arr)
    a = invert_transform(ti_arr) @ tj_arr  # flange motion
    b = ci @ invert_transform(cj)  # camera motion (target-in-cam convention)
    return a, b


def axxb_residuals(
    stations: list[Station],
    tf_handeye: np.ndarray,
    *,
    mode: CalibrationMode = "eye_in_hand",
) -> tuple[VerifyStat, VerifyStat]:
    """AX=XB consistency across station pairs -> (rot_deg stat, trans_mm stat).

    ``mode`` defaults to eye-in-hand for compatibility with the legacy helper;
    eye-to-hand callers must pass ``mode="eye_to_hand"`` so flange motion is
    computed from the inverted robot poses used by OpenCV.
    """
    _validate_mode(mode)
    rot_errs: list[float] = []
    trans_errs: list[float] = []
    n = len(stations)
    for i in range(n):
        for j in range(i + 1, n):
            ti, tj = stations[i].tf_base_flange, stations[j].tf_base_flange
            ci, cj = stations[i].detection.tf_cam_target, stations[j].detection.tf_cam_target
            if ci is None or cj is None:
                continue  # caller filtered; defensive narrowing only
            a, b = _axxb_relative_pair(ti, tj, np.asarray(ci), np.asarray(cj), mode=mode)
            m = (a @ tf_handeye) @ invert_transform(tf_handeye @ b)  # ideal: identity
            c = np.clip((np.trace(m[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
            rot_errs.append(float(np.degrees(np.arccos(c))))
            trans_errs.append(float(np.linalg.norm(m[:3, 3])))
    return _stat(rot_errs), _stat(trans_errs)


# =============================================================================
# Board origin invariants
# =============================================================================
def board_origin_points(stations: list[Station], tf_handeye: np.ndarray) -> np.ndarray:
    """Per-station board origin in base frame, shape (N, 3) mm.

    Computes ``T_base_target = (T_base_flange @ tf_handeye) @ T_cam_target``.
    For eye-in-hand ``tf_handeye = T_flange_cam`` this is the constant board
    origin in base (the arm moves with the board); for eye-to-hand
    ``tf_handeye = T_base_cam`` this expression is NOT a rigidity invariant —
    use :func:`t_flange_target_consistency` for eye-to-hand instead.
    """
    pts = []
    for s in stations:
        t_base_target = (s.tf_base_flange @ tf_handeye) @ s.detection.tf_cam_target
        pts.append(t_base_target[:3, 3])
    return np.asarray(pts)


def board_origin_in_base(stations: list[Station], tf_handeye: np.ndarray) -> tuple[np.ndarray, VerifyStat]:
    """Recompute the board origin in base from every station -> (mean, spread).

    This is the eye-in-hand rigidity invariant (``invariant_frame="base_target"``).
    """
    pts = board_origin_points(stations, tf_handeye)
    mean = pts.mean(axis=0)
    dists = np.linalg.norm(pts - mean, axis=1)
    return mean, _stat([float(d) for d in dists])


def t_flange_target_consistency(stations: list[Station], tf_base_cam: np.ndarray) -> np.ndarray:
    """Per-station T_flange_target = inv(T_base_flange) @ T_base_cam @ T_cam_target.

    The camera is fixed in base, so the board origin in the flange frame is
    constant across stations. Returns the mean (translation mean + SO(3) mean,
    orthonormalised) as a full 4x4 SE(3). This is the eye-to-hand rigidity
    invariant (``invariant_frame="flange_target"``).
    """
    tfs = []
    for s in stations:
        tf = invert_transform(s.tf_base_flange) @ tf_base_cam @ s.detection.tf_cam_target
        tfs.append(tf)
    tfs_arr = np.stack([np.asarray(t, dtype=np.float64) for t in tfs])
    trans_mean = tfs_arr[:, :3, 3].mean(axis=0)
    rots = tfs_arr[:, :3, :3]
    try:
        rot_mean = _rotation_from_matrix_mean(rots)
    except Exception:
        rot_mean = rots[0]
    return make_transform(orthonormalize(rot_mean), trans_mean)


def _rotation_from_matrix_mean(rots: np.ndarray) -> np.ndarray:
    """Chordal/SO(3) mean of a stack of rotation matrices (3x3 each)."""
    return np.asarray(Rotation.from_matrix(rots).mean().as_matrix(), dtype=np.float64)


def _relative_angle_deg(rel: np.ndarray) -> float:
    """Rotation magnitude (deg) of a 4x4 relative transform's rotation block."""
    r = rel[:3, :3]
    c = float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))


def _relative_translation_mm(rel: np.ndarray) -> float:
    """Translation magnitude (mm) of a 4x4 relative transform."""
    return float(np.linalg.norm(rel[:3, 3]))


# =============================================================================
# Outlier detection (robust median + MAD)
# =============================================================================
def outlier_mask(points: np.ndarray, *, k: float = 3.0) -> tuple[np.ndarray, np.ndarray, float]:
    """Robust outlier flags via median + MAD (NOT mean/std — mean is itself skewed by outliers).

    Returns ``(mask, dists, median_dist)``:
      * ``dists``: each point's distance to the **median** center (mm);
      * ``median_dist``: median of ``dists`` — if this is already large, the error
        is *systematic* (all frames off), not a few outliers, so the caller should
        refuse to drop and suspect scale/axes instead;
      * ``mask``: True where ``dist > median_dist + k*MAD``. When MAD≈0 a 1mm floor
        avoids false drops.
    """
    center = np.median(points, axis=0)
    dists = np.linalg.norm(points - center, axis=1)
    med = float(np.median(dists))
    mad = float(np.median(np.abs(dists - med))) * 1.4826  # normal-consistent MAD
    thr = med + (k * mad if mad > 1e-9 else 1.0)
    return dists > thr, dists, med


# =============================================================================
# Observability — relative-motion excitation quality (flange AND camera sides)
# =============================================================================
@dataclass(frozen=True)
class ObservabilityThresholds:
    """First-version fixed thresholds (CLI/profile-overridable).

    All rotation thresholds are in degrees, all translation in mm — never mixed.
    Camera-side baselines are separate fields so flange and camera excitation
    can never be conflated.
    """

    min_relative_rotation_deg: float = 5.0
    min_axis_separation_deg: float = 15.0
    min_max_rotation_deg: float = 20.0
    min_translation_baseline_mm: float = 30.0
    min_camera_translation_baseline_mm: float = 30.0
    duplicate_rotation_deg: float = 2.0
    duplicate_translation_mm: float = 5.0

    def __post_init__(self) -> None:
        _validate_finite_thresholds(self)


@dataclass(frozen=True)
class _SideMetrics:
    """Per-side (flange or camera) relative-motion metrics used for gating."""

    n_axes: int
    max_rotation_deg: float
    max_translation_mm: float
    max_axis_separation_deg: float
    n_duplicates: int


@dataclass(frozen=True)
class ObservabilityReport:
    """Quality report for the relative-motion excitation of a station set.

    Holds measurements only — accept/reasons are produced by the
    :class:`EyeToHandAcceptancePolicy` (eye-in-hand uses the legacy
    reproj/axxb/board-origin thresholds via :class:`EyeInHandAcceptancePolicy`
    and does not gate on observability). Both the flange side (A_ij) and the
    camera side (B_ij) must show non-degenerate excitation; a fixed camera that
    sees a stationary board yields a degenerate B side.
    """

    n_valid_pairs: int
    # flange side
    n_relative_axes: int
    max_relative_rotation_deg: float
    max_relative_translation_mm: float
    max_axis_separation_deg: float
    # camera side
    n_relative_axes_camera: int
    max_relative_rotation_camera_deg: float
    max_relative_translation_camera_mm: float
    max_axis_separation_camera_deg: float
    n_duplicates: int
    svd_condition_min: float  # reported only, never gates ACCEPT


def _rotation_axis_from_relative(rel: np.ndarray, *, min_angle_deg: float) -> np.ndarray | None:
    """Unit rotation axis of a relative transform, or None if rotation < min_angle."""
    angle_deg = _relative_angle_deg(rel)
    if angle_deg < min_angle_deg:
        return None
    rotvec = Rotation.from_matrix(rel[:3, :3]).as_rotvec()
    n = float(np.linalg.norm(rotvec))
    if n < 1e-12:
        return None
    return np.asarray(rotvec / n, dtype=np.float64)


def _undirected_axis_angle(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest angle between two axes treating a/-a as the same axis (deg)."""
    return float(np.degrees(np.arccos(np.clip(abs(float(np.dot(a, b))), 0.0, 1.0))))


def _dedup_axes(axes: list[np.ndarray], *, duplicate_deg: float) -> list[np.ndarray]:
    distinct: list[np.ndarray] = []
    for ax in axes:
        if all(_undirected_axis_angle(ax, d) > max(1e-6, duplicate_deg) for d in distinct):
            distinct.append(ax)
    return distinct


def _max_axis_separation(axes: list[np.ndarray]) -> float:
    if len(axes) < 2:
        return 0.0
    return max(_undirected_axis_angle(axes[i], axes[j]) for i in range(len(axes)) for j in range(i + 1, len(axes)))


def _svd_condition_min(axes: list[np.ndarray]) -> float:
    if len(axes) < 2:
        return float("inf")
    m = np.stack(axes)
    sv = np.linalg.svd(m, compute_uv=False)
    smax, smin = float(sv[0]), float(sv[-1])
    return float(smax / smin) if smin > 1e-12 else float("inf")


def _side_metrics(rels: list[np.ndarray], *, th: ObservabilityThresholds) -> _SideMetrics:
    """Aggregate relative-motion metrics for one side (A or B)."""
    if not rels:
        return _SideMetrics(0, 0.0, 0.0, 0.0, 0)
    rots = [_relative_angle_deg(r) for r in rels]
    trans = [_relative_translation_mm(r) for r in rels]
    duplicates = sum(
        1
        for ro, tr in zip(rots, trans, strict=True)
        if ro < th.duplicate_rotation_deg and tr < th.duplicate_translation_mm
    )
    axes = [_rotation_axis_from_relative(r, min_angle_deg=th.min_relative_rotation_deg) for r in rels]
    distinct = _dedup_axes([a for a in axes if a is not None], duplicate_deg=th.duplicate_rotation_deg)
    return _SideMetrics(
        n_axes=len(distinct),
        max_rotation_deg=max(rots),
        max_translation_mm=max(trans),
        max_axis_separation_deg=_max_axis_separation(distinct),
        n_duplicates=duplicates,
    )


def observability_report(
    stations: list[Station],
    *,
    mode: CalibrationMode = "eye_in_hand",
    thresholds: ObservabilityThresholds | None = None,
) -> ObservabilityReport:
    """Assess relative-motion excitation quality from BOTH the flange side and camera side.

    For each valid pair (i,j) compute A_ij (flange relative motion) and B_ij
    (camera relative motion) via the shared :func:`_axxb_relative_pair` helper,
    so the convention is identical to :func:`axxb_residuals`. ``mode`` defaults
    to eye-in-hand for compatibility; eye-to-hand callers use the inverted
    robot pose convention. Measurements only;
    the eye-to-hand acceptance policy reads this to gate on flange/camera
    excitation. SVD condition number of the flange axis matrix is reported for
    diagnostics but does NOT gate ACCEPT.
    """
    _validate_mode(mode)
    th = thresholds or ObservabilityThresholds()
    n = len(stations)
    a_rels: list[np.ndarray] = []
    b_rels: list[np.ndarray] = []
    for i in range(n):
        for j in range(i + 1, n):
            ci, cj = stations[i].detection.tf_cam_target, stations[j].detection.tf_cam_target
            if ci is None or cj is None:
                continue
            a_ij, b_ij = _axxb_relative_pair(
                stations[i].tf_base_flange,
                stations[j].tf_base_flange,
                np.asarray(ci),
                np.asarray(cj),
                mode=mode,
            )
            a_rels.append(a_ij)
            b_rels.append(b_ij)

    flange_m = _side_metrics(a_rels, th=th)
    camera_m = _side_metrics(b_rels, th=th)
    svd_cond_min = _svd_condition_min(
        _dedup_axes(
            [
                a
                for a in (_rotation_axis_from_relative(r, min_angle_deg=th.min_relative_rotation_deg) for r in a_rels)
                if a is not None
            ],
            duplicate_deg=th.duplicate_rotation_deg,
        )
    )

    return ObservabilityReport(
        n_valid_pairs=len(a_rels),
        n_relative_axes=flange_m.n_axes,
        max_relative_rotation_deg=flange_m.max_rotation_deg,
        max_relative_translation_mm=flange_m.max_translation_mm,
        max_axis_separation_deg=flange_m.max_axis_separation_deg,
        n_relative_axes_camera=camera_m.n_axes,
        max_relative_rotation_camera_deg=camera_m.max_rotation_deg,
        max_relative_translation_camera_mm=camera_m.max_translation_mm,
        max_axis_separation_camera_deg=camera_m.max_axis_separation_deg,
        n_duplicates=flange_m.n_duplicates + camera_m.n_duplicates,
        svd_condition_min=svd_cond_min,
    )


# =============================================================================
# Target consistency (post-solve rigidity) — both modes via one helper
# =============================================================================
@dataclass(frozen=True)
class TargetConsistencyThresholds:
    """Post-solve rigidity thresholds (profile-overridable)."""

    max_translation_std_mm: float = 3.0
    max_rotation_spread_deg: float = 2.0

    def __post_init__(self) -> None:
        _validate_finite_thresholds(self)


def _target_consistency_for_mode(
    stations: list[Station],
    tf_handeye: np.ndarray,
    *,
    invariant_frame: Literal["base_target", "flange_target"],
) -> tuple[np.ndarray, list[float], list[float]]:
    """Compute the per-station invariant transform + residuals for one mode.

    ``base_target`` (eye-in-hand): ``T_base_target = T_base_flange @ X @ T_cam_target``.
    ``flange_target`` (eye-to-hand): ``T_flange_target = inv(T_base_flange) @ X @ T_cam_target``.
    Returns ``(mean_transform, trans_residuals_mm, rot_residuals_deg)``.
    """
    if len(stations) < 2:
        return np.asarray(tf_handeye, dtype=np.float64), [], []
    x = np.asarray(tf_handeye, dtype=np.float64)
    if invariant_frame == "flange_target":
        yis = [
            invert_transform(np.asarray(s.tf_base_flange)) @ x @ np.asarray(s.detection.tf_cam_target) for s in stations
        ]
    else:
        yis = [np.asarray(s.tf_base_flange) @ x @ np.asarray(s.detection.tf_cam_target) for s in stations]
    y_arr = np.stack(yis)
    trans_mean = y_arr[:, :3, 3].mean(axis=0)
    rot_mean = _rotation_from_matrix_mean(y_arr[:, :3, :3])
    y_mean = make_transform(orthonormalize(rot_mean), trans_mean)
    trans_residuals = [float(np.linalg.norm(y[:3, 3] - trans_mean)) for y in yis]
    rot_residuals = [_relative_angle_deg(invert_transform(y_mean) @ y) for y in yis]
    return y_mean, trans_residuals, rot_residuals


def target_consistency_report(
    stations: list[Station],
    tf_handeye: np.ndarray,
    *,
    invariant_frame: Literal["base_target", "flange_target"],
    thresholds: TargetConsistencyThresholds | None = None,
) -> TargetConsistencyReport:
    """Build a :class:`TargetConsistencyReport` for the given mode invariant."""
    if thresholds is None:
        thresholds = TargetConsistencyThresholds()
    y_mean, trans_res, rot_res = _target_consistency_for_mode(stations, tf_handeye, invariant_frame=invariant_frame)
    return TargetConsistencyReport(
        invariant_frame=invariant_frame,
        mean_transform=y_mean,
        translation_residual_mm=_stat(trans_res),
        rotation_residual_deg=_stat(rot_res),
    )


# =============================================================================
# evaluate_calibration_quality — assemble the unified quality report
# =============================================================================
def evaluate_calibration_quality(
    stations: list[Station],
    tf_handeye: np.ndarray,
    *,
    mode: Literal["eye_in_hand", "eye_to_hand"],
    per_view_reproj_rms_px: Sequence[float | None],
    cross_check_transforms: dict[str, np.ndarray] | None = None,
    observability_thresholds: ObservabilityThresholds | None = None,
    target_consistency_thresholds: TargetConsistencyThresholds | None = None,
) -> CalibrationQualityReport:
    """Assemble the unified :class:`CalibrationQualityReport` for one solve.

    Computes reprojection, observability, AX=XB residual, the mode-correct
    target-consistency invariant and (optionally) cross-check, returning one
    report shared by both eye-in-hand and eye-to-hand results. The caller binds
    it to an :class:`EyeInHandResult` or :class:`EyeToHandResult`.
    """
    _validate_mode(mode)
    good = [s for s in stations if s.detection.ok and s.detection.tf_cam_target is not None]
    reproj_values: list[float] = []
    reproj_invalid = len(per_view_reproj_rms_px) != len(good)
    for value in per_view_reproj_rms_px:
        if value is None:
            reproj_values.append(float("nan"))
            reproj_invalid = True
            continue
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            reproj_values.append(float("nan"))
            reproj_invalid = True
            continue
        reproj_values.append(numeric)
        if not np.isfinite(numeric) or numeric < 0.0:
            reproj_invalid = True
    reproj_list = tuple(reproj_values)
    if reproj_invalid:
        # Preserve the raw per-view values for diagnostics, but make the summary
        # fail closed for both acceptance policies.  In particular, an empty
        # list must never become the all-zero summary from ``_stat([])``.
        reproj_summary = VerifyStat(float("inf"), float("inf"), float("inf"))
    else:
        reproj_summary = _stat(list(reproj_list))
    reprojection = ReprojectionReport(
        per_view_rms_px=reproj_list,
        summary=reproj_summary,
    )
    observability = observability_report(good, mode=mode, thresholds=observability_thresholds)
    rot_stat, trans_stat = axxb_residuals(good, tf_handeye, mode=mode)
    axxb_residual = AxxbResidualReport(rotation_deg=rot_stat, translation_mm=trans_stat)
    invariant_frame: Literal["base_target", "flange_target"] = (
        "base_target" if mode == "eye_in_hand" else "flange_target"
    )
    target_consistency = target_consistency_report(
        good,
        tf_handeye,
        invariant_frame=invariant_frame,
        thresholds=target_consistency_thresholds,
    )
    cross_check: CrossCheckReport | None = None
    if cross_check_transforms:
        cc_deg = 0.0
        cc_mm = 0.0
        for xi, xj in itertools.combinations(cross_check_transforms.values(), 2):
            cc_deg = max(cc_deg, float(rotation_angle_deg(xi[:3, :3], xj[:3, :3])))
            cc_mm = max(cc_mm, float(np.linalg.norm(xi[:3, 3] - xj[:3, 3])))
        cross_check = CrossCheckReport(
            transforms=cross_check_transforms,
            max_rotation_deg=cc_deg,
            max_translation_mm=cc_mm,
        )
    return CalibrationQualityReport(
        reprojection=reprojection,
        observability=observability,
        axxb_residual=axxb_residual,
        target_consistency=target_consistency,
        cross_check=cross_check,
    )


# =============================================================================
# Acceptance policies (mode-specific; quality sub-reports hold no accept/reasons)
# =============================================================================
@dataclass(frozen=True)
class EyeInHandAcceptancePolicy:
    """Eye-in-hand acceptance: preserves Piper legacy CLI thresholds.

    Gates on reproj mean, AX=XB residual and board-origin spread — the three
    bands the Piper CLI ``_print_report`` historically judged. Does NOT gate
    on observability (the legacy eye-in-hand flow never did). The exact
    thresholds match the Piper ``_judge`` bands so the legacy report behaviour
    is preserved bit-for-bit.
    """

    reproj_good_px: float = 1.0
    reproj_warn_px: float = 2.0
    axxb_rot_good_deg: float = 0.5
    axxb_rot_warn_deg: float = 1.0
    axxb_trans_good_mm: float = 2.0
    axxb_trans_warn_mm: float = 4.0
    board_origin_spread_good_mm: float = 2.0
    board_origin_spread_warn_mm: float = 4.0

    def __post_init__(self) -> None:
        _validate_finite_thresholds(self)

    def decide(self, quality: CalibrationQualityReport) -> CalibrationDecision:
        failed: list[str] = []
        reasons: list[str] = []
        if _reprojection_invalid(quality.reprojection):
            failed.append("reproj_invalid")
            reasons.append("reprojection RMS is missing or non-finite")
        if not _all_finite(
            quality.axxb_residual.rotation_deg.mean,
            quality.axxb_residual.translation_mm.mean,
            quality.target_consistency.translation_residual_mm.std,
        ):
            failed.append("quality_nonfinite")
            reasons.append("AX=XB or target-consistency quality contains non-finite values")
        # Legacy _judge returns ✅/⚠️/❌; a ❌ on any band fails acceptance.
        reproj = quality.reprojection.summary.mean
        if reproj > self.reproj_warn_px:
            failed.append("reproj")
            reasons.append(f"reproj mean {reproj:.2f}px > {self.reproj_warn_px}px")
        rot = quality.axxb_residual.rotation_deg.mean
        if rot > self.axxb_rot_warn_deg:
            failed.append("axxb_rot")
            reasons.append(f"axxb rotation {rot:.3f}deg > {self.axxb_rot_warn_deg}deg")
        trans = quality.axxb_residual.translation_mm.mean
        if trans > self.axxb_trans_warn_mm:
            failed.append("axxb_trans")
            reasons.append(f"axxb translation {trans:.2f}mm > {self.axxb_trans_warn_mm}mm")
        spread = quality.target_consistency.translation_residual_mm.std
        if spread > self.board_origin_spread_warn_mm:
            failed.append("board_origin_spread")
            reasons.append(f"board origin spread {spread:.2f}mm > {self.board_origin_spread_warn_mm}mm")
        return CalibrationDecision(
            accept=not failed,
            failed_checks=tuple(failed),
            reasons=tuple(reasons),
        )


@dataclass(frozen=True)
class EyeToHandAcceptancePolicy:
    """Eye-to-hand acceptance: observability + target consistency.

    Requires flange + camera observability (non-degenerate excitation on both
    sides) and the ``flange_target`` rigidity invariant. The reload smoke is a
    separate caller check (it needs the adapter loader + a temporary JSON) and
    is NOT part of this pure-data policy.
    """

    observability_thresholds: ObservabilityThresholds = ObservabilityThresholds()
    target_consistency_thresholds: TargetConsistencyThresholds = TargetConsistencyThresholds()

    def decide(self, quality: CalibrationQualityReport) -> CalibrationDecision:
        failed: list[str] = []
        reasons: list[str] = []
        if _reprojection_invalid(quality.reprojection):
            failed.append("reproj_invalid")
            reasons.append("reprojection RMS is missing or non-finite")
        th = self.observability_thresholds
        obs = quality.observability
        tc = quality.target_consistency
        if not _all_finite(
            obs.max_axis_separation_deg,
            obs.max_relative_rotation_deg,
            obs.max_relative_translation_mm,
            obs.max_axis_separation_camera_deg,
            obs.max_relative_rotation_camera_deg,
            obs.max_relative_translation_camera_mm,
            tc.translation_residual_mm.std,
            tc.rotation_residual_deg.max,
        ):
            failed.append("quality_nonfinite")
            reasons.append("observability or target-consistency quality contains non-finite values")
        # Flange side gates.
        if obs.n_relative_axes < 2:
            failed.append("observability_flange_axes")
            reasons.append(f"flange: only {obs.n_relative_axes} rotation axis(es) (need >=2)")
        if obs.max_axis_separation_deg < th.min_axis_separation_deg:
            failed.append("observability_flange_sep")
            reasons.append(
                f"flange: axis separation {obs.max_axis_separation_deg:.1f}deg < {th.min_axis_separation_deg}deg"
            )
        if obs.max_relative_rotation_deg < th.min_max_rotation_deg:
            failed.append("observability_flange_rot")
            reasons.append(
                f"flange: max rotation {obs.max_relative_rotation_deg:.1f}deg < {th.min_max_rotation_deg}deg"
            )
        if obs.max_relative_translation_mm < th.min_translation_baseline_mm:
            failed.append("observability_flange_trans")
            reasons.append(
                f"flange: max translation {obs.max_relative_translation_mm:.1f}mm < {th.min_translation_baseline_mm}mm"
            )
        # Camera side gates — a fixed camera with a stationary board fails here.
        if obs.n_relative_axes_camera < 2:
            failed.append("observability_camera_axes")
            reasons.append(f"camera: only {obs.n_relative_axes_camera} axis(es) (need >=2) — degenerate")
        if obs.max_axis_separation_camera_deg < th.min_axis_separation_deg:
            failed.append("observability_camera_sep")
            reasons.append(
                f"camera: axis separation {obs.max_axis_separation_camera_deg:.1f}deg < {th.min_axis_separation_deg}deg"
            )
        if obs.max_relative_rotation_camera_deg < th.min_max_rotation_deg:
            failed.append("observability_camera_rot")
            reasons.append(
                f"camera: max rotation {obs.max_relative_rotation_camera_deg:.1f}deg < {th.min_max_rotation_deg}deg"
            )
        if obs.max_relative_translation_camera_mm < th.min_camera_translation_baseline_mm:
            failed.append("observability_camera_trans")
            reasons.append(
                f"camera: max translation {obs.max_relative_translation_camera_mm:.1f}mm "
                f"< {th.min_camera_translation_baseline_mm}mm"
            )
        if obs.n_duplicates > 0:
            failed.append("observability_duplicates")
            reasons.append(f"{obs.n_duplicates} duplicate station pair(s)")
        # Target consistency (flange_target rigidity).
        tc_th = self.target_consistency_thresholds
        if tc.translation_residual_mm.std > tc_th.max_translation_std_mm:
            failed.append("target_consistency_trans")
            reasons.append(
                f"T_flange_target translation std {tc.translation_residual_mm.std:.2f}mm "
                f"> {tc_th.max_translation_std_mm}mm"
            )
        if tc.rotation_residual_deg.max > tc_th.max_rotation_spread_deg:
            failed.append("target_consistency_rot")
            reasons.append(
                f"T_flange_target rotation spread {tc.rotation_residual_deg.max:.2f}deg "
                f"> {tc_th.max_rotation_spread_deg}deg"
            )
        return CalibrationDecision(
            accept=not failed,
            failed_checks=tuple(failed),
            reasons=tuple(reasons),
        )


__all__ = [
    "EyeInHandAcceptancePolicy",
    "EyeToHandAcceptancePolicy",
    "ObservabilityReport",
    "ObservabilityThresholds",
    "TargetConsistencyThresholds",
    "_axxb_relative_pair",
    "_relative_angle_deg",
    "_relative_translation_mm",
    "_rotation_from_matrix_mean",
    "_side_metrics",
    "axxb_residuals",
    "board_origin_in_base",
    "board_origin_points",
    "evaluate_calibration_quality",
    "observability_report",
    "outlier_mask",
    "target_consistency_report",
    "t_flange_target_consistency",
]
