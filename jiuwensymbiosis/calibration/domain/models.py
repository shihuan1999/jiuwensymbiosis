# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Frame-explicit calibration data types and the unified quality model.

This module is the single home of the calibration data contract:

* frame-explicit result types ``EyeInHandResult`` (``tf_flange_cam``) and
  ``EyeToHandResult`` (``tf_base_cam``) — a public type can no longer carry
  two coordinate-frame meanings behind one ``frame_field`` string;
* the shared :class:`CalibrationQualityReport` composed of reprojection,
  observability, AX=XB residual, target-consistency and cross-check
  sub-reports — both result types compose the *same* quality structure, so
  the unified quality model holds measurements, not accept/reasons;
* :class:`CalibrationDecision` carries the accept verdict and failure reasons
  produced by a mode-specific acceptance policy, keeping acceptance out of
  the measurement sub-reports;
* legacy ``Station`` / ``ViewDetection`` / ``VerifyStat`` remain available as
  the small data contracts used by the calibration workflows.

Translation unit: **mm** for every SE(3) transform. ``require_se3_mm`` is the
single validator that every transform passes before it reaches the solver or
an archive — it checks shape, finiteness, a ``[0,0,0,1]`` homogeneous row and
a proper orthogonal rotation block (det≈+1). It never guesses or rescales
units; m→mm conversion stays the adapter geometry boundary's responsibility.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Protocol, runtime_checkable

import numpy as np

if TYPE_CHECKING:
    from jiuwensymbiosis.calibration.domain.quality import ObservabilityReport


# =============================================================================
# VerifyStat — mean/max/std summary for a set of scalar errors
# =============================================================================
@dataclass(frozen=True)
class VerifyStat:
    """Mean/max/std of a set of scalar errors (deg or mm — never mixed)."""

    mean: float
    max: float
    std: float


def _stat(vals: list[float]) -> VerifyStat:
    """Build a :class:`VerifyStat` from a list of scalar errors."""
    if not vals:
        return VerifyStat(0.0, 0.0, 0.0)
    a = np.asarray(vals, dtype=np.float64)
    return VerifyStat(float(a.mean()), float(a.max()), float(a.std()))


# =============================================================================
# SE(3) validation — the single choke point for every transform entering solve/archive
# =============================================================================
def require_se3_mm(value: np.ndarray, *, field: str) -> np.ndarray:
    """Validate a 4x4 SE(3) transform whose translation is in mm.

    Checks shape ``(4, 4)``, finite values, a homogeneous last row close to
    ``[0, 0, 0, 1]`` and a rotation block that is approximately orthogonal
    with determinant close to ``+1``. No unit guess or rescale — m→mm is the
    adapter's job. Returns a float64 copy on success.
    """
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (4, 4):
        raise ValueError(f"{field}: expected (4,4), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{field}: non-finite values")
    if not np.allclose(arr[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise ValueError(f"{field}: last row must be [0,0,0,1], got {arr[3, :]}")
    rot = arr[:3, :3]
    # Proper orthogonal: R R^T ≈ I and det(R) ≈ +1 (not a reflection).
    if not np.allclose(rot @ rot.T, np.eye(3), atol=1e-6):
        raise ValueError(f"{field}: rotation block not orthogonal")
    det = float(np.linalg.det(rot))
    if not np.isclose(det, 1.0, atol=1e-6):
        raise ValueError(f"{field}: rotation determinant {det} ≠ +1 (reflection?)")
    return arr


# =============================================================================
# Per-station detection + station (robot pose coupled with a board detection)
# =============================================================================
@dataclass
class ViewDetection:
    """A single image's board detection (no robot info).

    ``tf_cam_target`` is the target-in-camera transform (maps target points to
    the camera frame); translation in mm.
    """

    ok: bool
    image_points: np.ndarray | None = None  # (N,2)
    object_points: np.ndarray | None = None  # (N,3) mm
    tf_cam_target: np.ndarray | None = None  # (4,4) target-in-camera, mm
    reproj_rms_px: float | None = None
    reason: str = ""


@dataclass
class Station:
    """Robot flange pose coupled with a board detection.

    ``tf_base_flange`` is flange-in-base (maps flange points to base);
    translation in mm. This is the only place robot and vision data couple.
    """

    tf_base_flange: np.ndarray  # (4,4) flange-in-base, mm
    detection: ViewDetection


# =============================================================================
# Unified quality model — sub-reports hold measurements, not accept/reasons
# =============================================================================
@dataclass(frozen=True)
class ReprojectionReport:
    """Per-view reprojection RMS (px) and a summary stat."""

    per_view_rms_px: tuple[float, ...]
    summary: VerifyStat


@dataclass(frozen=True)
class AxxbResidualReport:
    """AX=XB residual across station pairs (rotation deg, translation mm)."""

    rotation_deg: VerifyStat
    translation_mm: VerifyStat


@dataclass(frozen=True)
class TargetConsistencyReport:
    """Post-solve rigidity of the board-origin invariant transform.

    ``invariant_frame`` selects the rigidity invariant by mode:

    * ``"base_target"`` (eye-in-hand): ``T_base_target = T_base_flange @ T_flange_cam @ T_cam_target``
      — the board is clamped to the arm, so the board origin in base is constant
      as the arm moves;
    * ``"flange_target"`` (eye-to-hand): ``T_flange_target = inv(T_base_flange) @ T_base_cam @ T_cam_target``
      — the camera is fixed in base, so the board origin in flange is constant.

    The private numeric layer computes the right formula for the mode; this
    type records *which* invariant the numbers refer to so the two modes never
    pretend to share one rigidity semantics. ``board_origin_in_base`` in an
    eye-in-hand report maps to ``mean_transform[:3, 3]`` with
    ``invariant_frame == "base_target"``.
    """

    invariant_frame: Literal["base_target", "flange_target"]
    mean_transform: np.ndarray  # (4,4) SE(3), mm
    translation_residual_mm: VerifyStat
    rotation_residual_deg: VerifyStat


@dataclass(frozen=True)
class CrossCheckReport:
    """Multi-method cross-check: max disagreement across methods."""

    transforms: dict[str, np.ndarray] | None
    max_rotation_deg: float
    max_translation_mm: float


# ObservabilityReport / ObservabilityThresholds live in quality.py; the quality
# report references the type via a forward string to avoid an import cycle
# (quality.py imports models.py for VerifyStat/Station).
@dataclass(frozen=True)
class CalibrationQualityReport:
    """Unified quality model shared by both eye-in-hand and eye-to-hand results.

    Every sub-report holds measurements only — accept/reasons are produced by
    the mode-specific acceptance policy and live in :class:`CalibrationDecision`.
    """

    reprojection: ReprojectionReport
    observability: ObservabilityReport
    axxb_residual: AxxbResidualReport
    target_consistency: TargetConsistencyReport
    cross_check: CrossCheckReport | None


@dataclass(frozen=True)
class CalibrationDecision:
    """Acceptance verdict + failure reasons produced by an acceptance policy.

    Quality sub-reports never carry ``accept``; the policy reads the unified
    quality report and emits this decision. Artifact publication reads only
    the decision, never a second acceptance rule hidden in a sub-report.
    """

    accept: bool
    failed_checks: tuple[str, ...]
    reasons: tuple[str, ...]


# =============================================================================
# Frame-explicit result types
# =============================================================================
@dataclass(frozen=True)
class EyeInHandResult:
    """Eye-in-hand solve result: camera-in-flange (``tf_flange_cam``), mm.

    The transform field name carries the frame; there is no mutable
    ``frame_field``. Legacy quality fields are exposed as read-only properties
    mapping to ``quality.*`` so the Piper CLI report code keeps working
    through the transition without duplicate storage.
    """

    tf_flange_cam: np.ndarray  # (4,4) camera-in-flange, mm
    method: str
    n_stations: int
    intrinsics: np.ndarray
    quality: CalibrationQualityReport

    # ---- legacy compatibility properties (§4.3 mapping table) ----
    # These exist only to keep calibrate_hand_eye.py's _report_dict/_print_report
    # reading the old field names during the transition. They read from quality.*
    # so there is no duplicate storage; do NOT add new callers.
    @property
    def frame_field(self) -> str:
        return "T_flange_cam"

    @property
    def per_view_reproj_rms_px(self) -> list[float]:
        return list(self.quality.reprojection.per_view_rms_px)

    @property
    def reproj(self) -> VerifyStat:
        return self.quality.reprojection.summary

    @property
    def axxb_rot_deg(self) -> VerifyStat:
        return self.quality.axxb_residual.rotation_deg

    @property
    def axxb_trans_mm(self) -> VerifyStat:
        return self.quality.axxb_residual.translation_mm

    @property
    def board_origin_base_mm(self) -> np.ndarray:
        # invariant_frame must be base_target for eye-in-hand.
        tc = self.quality.target_consistency
        if tc.invariant_frame != "base_target":
            raise AssertionError(
                f"board_origin_base_mm requires invariant_frame='base_target', got {tc.invariant_frame!r}"
            )
        return tc.mean_transform[:3, 3].copy()

    @property
    def board_origin_spread_mm(self) -> VerifyStat:
        return self.quality.target_consistency.translation_residual_mm

    @property
    def rotation_spread_deg(self) -> float:
        # Legacy rotation_spread_deg is the flange pairwise max rotation — the
        # observability flange-side max relative rotation is the equivalent value.
        return _observability_flange_rotation(self.quality.observability)

    @property
    def cross_check(self) -> dict[str, np.ndarray] | None:
        return self.quality.cross_check.transforms if self.quality.cross_check else None

    @property
    def cross_check_max_deg(self) -> float | None:
        return self.quality.cross_check.max_rotation_deg if self.quality.cross_check else None

    @property
    def cross_check_max_mm(self) -> float | None:
        return self.quality.cross_check.max_translation_mm if self.quality.cross_check else None


@dataclass(frozen=True)
class EyeToHandResult:
    """Eye-to-hand solve result: camera-in-base (``tf_base_cam``), mm.

    No ``tf_flange_cam`` property is exposed — that name would propagate the
    wrong-frame semantics the old wrapper had (assigning ``tf_flange_cam`` to a
    base-frame matrix). Equivalent quality dimensions are reachable via
    ``quality.*``.
    """

    tf_base_cam: np.ndarray  # (4,4) camera-in-base, mm
    method: str
    n_stations: int
    intrinsics: np.ndarray
    quality: CalibrationQualityReport


def _observability_flange_rotation(observability: object) -> float:
    """Best-effort extraction of the legacy ``rotation_spread_deg`` value.

    Legacy ``rotation_spread_deg`` measured the flange pairwise max rotation.
    The unified ``ObservabilityReport`` flange-side ``max_relative_rotation_deg``
    is the equivalent value; if the report does not expose it (e.g. the eye-in-hand
    policy computed rotation_spread separately), fall back to 0.0 and let the
    policy override.
    """
    return float(getattr(observability, "max_relative_rotation_deg", 0.0) or 0.0)


# =============================================================================
# Acceptance policy protocol (mode-specific implementations live in quality.py)
# =============================================================================
@runtime_checkable
class AcceptancePolicy(Protocol):
    """Acceptance policy: quality measurements -> CalibrationDecision.

    Quality sub-reports hold only measurements; the policy decides accept/reasons
    per the mode's thresholds. Two concrete policies (eye-in-hand / eye-to-hand)
    implement this so Piper's legacy thresholds and eye-to-hand's formal gate
    can differ while the data model stays unified.
    """

    def decide(self, quality: CalibrationQualityReport) -> CalibrationDecision:
        pass


__all__ = [
    "AcceptancePolicy",
    "AxxbResidualReport",
    "CalibrationDecision",
    "CalibrationQualityReport",
    "CrossCheckReport",
    "EyeInHandResult",
    "EyeToHandResult",
    "ReprojectionReport",
    "Station",
    "TargetConsistencyReport",
    "VerifyStat",
    "_observability_flange_rotation",
    "_stat",
    "require_se3_mm",
]
