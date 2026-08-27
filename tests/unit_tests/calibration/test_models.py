# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""§11 acceptance cases #9-#13, #33-#35: model invariants.

* #9 ``Station`` / ``ViewDetection`` have a single canonical definition in
  ``calibration.models``; the CLI facades only re-export.
* #10 ``solve_eye_in_hand`` returns ``EyeInHandResult`` exposing only
  ``tf_flange_cam``.
* #11 ``solve_eye_to_hand`` returns ``EyeToHandResult`` exposing only
  ``tf_base_cam``.
* #12 the two result types cannot masquerade for each other via ``frame_field``.
* #33 both result types carry the same ``CalibrationQualityReport`` shape
  with all five sub-reports.
"""

from __future__ import annotations

import numpy as np

from jiuwensymbiosis.calibration.domain.models import (
    CalibrationQualityReport,
    EyeInHandResult,
    EyeToHandResult,
    Station,
    ViewDetection,
)
from jiuwensymbiosis.calibration.domain.quality import evaluate_calibration_quality
from jiuwensymbiosis.utils.geometry import make_transform


def _station():
    return Station(
        tf_base_flange=make_transform(np.eye(3), [0, 0, 500.0]),
        detection=ViewDetection(ok=True, tf_cam_target=np.eye(4), reproj_rms_px=0.1),
    )


class TestStationAndViewDetectionSingleDefinition:
    """#9: Station/ViewDetection are defined in calibration.models only.

    The handeye_core compatibility facade was removed; this now verifies the
    package-level re-export points at the SAME class object as the defining
    module ``calibration.models`` (single ownership, no shadow copy).
    """

    def test_station_class_object_identity(self):
        import jiuwensymbiosis.calibration as calibration
        import jiuwensymbiosis.calibration.domain.models as models

        assert Station is models.Station
        assert calibration.Station is models.Station

    def test_view_detection_class_object_identity(self):
        import jiuwensymbiosis.calibration as calibration
        import jiuwensymbiosis.calibration.domain.models as models

        assert ViewDetection is models.ViewDetection
        assert calibration.ViewDetection is models.ViewDetection

    def test_station_fields(self):
        s = _station()
        assert s.tf_base_flange.shape == (4, 4)
        assert s.detection.ok is True


class TestFrameExplicitResults:
    """#10/#11/#12: frame-explicit transform fields, no cross-masquerading."""

    def test_eye_in_hand_exposes_tf_flange_cam_only(self):
        # #10: EyeInHandResult declares tf_flange_cam (dataclass field).
        fields = set(EyeInHandResult.__dataclass_fields__)
        assert "tf_flange_cam" in fields
        # #12: it must NOT declare tf_base_cam (no alias to the other mode).
        assert "tf_base_cam" not in fields

    def test_eye_to_hand_exposes_tf_base_cam_only(self):
        # #11: EyeToHandResult declares tf_base_cam (dataclass field).
        fields = set(EyeToHandResult.__dataclass_fields__)
        assert "tf_base_cam" in fields
        # #12: it must NOT declare tf_flange_cam (no alias to the other mode).
        assert "tf_flange_cam" not in fields

    def test_no_frame_field_mutable_string(self):
        # #12: neither result carries a mutable frame_field string field.
        for cls in (EyeInHandResult, EyeToHandResult):
            fields = set(cls.__dataclass_fields__)
            assert "frame_field" not in fields, f"{cls.__name__} still has frame_field"


class TestUnifiedQualityReportShape:
    """#33: both modes carry the same CalibrationQualityReport with 5 sub-reports."""

    def _quality(self, invariant_frame):
        # Build a minimal quality report via the public evaluator.
        tf_cam = np.eye(4)
        tf_cam[:3, 3] = [300.0, 0.0, 500.0]
        tf_flange_target = np.eye(4)
        tf_flange_target[:3, 3] = [0.0, 0.0, 150.0]
        rng = np.random.default_rng(7)
        stations = []
        for _ in range(8):
            t = rng.uniform([-100, -100, 300], [100, 100, 500])
            tf_bf = make_transform(np.eye(3), t)
            tf_cam_target = np.linalg.inv(tf_cam) @ tf_bf @ tf_flange_target
            stations.append(
                Station(
                    tf_base_flange=tf_bf,
                    detection=ViewDetection(ok=True, tf_cam_target=tf_cam_target, reproj_rms_px=0.1),
                )
            )
        return evaluate_calibration_quality(
            stations,
            tf_cam,
            mode="eye_to_hand" if invariant_frame == "flange_target" else "eye_in_hand",
            per_view_reproj_rms_px=[0.1] * len(stations),
        )

    def test_both_modes_carry_five_sub_reports(self):
        for invariant in ("base_target", "flange_target"):
            q = self._quality(invariant)
            assert isinstance(q, CalibrationQualityReport)
            # All five sub-report fields exist on the report (cross_check may be
            # None when --cross-check is off, but the field is present).
            assert q.reprojection is not None
            assert q.observability is not None
            assert q.axxb_residual is not None
            assert q.target_consistency is not None
            assert hasattr(q, "cross_check")

    def test_invariant_frame_matches_mode(self):
        # #34: eye-in-hand → base_target; eye-to-hand → flange_target. Never mixed.
        q_eih = self._quality("base_target")
        assert q_eih.target_consistency.invariant_frame == "base_target"
        q_eth = self._quality("flange_target")
        assert q_eth.target_consistency.invariant_frame == "flange_target"
