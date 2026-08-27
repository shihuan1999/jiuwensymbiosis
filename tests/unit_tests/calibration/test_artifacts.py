# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""§11 acceptance cases #16, #29: formal artifact publication invariants.

* #16 a formal calibration is published only after the adapter reload smoke
  passes (a rejected quality writes a candidate, not a formal artifact).
* #29 ``Station.tf_base_flange``, ``ViewDetection.tf_cam_target`` and
  ``EyeToHandResult.tf_base_cam`` are all mm/frame SE(3) invariants.
"""

from __future__ import annotations

import numpy as np
import pytest

from jiuwensymbiosis.calibration.artifacts.artifacts import (
    is_candidate_report,
    save_candidate_report,
    save_eye_to_hand_calibration,
)
from jiuwensymbiosis.calibration.domain.models import EyeToHandResult, Station, ViewDetection
from jiuwensymbiosis.calibration.domain.quality import observability_report
from jiuwensymbiosis.calibration.domain.solver import save_calibration
from jiuwensymbiosis.utils.geometry import make_transform


class TestMmFrameInvariants:
    """#29: the three SE(3) fields are 4x4 mm transforms with consistent frames."""

    def test_station_tf_base_flange_is_se3_mm(self):
        s = Station(
            tf_base_flange=make_transform(np.eye(3), [100.0, 200.0, 300.0]),
            detection=ViewDetection(ok=True, tf_cam_target=np.eye(4)),
        )
        assert s.tf_base_flange.shape == (4, 4)
        # Translation is in mm — the third row must be the homogeneous row.
        assert np.allclose(s.tf_base_flange[3, :], [0, 0, 0, 1])

    def test_view_detection_tf_cam_target_is_se3(self):
        d = ViewDetection(ok=True, tf_cam_target=make_transform(np.eye(3), [0.0, 0.0, 600.0]))
        assert d.tf_cam_target.shape == (4, 4)
        assert np.allclose(d.tf_cam_target[3, :], [0, 0, 0, 1])

    def test_eye_to_hand_result_tf_base_cam_field(self):
        # EyeToHandResult declares tf_base_cam (not tf_flange_cam) — mm, SE(3).
        import dataclasses

        fields = {f.name for f in dataclasses.fields(EyeToHandResult)}
        assert "tf_base_cam" in fields
        assert "tf_flange_cam" not in fields


class TestFormalVsCandidatePublication:
    """#16: reload-pass → formal; reject/review → candidate (not loadable)."""

    def test_candidate_report_is_marked_not_loadable(self, tmp_path):
        # Build enough stations for a real observability report (no cv2).
        stations = [
            Station(
                tf_base_flange=make_transform(np.eye(3), [i * 50.0, 0, 500.0]),
                detection=ViewDetection(ok=True, tf_cam_target=np.eye(4), reproj_rms_px=0.1),
            )
            for i in range(6)
        ]
        obs = observability_report(stations)
        x = make_transform(np.eye(3), [300.0, 0.0, 500.0])
        k = np.eye(3) * 800
        cand = tmp_path / "cand.json"
        save_candidate_report(
            cand,
            x,
            k,
            mount="eye_to_hand",
            observability=obs,
            n_stations=6,
            reasons=["observability too low"],
            top_comment="REVIEW — not loadable",
        )
        assert is_candidate_report(cand)

    def test_formal_calibration_has_top_level_T_base_cam(self, tmp_path):
        import json

        x = make_transform(np.eye(3), [300.0, 0.0, 500.0])
        k = np.eye(3) * 800
        formal = tmp_path / "calib.json"
        save_eye_to_hand_calibration(formal, x, k, mount="eye_to_hand", n_stations=8)
        data = json.loads(formal.read_text(encoding="utf-8"))
        assert "T_base_cam" in data
        assert "matrix_4x4" in data["T_base_cam"]
        assert not is_candidate_report(formal)

    @pytest.mark.parametrize("writer", ["formal", "candidate"])
    @pytest.mark.parametrize("corruption", ["last_row", "nan", "zero"])
    def test_publication_rejects_non_se3_transform(self, tmp_path, writer, corruption):
        tf = np.eye(4)
        if corruption == "last_row":
            tf[3] = [1.0, 2.0, 3.0, 4.0]
        elif corruption == "nan":
            tf[0, 3] = np.nan
        else:
            tf[:] = 0.0

        if writer == "formal":
            call = lambda: save_eye_to_hand_calibration(  # noqa: E731
                tmp_path / "invalid.json", tf, np.eye(3), mount="eye_to_hand"
            )
        else:
            call = lambda: save_candidate_report(  # noqa: E731
                tmp_path / "invalid.json",
                tf,
                np.eye(3),
                mount="eye_to_hand",
                observability=observability_report([]),
                n_stations=0,
            )
        with pytest.raises(ValueError, match="candidate.T_base_cam|T_base_cam"):
            call()
        assert not (tmp_path / "invalid.json").exists()

    def test_formal_rejects_non_se3_flange_target(self, tmp_path):
        invalid = np.eye(4)
        invalid[3, 3] = 0.0
        with pytest.raises(ValueError, match="T_flange_target"):
            save_eye_to_hand_calibration(
                tmp_path / "invalid.json",
                np.eye(4),
                np.eye(3),
                mount="eye_to_hand",
                t_flange_target=invalid,
            )

    def test_eye_in_hand_saver_rejects_non_se3_transform(self, tmp_path):
        invalid = np.zeros((4, 4))
        with pytest.raises(ValueError, match="T_flange_cam"):
            save_calibration(tmp_path / "invalid.json", invalid, np.eye(3), np.zeros(3))
        assert not (tmp_path / "invalid.json").exists()

    def test_eye_in_hand_schema2_omits_unmeasured_object_anchor(self, tmp_path):
        """An omitted anchor stays absent instead of becoming a zero vector."""
        import json

        path = tmp_path / "eye-in-hand.json"
        save_calibration(path, np.eye(4), np.eye(3), None)
        payload = json.loads(path.read_text(encoding="utf-8"))
        assert payload["schema_version"] == 2
        assert "object" not in payload

    def test_eye_in_hand_schema2_preserves_real_object_anchor(self, tmp_path):
        """Existing callers that provide a measured anchor retain it verbatim."""
        import json

        path = tmp_path / "eye-in-hand-with-anchor.json"
        save_calibration(path, np.eye(4), np.eye(3), [10.0, -20.0, 30.12567])
        payload = json.loads(path.read_text(encoding="utf-8"))
        assert payload["object"]["xyz_base_mm"] == [10.0, -20.0, 30.1257]

    @pytest.mark.parametrize("mount", [None, "ceiling"])
    def test_candidate_rejects_invalid_mount(self, tmp_path, mount):
        with pytest.raises(ValueError, match="camera mount"):
            save_candidate_report(
                tmp_path / "candidate.json",
                np.eye(4),
                np.eye(3),
                mount=mount,
                observability=observability_report([]),
                n_stations=0,
            )

    def test_candidate_accepts_eye_in_hand_with_flange_field(self, tmp_path):
        # candidate report is mount-neutral; EIH uses T_flange_cam.
        import json

        cand = tmp_path / "eih-candidate.json"
        save_candidate_report(
            cand,
            np.eye(4),
            np.eye(3) * 800,
            mount="eye_in_hand",
            observability=observability_report([]),
            n_stations=0,
        )
        data = json.loads(cand.read_text(encoding="utf-8"))
        assert data["artifact_kind"] == "eye_in_hand_solve_report"
        assert "T_flange_cam" in data["candidate"]
        assert "T_base_cam" not in data["candidate"]
        assert is_candidate_report(cand)

    @pytest.mark.parametrize("mount", [None, "eye_in_hand", "ceiling"])
    def test_formal_rejects_non_eye_to_hand_mount(self, tmp_path, mount):
        with pytest.raises(ValueError, match="camera mount|eye-to-hand calibration artifact"):
            save_eye_to_hand_calibration(
                tmp_path / "calibration.json",
                np.eye(4),
                np.eye(3),
                mount=mount,
            )
