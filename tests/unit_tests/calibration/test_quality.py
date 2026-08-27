# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""§11 acceptance cases #34, #36: unified quality model + acceptance policy.

* #34 synthetic eye-in-hand data keeps T_base_target constant; synthetic
  eye-to-hand data keeps T_flange_target constant; ``invariant_frame`` is
  never mixed.
* #36 the same quality fact fed to ``EyeInHandAcceptancePolicy`` and
  ``EyeToHandAcceptancePolicy`` preserves the Piper legacy verdict while
  eye-to-hand still runs its formal-artifact gate.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from jiuwensymbiosis.calibration.domain.models import Station, VerifyStat, ViewDetection
from jiuwensymbiosis.calibration.domain.quality import (
    EyeInHandAcceptancePolicy,
    EyeToHandAcceptancePolicy,
    ObservabilityThresholds,
    TargetConsistencyThresholds,
    evaluate_calibration_quality,
)
from jiuwensymbiosis.utils.geometry import make_transform


def _synth_stations(n: int, *, tf_base_cam: np.ndarray, tf_flange_target: np.ndarray):
    """Build stations consistent with a given T_base_cam + T_flange_target.

    For eye-to-hand: tf_cam_target = inv(T_base_cam) @ T_base_flange @ T_flange_target.
    """
    rng = np.random.default_rng(42)
    stations = []
    for _ in range(n):
        t = rng.uniform([-100, -100, 300], [100, 100, 500])
        tf_bf = make_transform(np.eye(3), t)
        tf_cam_target = np.linalg.inv(tf_base_cam) @ tf_bf @ tf_flange_target
        stations.append(Station(tf_base_flange=tf_bf, detection=ViewDetection(ok=True, tf_cam_target=tf_cam_target)))
    return stations


def _synth_eye_in_hand_stations(n: int = 8) -> tuple[list[Station], np.ndarray]:
    """Build exact eye-in-hand stations with a fixed target in base."""
    rng = np.random.default_rng(7)
    tf_flange_cam = make_transform(
        Rotation.from_euler("xyz", [10.0, -15.0, 20.0], degrees=True).as_matrix(), [40.0, -20.0, 80.0]
    )
    tf_base_target = make_transform(
        Rotation.from_euler("xyz", [2.0, 1.0, 5.0], degrees=True).as_matrix(), [300.0, 0.0, 30.0]
    )
    stations = []
    for _ in range(n):
        tf_base_flange = make_transform(
            Rotation.random(random_state=rng).as_matrix(), rng.uniform([-300, -300, 200], [300, 300, 700])
        )
        tf_cam_target = np.linalg.inv(tf_flange_cam) @ np.linalg.inv(tf_base_flange) @ tf_base_target
        stations.append(Station(tf_base_flange, ViewDetection(ok=True, tf_cam_target=tf_cam_target)))
    return stations, tf_flange_cam


def _synth_eth_rotation_stations(*, rotate_about_flange_origin: bool) -> tuple[list[Station], np.ndarray]:
    """Build exact ETH stations that distinguish the two A conventions.

    ``T_i=[R_i, [0,0,500]]`` has a large correct base-to-gripper translation
    baseline induced by rotating an offset flange origin.  ``T_i=[R_i,R_i p]``
    instead has zero correct baseline while the legacy EIH formula reports a
    large one.  Both sets have rich multi-axis rotation and exact target data.
    """
    tf_base_cam = make_transform(np.eye(3), [300.0, 0.0, 500.0])
    tf_flange_target = make_transform(np.eye(3), [0.0, 0.0, 150.0])
    p = np.array([0.0, 0.0, 500.0])
    eulers = ((0, 0, 0), (30, 0, 0), (0, 30, 0), (0, 0, 30), (30, 30, 0), (15, 10, 20))
    stations = []
    for euler in eulers:
        rot = Rotation.from_euler("xyz", euler, degrees=True).as_matrix()
        translation = rot @ p if rotate_about_flange_origin else p
        tf_base_flange = make_transform(rot, translation)
        tf_cam_target = np.linalg.inv(tf_base_cam) @ tf_base_flange @ tf_flange_target
        stations.append(Station(tf_base_flange, ViewDetection(ok=True, tf_cam_target=tf_cam_target)))
    return stations, tf_base_cam


class TestInvariantFrameNotMixed:
    """#34: invariant_frame matches mode and is never cross-assigned."""

    def test_eye_to_hand_uses_flange_target(self):
        tf_base_cam = np.eye(4)
        tf_base_cam[:3, 3] = [300, 0, 500]
        tf_flange_target = np.eye(4)
        tf_flange_target[:3, 3] = [0, 0, 150]
        stations = _synth_stations(8, tf_base_cam=tf_base_cam, tf_flange_target=tf_flange_target)
        q = evaluate_calibration_quality(
            stations, tf_base_cam, mode="eye_to_hand", per_view_reproj_rms_px=[0.1] * len(stations)
        )
        assert q.target_consistency.invariant_frame == "flange_target"
        # With perfectly consistent synthetic data, the flange_target residual is ~0.
        assert q.target_consistency.translation_residual_mm.max < 1.0

    def test_eye_in_hand_uses_base_target(self):
        tf_base_cam = np.eye(4)
        tf_base_cam[:3, 3] = [300, 0, 500]
        tf_flange_target = np.eye(4)
        tf_flange_target[:3, 3] = [0, 0, 150]
        stations = _synth_stations(8, tf_base_cam=tf_base_cam, tf_flange_target=tf_flange_target)
        q = evaluate_calibration_quality(
            stations, tf_base_cam, mode="eye_in_hand", per_view_reproj_rms_px=[0.1] * len(stations)
        )
        assert q.target_consistency.invariant_frame == "base_target"

    def test_eye_to_hand_axxb_uses_base_to_gripper_convention(self):
        stations, tf_base_cam = _synth_eth_rotation_stations(rotate_about_flange_origin=False)
        q = evaluate_calibration_quality(
            stations, tf_base_cam, mode="eye_to_hand", per_view_reproj_rms_px=[0.1] * len(stations)
        )
        assert q.axxb_residual.rotation_deg.max < 1e-5
        assert q.axxb_residual.translation_mm.max < 1e-5

    def test_eye_to_hand_observability_false_reject_and_false_accept_regressions(self):
        # Correct ETH A=T_i@inv(T_j) sees the large baseline induced by rotating
        # an offset flange origin, so this exact dataset should pass the gate.
        stations, tf_base_cam = _synth_eth_rotation_stations(rotate_about_flange_origin=False)
        q = evaluate_calibration_quality(
            stations, tf_base_cam, mode="eye_to_hand", per_view_reproj_rms_px=[0.1] * len(stations)
        )
        assert q.observability.max_relative_translation_mm > 30.0
        assert EyeToHandAcceptancePolicy().decide(q).accept

        # For T_i=[R_i,R_i p], the correct ETH relative translation is zero.
        # The old EIH convention reported a 353mm baseline and accepted this
        # otherwise degenerate trajectory; the corrected policy must reject it.
        stations, tf_base_cam = _synth_eth_rotation_stations(rotate_about_flange_origin=True)
        q = evaluate_calibration_quality(
            stations, tf_base_cam, mode="eye_to_hand", per_view_reproj_rms_px=[0.1] * len(stations)
        )
        assert q.observability.max_relative_translation_mm < 1e-5
        decision = EyeToHandAcceptancePolicy().decide(q)
        assert not decision.accept
        assert "observability_flange_trans" in decision.failed_checks


class TestAcceptancePolicies:
    """#36: same fact → both policies decide; eye-to-hand gates formal artifact."""

    def test_policies_return_decision_with_reasons(self):
        tf_base_cam = np.eye(4)
        tf_base_cam[:3, 3] = [300, 0, 500]
        tf_flange_target = np.eye(4)
        tf_flange_target[:3, 3] = [0, 0, 150]
        stations = _synth_stations(8, tf_base_cam=tf_base_cam, tf_flange_target=tf_flange_target)
        q = evaluate_calibration_quality(
            stations, tf_base_cam, mode="eye_to_hand", per_view_reproj_rms_px=[0.1] * len(stations)
        )
        eih = EyeInHandAcceptancePolicy().decide(q)
        eth = EyeToHandAcceptancePolicy().decide(q)
        # Both return a CalibrationDecision with accept + reasons attributes.
        assert hasattr(eih, "accept")
        assert hasattr(eih, "reasons")
        assert hasattr(eth, "accept")
        assert hasattr(eth, "reasons")

    @pytest.mark.parametrize(
        "factory",
        [
            lambda: ObservabilityThresholds(min_translation_baseline_mm=np.nan),
            lambda: TargetConsistencyThresholds(max_translation_std_mm=np.inf),
            lambda: EyeInHandAcceptancePolicy(reproj_warn_px=-np.inf),
        ],
    )
    def test_nonfinite_thresholds_rejected_at_construction(self, factory):
        with pytest.raises(ValueError, match="must be finite"):
            factory()

    @pytest.mark.parametrize(
        "reprojection",
        [[], [None] * 8, [np.nan] * 8],
    )
    def test_missing_or_nan_reprojection_fails_closed(self, reprojection):
        stations, tf_flange_cam = _synth_eye_in_hand_stations()
        q = evaluate_calibration_quality(
            stations,
            tf_flange_cam,
            mode="eye_in_hand",
            per_view_reproj_rms_px=reprojection,
        )
        eih = EyeInHandAcceptancePolicy().decide(q)
        eth = EyeToHandAcceptancePolicy().decide(q)
        assert not eih.accept
        assert not eth.accept
        assert "reproj_invalid" in eih.failed_checks
        assert "reproj_invalid" in eth.failed_checks

    def test_nan_quality_measurement_fails_closed(self):
        stations, tf_base_cam = _synth_eth_rotation_stations(rotate_about_flange_origin=False)
        quality = evaluate_calibration_quality(
            stations,
            tf_base_cam,
            mode="eye_to_hand",
            per_view_reproj_rms_px=[0.1] * len(stations),
        )
        target = replace(
            quality.target_consistency,
            translation_residual_mm=VerifyStat(0.0, 0.0, float("nan")),
        )
        quality = replace(quality, target_consistency=target)

        for policy in (EyeInHandAcceptancePolicy(), EyeToHandAcceptancePolicy()):
            decision = policy.decide(quality)
            assert not decision.accept
            assert "quality_nonfinite" in decision.failed_checks
