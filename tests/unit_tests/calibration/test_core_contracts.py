# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Calibration package, port and workflow contracts.

These tests deliberately depend only on the ``jiuwensymbiosis.calibration`` package and
its numerical dependencies.  Framework integration (Env, adapters and the
reload bridge) is tested in the host framework's test suite instead.
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from jiuwensymbiosis.utils.geometry import make_transform


def _rpy_deg_to_rot(rx: float, ry: float, rz: float) -> np.ndarray:
    """Small test-only helper; the domain package needs no framework geometry."""
    return Rotation.from_euler("xyz", [rx, ry, rz], degrees=True).as_matrix()


class TestPublicApiContract:
    """The facade keeps the promised workflow/domain symbols importable."""

    EXPECTED = frozenset(
        {
            "CalibrationCameraFrame",
            "CalibrationCaptureSource",
            "CalibrationDecision",
            "CalibrationPoseSource",
            "CalibrationQualityReport",
            "CalibrationRunOptions",
            "CartesianCalibrationMotion",
            "EyeInHandResult",
            "EyeToHandResult",
            "JointCalibrationMotion",
            "JointState",
            "ManualGuidance",
            "ManualGuidanceRecoveryError",
            "RunOutcome",
            "Station",
            "ViewDetection",
            "collect_waypoints",
            "dump_station_archive",
            "dump_waypoint_archive",
            "execute_calibration",
            "load_station_archive",
            "load_waypoint_archive",
            "replay_calibration",
            "save_calibration",
            "save_candidate_report",
            "save_eye_to_hand_calibration",
            "solve_eye_in_hand",
            "solve_eye_to_hand",
        }
    )

    def test_all_is_frozen_and_importable(self):
        import jiuwensymbiosis.calibration as calibration

        assert set(calibration.__all__) == self.EXPECTED
        for name in calibration.__all__:
            assert getattr(calibration, name) is not None


class TestPortOwnership:
    """Capture types are defined once and re-used by the capture workflow."""

    def test_capture_types_are_shared_with_domain_ports(self):
        from jiuwensymbiosis.calibration.domain.ports import CalibrationCameraFrame as PortFrame
        from jiuwensymbiosis.calibration.domain.ports import CalibrationCaptureSource as PortSource
        from jiuwensymbiosis.calibration.workflows.capture import CalibrationCameraFrame as CaptureFrame
        from jiuwensymbiosis.calibration.workflows.capture import CalibrationCaptureSource as CaptureSource

        assert PortFrame is CaptureFrame
        assert PortSource is CaptureSource


class TestCaptureContracts:
    """Capture calls the injected ports and never reaches a vendor object."""

    def test_capture_calls_protocol_not_vendor_object(self):
        from jiuwensymbiosis.calibration.domain.models import ViewDetection
        from jiuwensymbiosis.calibration.domain.ports import CalibrationCameraFrame, JointState
        from jiuwensymbiosis.calibration.domain.trajectory import interpolate_joint_sequence
        from jiuwensymbiosis.calibration.workflows.execute import capture_stations
        from jiuwensymbiosis.calibration.workflows.workflows import CalibrationRunOptions

        calls: list[str] = []

        class _Device:
            camera_mount = "eye_to_hand"
            _q = np.zeros(5)

            def get_joint_state(self):
                return JointState(
                    values=self._q.copy(),
                    unit="deg",
                    order=("j1", "j2", "j3", "j4", "j5"),
                    periodic=(False,) * 5,
                )

            def get_flange_transform_mm(self):
                return make_transform(np.eye(3), [0, 0, 500.0])

            def move_joint_vector(self, q):
                self._q = np.asarray(q, dtype=np.float64).copy()

            def wait_until_settled(self):
                return True

            def capture_calibration_frame(self):
                calls.append("capture_calibration_frame")
                return CalibrationCameraFrame(
                    rgb=np.zeros((4, 4, 3), dtype=np.uint8),
                    intrinsics=np.array([[800.0, 0, 320], [0, 800.0, 240], [0, 0, 1]]),
                    distortion=None,
                    captured_at_ns=0,
                    tf_cam_target=np.eye(4),
                    reproj_rms_px=0.1,
                )

        class _Explodes:
            def __getattr__(self, name):
                raise AssertionError(f"capture workflow accessed vendor.{name} (forbidden)")

        device = _Device()
        device.low_level = _Explodes()  # type: ignore[attr-defined]
        waypoints = {
            "space": "joint",
            "joint_values": np.array([[0, 0, 0, 0, 0], [5, 0, 0, 0, 0]], dtype=float),
            "joint_unit": "deg",
            "joint_order": ("j1", "j2", "j3", "j4", "j5"),
            "joint_periodic": (False,) * 5,
        }
        dense = interpolate_joint_sequence(
            waypoints["joint_values"],
            unit="deg",
            order=waypoints["joint_order"],
            periodic=waypoints["joint_periodic"],
            max_step_deg=5.0,
        )
        result = capture_stations(
            device,
            waypoints,
            dense,
            "joint",
            options=CalibrationRunOptions(n_stations=2),
            detect_fn=lambda _frame: ViewDetection(ok=True, tf_cam_target=np.eye(4), reproj_rms_px=0.1),
        )
        assert "capture_calibration_frame" in calls
        assert result.stations
        assert result.intrinsics is not None


class TestExecuteLifecycle:
    """The caller owns lifecycle; workflows assume an already connected device."""

    @staticmethod
    def _joint_wp(tmp_path):
        from jiuwensymbiosis.calibration.artifacts.archives import dump_waypoint_archive

        archive = tmp_path / "wp.npz"
        dump_waypoint_archive(
            archive,
            space="joint",
            joint_values=np.array([[0, 0, 0, 0, 0], [5, 0, 0, 0, 0]], dtype=float),
            joint_unit="deg",
            joint_order=("j1", "j2", "j3", "j4", "j5"),
            joint_periodic=(False,) * 5,
            adapter_module="adapter.so101",
            mount="eye_to_hand",
            config=None,
        )
        return archive

    @staticmethod
    def _options(**over):
        values = {"dry_run": False, "config": None}
        values.update(over)
        from jiuwensymbiosis.calibration.workflows.workflows import CalibrationRunOptions

        return CalibrationRunOptions(**values)

    @staticmethod
    def _fieldless_env(*, unit="deg", order=("j1", "j2", "j3", "j4", "j5"), periodic=(False,) * 5):
        from jiuwensymbiosis.calibration.domain.ports import JointState

        q = np.zeros(len(order))

        class _Device:
            camera_mount = "eye_to_hand"

            def get_flange_transform_mm(self):
                return make_transform(np.eye(3), [0.0, 0.0, 500.0])

            def get_joint_state(self):
                return JointState(values=q.copy(), unit=unit, order=order, periodic=periodic)

            def move_joint_vector(self, values):
                q[:] = np.asarray(values, dtype=np.float64)

        return _Device()

    def test_execute_does_not_probe_private_connected(self, tmp_path):
        from jiuwensymbiosis.calibration.workflows.execute import execute_calibration
        from jiuwensymbiosis.calibration.workflows.preflight import PreflightError

        with pytest.raises(PreflightError, match="CalibrationCaptureSource"):
            execute_calibration(self._fieldless_env(), self._joint_wp(tmp_path), self._options())

    def test_execute_dry_run_short_circuits_before_capture(self, tmp_path):
        from jiuwensymbiosis.calibration.workflows.execute import execute_calibration

        outcome = execute_calibration(self._fieldless_env(), self._joint_wp(tmp_path), self._options(dry_run=True))
        assert outcome.artifact_path is None
        assert outcome.candidate is False

    @pytest.mark.parametrize("camera_mount", [None, "ceiling"])
    def test_execute_requires_explicit_valid_camera_mount(self, tmp_path, camera_mount):
        from jiuwensymbiosis.calibration.workflows.execute import execute_calibration
        from jiuwensymbiosis.calibration.workflows.preflight import PreflightError

        device = self._fieldless_env()
        device.camera_mount = camera_mount
        with pytest.raises(PreflightError, match="camera_mount"):
            execute_calibration(device, self._joint_wp(tmp_path), self._options(dry_run=True))

    @pytest.mark.parametrize(
        ("kwargs", "message"),
        [
            ({"unit": "rad"}, "joint unit"),
            ({"periodic": (False, True, False, False, False)}, "periodic"),
            ({"order": ("q1", "q2", "q3", "q4", "q5")}, "joint order"),
        ],
    )
    def test_execute_rejects_joint_metadata_mismatch(self, tmp_path, kwargs, message):
        from jiuwensymbiosis.calibration.workflows.execute import execute_calibration
        from jiuwensymbiosis.calibration.workflows.preflight import PreflightError

        with pytest.raises(PreflightError, match=message):
            execute_calibration(self._fieldless_env(**kwargs), self._joint_wp(tmp_path), self._options())


class TestReplayMountAuthority:
    """Replay treats station-archive mount metadata as the authority."""

    @staticmethod
    def _station_archive(tmp_path, *, mount="eye_to_hand"):
        from jiuwensymbiosis.calibration.artifacts.archives import dump_station_archive
        from jiuwensymbiosis.calibration.domain.models import Station, ViewDetection

        t_base_cam = make_transform(_rpy_deg_to_rot(20, 5, -10), [50.0, -30.0, 200.0])
        t_flange_target = make_transform(_rpy_deg_to_rot(0, 180, 0), [0.0, 0.0, 80.0])
        poses = [
            (_rpy_deg_to_rot(0, 0, 0), [0.0, 0.0, 0.0]),
            (_rpy_deg_to_rot(15, 5, 0), [40.0, 10.0, 0.0]),
            (_rpy_deg_to_rot(0, 20, 10), [80.0, 0.0, 0.0]),
            (_rpy_deg_to_rot(10, 15, 5), [120.0, 10.0, 0.0]),
            (_rpy_deg_to_rot(5, 20, 15), [160.0, 0.0, 0.0]),
            (_rpy_deg_to_rot(20, 10, 25), [200.0, 10.0, 0.0]),
        ]
        stations = [
            Station(
                tf_base_flange=make_transform(R, t),
                detection=ViewDetection(
                    ok=True,
                    tf_cam_target=np.linalg.inv(t_base_cam) @ make_transform(R, t) @ t_flange_target,
                ),
            )
            for R, t in poses
        ]
        archive = tmp_path / "stations.npz"
        dump_station_archive(archive, stations, np.eye(3) * 800, adapter_module="m", mount=mount)
        return archive

    def test_no_config_rejects_mount_mismatch_before_solver(self, tmp_path):
        from jiuwensymbiosis.calibration.workflows.replay import replay_calibration
        from jiuwensymbiosis.calibration.workflows.workflows import CalibrationRunOptions

        archive = self._station_archive(tmp_path)
        from jiuwensymbiosis.calibration.workflows.preflight import PreflightError

        with pytest.raises(PreflightError, match="expected mount.*actual mount"):
            replay_calibration(
                archive,
                CalibrationRunOptions(out=tmp_path / "out.json"),
                mount="eye_in_hand",
            )

    def test_insufficient_stations_takes_review_path_not_solver_valueerror(self, tmp_path):
        # Mirrors solve_and_publish: a thin archive stays on REVIEW, never the solver.
        from jiuwensymbiosis.calibration.artifacts.archives import dump_station_archive
        from jiuwensymbiosis.calibration.domain.models import Station, ViewDetection
        from jiuwensymbiosis.calibration.workflows.replay import replay_calibration
        from jiuwensymbiosis.calibration.workflows.workflows import CalibrationRunOptions

        stations = [
            Station(
                tf_base_flange=make_transform(_rpy_deg_to_rot(0, 0, 0), [0.0, 0.0, 300.0]),
                detection=ViewDetection(ok=True, tf_cam_target=np.eye(4)),
            ),
            Station(
                tf_base_flange=make_transform(_rpy_deg_to_rot(10, 5, 0), [40.0, 0.0, 300.0]),
                detection=ViewDetection(ok=True, tf_cam_target=np.eye(4)),
            ),
        ]
        archive = tmp_path / "thin.npz"
        dump_station_archive(archive, stations, np.eye(3) * 800, adapter_module="m", mount="eye_to_hand")

        outcome = replay_calibration(archive, CalibrationRunOptions(out=tmp_path / "out.json"))

        assert outcome.candidate is True
        assert outcome.result is None
        assert outcome.artifact_path is None

    def test_eye_in_hand_archive_mount_authority_preserved(self, tmp_path):
        # replay is mount-neutral. The archive mount is the authority; an
        # eye-in-hand archive with a matching caller mount passes the mount
        # consistency check (and proceeds to the solver / candidate path).
        from jiuwensymbiosis.calibration.workflows.replay import replay_calibration
        from jiuwensymbiosis.calibration.workflows.workflows import CalibrationRunOptions

        archive = self._station_archive(tmp_path, mount="eye_in_hand")
        # No config: the archive is the sole authority; solver runs (cv2-gated).
        try:
            outcome = replay_calibration(archive, CalibrationRunOptions(out=tmp_path / "out.json"))
        except RuntimeError as exc:
            if "OpenCV" in str(exc):
                if os.environ.get("JIUWEN_CALIB_REQUIRE_OPENCV") == "1":
                    pytest.fail(f"strict calibration test requires OpenCV calibrateHandEye: {exc}")
                pytest.skip("OpenCV calibrateHandEye is unavailable (install the calib extra)")
            raise
        assert outcome.candidate is True
        assert json.loads(outcome.artifact_path.read_text(encoding="utf-8"))["mount"] == "eye_in_hand"

    def test_replay_config_mount_mismatch_rejected_before_solver(self, tmp_path):
        # A caller/device mount that contradicts the archive mount is rejected
        # at the consistency check, before any solver runs.
        from jiuwensymbiosis.calibration.workflows.replay import replay_calibration
        from jiuwensymbiosis.calibration.workflows.workflows import CalibrationRunOptions

        archive = self._station_archive(tmp_path, mount="eye_to_hand")
        from jiuwensymbiosis.calibration.workflows.preflight import PreflightError

        with pytest.raises(PreflightError, match="expected mount.*actual mount"):
            replay_calibration(
                archive,
                CalibrationRunOptions(out=tmp_path / "out.json", config="x.yaml"),
                mount="eye_in_hand",
            )

    def test_invalid_archive_mount_rejected_at_load(self, tmp_path):
        from jiuwensymbiosis.calibration.workflows.replay import replay_calibration

        archive = self._station_archive(tmp_path)
        data = dict(np.load(str(archive), allow_pickle=False))
        metadata = json.loads(str(data["metadata"]))
        metadata["mount"] = "/some/path.npz"
        data["metadata"] = np.array(json.dumps(metadata, ensure_ascii=False))
        np.savez(str(archive), **data)
        with pytest.raises(ValueError, match="not a valid camera mount"):
            replay_calibration(archive)

    def test_missing_archive_mount_rejected_at_load(self, tmp_path):
        from jiuwensymbiosis.calibration.workflows.replay import replay_calibration

        archive = self._station_archive(tmp_path)
        data = dict(np.load(str(archive), allow_pickle=False))
        metadata = json.loads(str(data["metadata"]))
        metadata.pop("mount", None)
        data["metadata"] = np.array(json.dumps(metadata, ensure_ascii=False))
        np.savez(str(archive), **data)
        with pytest.raises(ValueError, match="metadata.mount is required"):
            replay_calibration(archive)
