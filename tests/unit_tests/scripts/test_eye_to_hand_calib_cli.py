# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""CLI smoke tests for scripts.calibrate.eye_to_hand_calib.

Covers the exit-code contract (--help / no-args / mutually-exclusive modes /
--auto-without-config / --import-poses-without-collect-poses) and the offline
``--collect-poses --import-poses`` normalisation path (no hardware, no cv2).
The live capture + solve paths need cv2 + hardware and are exercised elsewhere.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

import jiuwensymbiosis.calibration.artifacts.archives as archives
import jiuwensymbiosis.calibration.artifacts.artifacts as artifacts
from jiuwensymbiosis.calibration.domain.models import Station, ViewDetection
from jiuwensymbiosis.calibration.domain.validation import validate_intrinsics
from jiuwensymbiosis.calibration.workflows.capture import CaptureResult
from jiuwensymbiosis.calibration.workflows.preflight import (
    PreflightError,
    validate_archive_ownership,
    validate_cartesian_trajectory,
)
from jiuwensymbiosis.calibration.workflows.profile import resolve_out_path, resolve_space

SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "calibrate" / "eye_to_hand_calib.py"
CALIB_SRC = Path(__file__).resolve().parents[3]


def _run(argv: list[str]) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH")
    env["PYTHONPATH"] = os.pathsep.join(path for path in (str(CALIB_SRC), existing_pythonpath) if path)
    return subprocess.run(
        [sys.executable, str(SCRIPT), *argv],
        capture_output=True,
        text=True,
        timeout=30,
        env=env,
    )


class TestCliExitCodes:
    def test_help_exits_0(self):
        r = _run(["--help"])
        assert r.returncode == 0
        assert "eye-to-hand" in r.stdout.lower() or "usage" in r.stdout.lower()

    def test_no_args_exits_2(self):
        r = _run([])
        assert r.returncode == 2

    def test_mutually_exclusive_modes_exit_2(self):
        r = _run(["--auto", "a.npz", "--replay", "b.npz"])
        assert r.returncode == 2

    def test_auto_without_config_exit_2(self):
        r = _run(["--auto", "a.npz"])
        assert r.returncode == 2

    def test_import_poses_without_collect_poses_exit_2(self):
        r = _run(["--import-poses", "a.npz"])
        assert r.returncode == 2


class TestImportPosesOffline:
    """``--collect-poses OUTPUT --import-poses INPUT`` normalises without hardware."""

    def test_normalises_waypoint_archive(self, tmp_path):
        src = tmp_path / "src.npz"
        jv = np.array([[0, 0, 0, 0, 0], [10, 20, 0, 0, 0], [20, 40, 0, 0, 0]], dtype=float)
        archives.dump_waypoint_archive(
            src,
            space="joint",
            joint_values=jv,
            joint_unit="deg",
            joint_order=("j1", "j2", "j3", "j4", "j5"),
            joint_periodic=(False,) * 5,
            adapter_module="jiuwensymbiosis.adapters.so101",
            mount="eye_to_hand",
            config=None,
        )
        out = tmp_path / "out.npz"
        r = _run(["--collect-poses", str(out), "--import-poses", str(src)])
        assert r.returncode == 0, r.stdout + r.stderr
        loaded = archives.load_waypoint_archive(out)
        assert loaded["space"] == "joint"
        assert loaded["joint_values"].shape == (3, 5)
        assert loaded["metadata"]["mount"] == "eye_to_hand"

    def test_rejects_station_archive_as_input(self, tmp_path):
        # --import-poses only accepts waypoint archives.
        src = tmp_path / "st.npz"
        from jiuwensymbiosis.utils.geometry import make_transform

        stations = [
            Station(
                tf_base_flange=make_transform(np.eye(3), [0, 0, 0]),
                detection=ViewDetection(ok=True, tf_cam_target=np.eye(4)),
            )
        ]
        archives.dump_station_archive(src, stations, np.eye(3), adapter_module="m", mount="eye_to_hand")
        out = tmp_path / "out.npz"
        r = _run(["--collect-poses", str(out), "--import-poses", str(src)])
        assert r.returncode != 0

    def test_alias_rejects_wrong_mount_without_config(self, tmp_path):
        src = tmp_path / "wrist.npz"
        jv = np.array([[0, 0, 0, 0, 0], [10, 20, 0, 0, 0]], dtype=float)
        archives.dump_waypoint_archive(
            src,
            space="joint",
            joint_values=jv,
            joint_unit="deg",
            joint_order=("j1", "j2", "j3", "j4", "j5"),
            joint_periodic=(False,) * 5,
            adapter_module="jiuwensymbiosis.adapters.so101",
            mount="eye_in_hand",
            config=None,
        )
        out = tmp_path / "out.npz"
        r = _run(["--collect-poses", str(out), "--import-poses", str(src)])
        assert r.returncode == 2
        assert "eye_to_hand" in (r.stdout + r.stderr)


class TestPreflightErrors:
    """Preflight failures surface a non-zero exit (not a crash with a traceback)."""

    def test_replay_nonexistent_archive_exits_nonzero(self, tmp_path):
        r = _run(["--replay", str(tmp_path / "nope.npz")])
        assert r.returncode != 0

    def test_alias_rejects_wrong_replay_mount_without_config(self, tmp_path):
        from jiuwensymbiosis.utils.geometry import make_transform

        src = tmp_path / "wrist-stations.npz"
        station = Station(
            tf_base_flange=make_transform(np.eye(3), [0, 0, 300]),
            detection=ViewDetection(ok=True, tf_cam_target=np.eye(4)),
        )
        archives.dump_station_archive(src, [station], np.eye(3), adapter_module="m", mount="eye_in_hand")
        r = _run(["--replay", str(src)])
        assert r.returncode == 2
        assert "eye_to_hand" in (r.stdout + r.stderr)


# ===========================================================================
# Stage 2: canonical adapter package + archive ownership preflight
# ===========================================================================
class TestAdapterPackage:
    """The Jiuwen integration bridge owns adapter package normalization."""

    def test_normalises_config_submodule_to_package(self):
        from jiuwensymbiosis.calibration.integration.integration import adapter_package

        assert adapter_package("jiuwensymbiosis.adapters.piper.config") == "jiuwensymbiosis.adapters.piper"
        assert adapter_package("jiuwensymbiosis.adapters.so101") == "jiuwensymbiosis.adapters.so101"

    def test_normalises_env_submodule_to_package(self):
        from jiuwensymbiosis.calibration.integration.integration import adapter_package

        assert adapter_package("jiuwensymbiosis.adapters.piper.env") == "jiuwensymbiosis.adapters.piper"

    def test_rejects_unrelated_prefix(self):
        from jiuwensymbiosis.calibration.integration.integration import adapter_package

        with pytest.raises(ValueError, match="not under"):
            adapter_package("some.other.module")
        with pytest.raises(ValueError):
            adapter_package("jiuwensymbiosis.env.base")

    def test_archive_stores_opaque_adapter_identity(self, tmp_path):
        p = tmp_path / "w.npz"
        jv = np.zeros((2, 1), dtype=float)
        archives.dump_waypoint_archive(
            p,
            space="joint",
            joint_values=jv,
            joint_unit="deg",
            joint_order=("j1",),
            joint_periodic=(False,),
            adapter_module="jiuwensymbiosis.adapters.piper.config",
            mount="eye_to_hand",
            config=None,
        )
        loaded = archives.load_waypoint_archive(p)
        assert loaded["metadata"]["adapter_module"] == "jiuwensymbiosis.adapters.piper.config"

    def test_archive_dump_keeps_synthetic_marker(self, tmp_path):
        # Non-adapter markers are stored verbatim (synthetic/test archives survive).
        p = tmp_path / "w.npz"
        jv = np.zeros((2, 1), dtype=float)
        archives.dump_waypoint_archive(
            p,
            space="joint",
            joint_values=jv,
            joint_unit="deg",
            joint_order=("j1",),
            joint_periodic=(False,),
            adapter_module="m",
            mount="eye_to_hand",
            config=None,
        )
        loaded = archives.load_waypoint_archive(p)
        assert loaded["metadata"]["adapter_module"] == "m"


class TestArchiveOwnershipPreflight:
    """The domain ownership check rejects archives from another robot."""

    @staticmethod
    def _wp(mount: str, adapter: str, space: str = "joint") -> dict:
        return {
            "space": space,
            "metadata": {"mount": mount, "adapter_module": adapter},
        }

    def _cfg(self, module: str):
        class _Cfg:
            __module__ = module  # type: ignore[assignment]

            def __init__(self):
                self.camera_mount = "eye_to_hand"

        return _Cfg()

    def test_mount_mismatch_raises_preflight(self):
        with pytest.raises(PreflightError, match="mount"):
            validate_archive_ownership(
                self._wp("eye_in_hand", "jiuwensymbiosis.adapters.so101")["metadata"],
                "jiuwensymbiosis.adapters.so101",
                "eye_to_hand",
                "joint",
                "joint",
            )

    def test_adapter_mismatch_raises_preflight(self):
        with pytest.raises(PreflightError, match="adapter"):
            validate_archive_ownership(
                self._wp("eye_to_hand", "jiuwensymbiosis.adapters.piper")["metadata"],
                "jiuwensymbiosis.adapters.so101",
                "eye_to_hand",
                "joint",
                "joint",
            )

    def test_space_mismatch_raises_preflight(self):
        with pytest.raises(PreflightError, match="space"):
            validate_archive_ownership(
                self._wp("eye_to_hand", "jiuwensymbiosis.adapters.so101")["metadata"],
                "jiuwensymbiosis.adapters.so101",
                "eye_to_hand",
                "cartesian",
                "joint",
            )

    def test_legacy_config_submodule_canonicalises_and_passes(self):
        # An archive written from a .config submodule must compare equal to the
        # package the cfg class lives in.
        validate_archive_ownership(
            self._wp("eye_to_hand", "jiuwensymbiosis.adapters.so101")["metadata"],
            "jiuwensymbiosis.adapters.so101",
            "eye_to_hand",
            "joint",
            "joint",
        )

    def test_unknown_adapter_marker_passes(self):
        # Synthetic archives (adapter_module="unknown"/"m") skip the ownership
        # check rather than blocking test/offline normalisation.
        validate_archive_ownership(
            self._wp("eye_to_hand", "unknown")["metadata"],
            "jiuwensymbiosis.adapters.so101",
            "eye_to_hand",
            "joint",
            "joint",
        )


class TestCartesianTrajectoryValidation:
    """§7.5: Cartesian mode does SE(3) data validation only — NO workspace gate.

    ``validate_cartesian_trajectory`` rejects malformed/non-finite/non-SE(3)
    inputs but MUST NOT reject a valid SE(3) for workspace/reachability reasons.
    The old ``_cartesian_workspace_gate`` (Z floor + XY bounds) is deleted.
    """

    def test_rejects_wrong_shape(self):
        with pytest.raises(PreflightError, match="must be \\(N,4,4\\)"):
            validate_cartesian_trajectory(np.zeros((2, 3, 3)))

    def test_rejects_too_few_waypoints(self):
        from jiuwensymbiosis.utils.geometry import make_transform

        tfs = np.array([make_transform(np.eye(3), [0, 0, 50.0])])
        with pytest.raises(PreflightError, match="N>=2"):
            validate_cartesian_trajectory(tfs)

    def test_rejects_non_se3(self):
        from jiuwensymbiosis.utils.geometry import make_transform

        bad = make_transform(np.eye(3), [100.0, 100.0, 100.0])
        bad[3, 3] = 2.0  # break homogeneous row
        tfs = np.array([make_transform(np.eye(3), [0, 0, 0]), bad])
        with pytest.raises(PreflightError):
            validate_cartesian_trajectory(tfs)

    def test_passes_valid_se3_outside_any_workspace(self):
        """A valid SE(3) far outside any plausible workspace MUST NOT be rejected —
        workspace/reachability is the Driver's authority, not the calibration layer's.
        """
        from jiuwensymbiosis.utils.geometry import make_transform

        # These translations are far outside any robot workspace but are valid SE(3).
        tfs = np.array(
            [
                make_transform(np.eye(3), [10000.0, 10000.0, -5000.0]),
                make_transform(np.eye(3), [10000.0, 10000.0, 5000.0]),
            ]
        )
        validate_cartesian_trajectory(tfs)  # must not raise


# ===========================================================================
# Stage 3: intrinsics fail closed + CaptureResult contract
# ===========================================================================
class TestIntrinsicsValidation:
    """``validate_intrinsics`` never falls back to np.eye(3) — it fails closed."""

    def test_none_raises(self):
        with pytest.raises(RuntimeError, match="unavailable"):
            validate_intrinsics(None, source="t")

    def test_wrong_shape_raises(self):
        with pytest.raises(RuntimeError, match="invalid"):
            validate_intrinsics(np.eye(4), source="t")

    def test_non_finite_raises(self):
        k = np.eye(3)
        k[0, 0] = np.nan
        with pytest.raises(RuntimeError, match="invalid"):
            validate_intrinsics(k, source="t")

    def test_non_positive_focal_raises(self):
        k = np.eye(3)  # fx=1, fy=1 -> ok; make fx<=0 to trigger
        k[0, 0] = 0.0
        with pytest.raises(RuntimeError, match="non-positive focal length"):
            validate_intrinsics(k, source="t")

    def test_valid_returns_copy(self):
        k = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])
        out = validate_intrinsics(k, source="t")
        np.testing.assert_allclose(out, k)
        out[0, 0] = -1.0
        assert k[0, 0] == 800.0  # caller's array untouched


class TestCaptureResultContract:
    """CaptureResult is the fixed shape solve/publish reads intrinsics from."""

    def test_dry_run_fields(self):
        r = CaptureResult(stations=[], intrinsics=None, dry_run=True)
        assert r.dry_run is True
        assert r.stations == []
        assert r.intrinsics is None
        assert r.capture_meta == []
        assert r.joint_meta is None


# ===========================================================================
# Stage 6: calibration-owned loader hook + projection/rigidity reload gate
# ===========================================================================
class TestAdapterLoaderHook:
    """Adapters expose load_calibration_artifact; the reload gate uses it."""

    def _cfg(self, module: str):
        class _Cfg:
            __module__ = module  # type: ignore[assignment]
            camera_mount = "eye_to_hand"

        return _Cfg()

    def _good_stations(self, n=6):
        import numpy as np
        from scipy.spatial.transform import Rotation

        from jiuwensymbiosis.utils.geometry import make_transform

        rng = np.random.default_rng(1)
        tf_base_cam = np.eye(4)
        tf_base_cam[:3, 3] = [300, 0, 500]
        tf_ft = np.eye(4)
        tf_ft[:3, 3] = [0, 0, 150]
        out = []
        for _ in range(n):
            R = Rotation.random(random_state=rng).as_matrix()
            t = rng.uniform([-100, -100, 300], [100, 100, 500])
            tf_bf = make_transform(R, t)
            tf_cam = np.linalg.inv(tf_base_cam) @ tf_bf @ tf_ft
            out.append(Station(tf_base_flange=tf_bf, detection=ViewDetection(ok=True, tf_cam_target=tf_cam)))
        return out, tf_base_cam

    def test_piper_hook_loads_schema2(self, tmp_path):
        """Piper publishes and reloads its wrist-camera frame (``T_flange_cam``)."""
        from jiuwensymbiosis.calibration.domain.solver import save_calibration
        from jiuwensymbiosis.calibration_schema import EYE_IN_HAND_FRAME_FIELD

        _stations, x = self._good_stations()
        p = tmp_path / "c.json"
        save_calibration(p, x, np.eye(3) * 800, None, frame_field=EYE_IN_HAND_FRAME_FIELD)
        from jiuwensymbiosis.adapters.piper import load_calibration_artifact

        calib = load_calibration_artifact(str(p), mount="eye_in_hand")
        np.testing.assert_allclose(calib[EYE_IN_HAND_FRAME_FIELD]["matrix_4x4"], x)

    def test_so101_hook_loads_schema2(self, tmp_path):
        stations, x = self._good_stations()
        p = tmp_path / "c.json"
        artifacts.save_eye_to_hand_calibration(p, x, np.eye(3) * 800, mount="eye_to_hand", n_stations=len(stations))
        from jiuwensymbiosis.adapters.so101 import load_calibration_artifact

        calib = load_calibration_artifact(str(p), mount="eye_to_hand")
        np.testing.assert_allclose(calib["T_base_cam"]["matrix_4x4"], x)

    def test_reload_smoke_passes_on_good_data(self, tmp_path):
        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )

        calibration_adapter = load_adapter_spec("jiuwensymbiosis.adapters.so101")

        stations, x = self._good_stations()
        validate_adapter_reload(
            calibration_adapter,
            tmp_path / "reload.json",
            SolvedCalibration(x, np.eye(3) * 800, "eye_to_hand"),
            stations,
        )

    def test_reload_smoke_fails_on_wrong_matrix(self, tmp_path):
        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )

        calibration_adapter = load_adapter_spec("jiuwensymbiosis.adapters.so101")

        stations, x = self._good_stations()
        bad_x = np.eye(4)  # wrong camera pose -> projection/rigidity fails
        with pytest.raises(RuntimeError, match="projection"):
            validate_adapter_reload(
                calibration_adapter,
                tmp_path / "reload.json",
                SolvedCalibration(bad_x, np.eye(3) * 800, "eye_to_hand"),
                stations,
            )

    def test_spec_without_loader_fails_closed(self, tmp_path):
        # §6 / §11#28: validate_adapter_reload calls spec.load_calibration_artifact
        # directly. A spec whose loader is None/missing must fail closed — no
        # generic central-loader fallback publishes formal.
        from dataclasses import replace

        import numpy as np

        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )

        so101_spec = load_adapter_spec("jiuwensymbiosis.adapters.so101")

        stations, x = self._good_stations()
        spec = replace(so101_spec, load_calibration_artifact=None)  # type: ignore[arg-type]
        with pytest.raises((RuntimeError, TypeError, AttributeError)):
            validate_adapter_reload(
                spec,
                tmp_path / "c.json",
                SolvedCalibration(x, np.eye(3) * 800, "eye_to_hand"),
                stations,
            )

    def test_adapter_mismatch_replay_hard_fails(self, tmp_path):
        # archive says piper, cfg is so101 -> hard ValueError (not a silent pass).

        from jiuwensymbiosis.calibration.integration.integration import adapter_package

        class _So101Cfg:
            __module__ = "jiuwensymbiosis.adapters.so101.config"
            camera_mount = "eye_to_hand"

        archive_pkg = "jiuwensymbiosis.adapters.piper"
        cfg_pkg = adapter_package(_So101Cfg())
        assert archive_pkg != cfg_pkg  # the mismatch _run_replay would reject


# ===========================================================================
# Stage 8: --n-stations even sampling, space resolution, removed flags
# ===========================================================================
class TestNCaptureStations:
    def test_even_indices_include_first_last(self):
        from jiuwensymbiosis.calibration.workflows.execute import _capture_indices

        idx = _capture_indices(100, 20)
        assert idx[0] == 0
        assert idx[-1] == 99
        assert len(idx) == 20

    def test_short_dense_captures_all(self):
        from jiuwensymbiosis.calibration.workflows.execute import _capture_indices

        # fewer dense points than requested -> capture all, no truncation to zero.
        idx = _capture_indices(5, 20)
        assert idx == [0, 1, 2, 3, 4]

    def test_n_stations_below_minimum_rejected(self, tmp_path):
        # --n-stations < MIN_STATIONS is an argparse error (exit 2).
        r = _run(["--auto", "x.npz", "--config", "x.yaml", "--n-stations", "2", "--confirm-estop"])
        assert r.returncode != 0


class TestSpaceResolution:
    def test_space_no_inference_raises(self):
        # A profile without trajectory.space must NOT fall back to joint_limits guessing.
        with pytest.raises(ValueError, match="trajectory.space"):
            resolve_space({})

    def test_space_from_profile(self):
        assert resolve_space({"trajectory": {"space": "cartesian"}}) == "cartesian"
        assert resolve_space({"trajectory": {"space": "joint"}}) == "joint"


class TestOutPathResolution:
    def test_cli_out_wins(self, tmp_path):
        args = _ns(out=str(tmp_path / "cli.json"), config=None)
        assert resolve_out_path(args.out, args.config) == tmp_path / "cli.json"

    def test_profile_output_fallback(self, tmp_path):
        cfg_yaml = tmp_path / "calib.yaml"
        cfg_yaml.write_text("calibration:\n  output: tmp/from_profile.json\n", encoding="utf-8")
        args = _ns(out=None, config=str(cfg_yaml))
        assert resolve_out_path(args.out, args.config) == Path("tmp/from_profile.json")


def _ns(**over):
    import argparse

    base = {
        "out": None,
        "config": None,
        "n_stations": 20,
        "auto": None,
        "collect_poses": None,
        "replay": None,
        "import_poses": None,
        "board": "charuco",
        "squares_x": 5,
        "squares_y": 7,
        "square_size_mm": 15.28,
        "marker_size_mm": 11.0,
        "max_joint_step_deg": 5.0,
        "max_cartesian_step_mm": 10.0,
        "max_cartesian_step_deg": 5.0,
        "method": "PARK",
        "cross_check": False,
        "dry_run": False,
        "confirm_estop": True,
        "debug": False,
        "min_relative_rotation_deg": None,
        "min_axis_separation_deg": None,
        "min_max_rotation_deg": None,
        "min_translation_baseline_mm": None,
        "min_camera_translation_baseline_mm": None,
        "duplicate_rotation_deg": None,
        "duplicate_translation_mm": None,
    }
    base.update(over)
    return argparse.Namespace(**base)


class TestRemovedFlags:
    def test_drop_outliers_is_unknown(self):
        r = _run(["--drop-outliers", "--replay", "x.npz"])
        assert r.returncode != 0

    def test_save_review_is_unknown(self):
        r = _run(["--save-review", "--replay", "x.npz"])
        assert r.returncode != 0


class TestExitCodeMapping:
    """A missing artifact does not mean failure (dry-run) and a candidate report
    does not mean success — the two must not share exit code 0."""

    @staticmethod
    def _outcome(*, artifact_path, candidate):
        from jiuwensymbiosis.calibration.workflows.workflows import RunOutcome

        return RunOutcome(result=None, decision=None, artifact_path=artifact_path, candidate=candidate)

    def test_dry_run_without_artifact_is_success(self):
        from scripts.calibrate.hand_eye_calib import EXIT_OK, _exit_code

        assert _exit_code(self._outcome(artifact_path=None, candidate=False)) == EXIT_OK

    def test_formal_publication_is_success(self, tmp_path):
        from scripts.calibrate.hand_eye_calib import EXIT_OK, _exit_code

        assert _exit_code(self._outcome(artifact_path=tmp_path / "out.json", candidate=False)) == EXIT_OK

    def test_candidate_report_is_review_not_success(self, tmp_path):
        from scripts.calibrate.hand_eye_calib import EXIT_OK, EXIT_REVIEW, _exit_code

        code = _exit_code(self._outcome(artifact_path=tmp_path / "out.candidate.json", candidate=True))
        assert code == EXIT_REVIEW
        assert code != EXIT_OK

    def test_candidate_without_artifact_is_review(self):
        from scripts.calibrate.hand_eye_calib import EXIT_REVIEW, _exit_code

        assert _exit_code(self._outcome(artifact_path=None, candidate=True)) == EXIT_REVIEW

    def test_review_code_does_not_collide_with_error_or_preflight(self):
        from scripts.calibrate.hand_eye_calib import EXIT_ERROR, EXIT_OK, EXIT_PREFLIGHT, EXIT_REVIEW

        assert len({EXIT_OK, EXIT_ERROR, EXIT_PREFLIGHT, EXIT_REVIEW}) == 4
