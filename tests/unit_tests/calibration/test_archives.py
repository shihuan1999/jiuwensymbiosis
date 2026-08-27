# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""§11 acceptance cases #6, #17: archive reproducibility + legacy conversion.

* #6 the same waypoint archive produces the same dense trajectory.
* #17 a legacy SO-101 stations .npz can be converted to the unified format.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from jiuwensymbiosis.calibration.artifacts.archives import (
    dump_station_archive,
    dump_waypoint_archive,
    load_station_archive,
    load_waypoint_archive,
)
from jiuwensymbiosis.calibration.domain.models import Station, ViewDetection
from jiuwensymbiosis.calibration.domain.trajectory import interpolate_joint_sequence
from jiuwensymbiosis.utils.geometry import make_transform


class TestArchiveReproducibility:
    """#6: same archive -> same dense trajectory (deterministic)."""

    def test_same_archive_same_dense(self, tmp_path):
        jv = np.array([[0, 0, 0, 0, 0], [10, 20, 0, 0, 0], [20, 40, 0, 0, 0]], dtype=float)
        p = tmp_path / "wp.npz"
        dump_waypoint_archive(
            p,
            space="joint",
            joint_values=jv,
            joint_unit="deg",
            joint_order=("j1", "j2", "j3", "j4", "j5"),
            joint_periodic=(False,) * 5,
            adapter_module="adapter.so101",
            mount="eye_to_hand",
            config=None,
        )
        loaded = load_waypoint_archive(p)
        dense_a = interpolate_joint_sequence(
            loaded["joint_values"],
            unit=loaded["joint_unit"],
            order=tuple(loaded["joint_order"]),
            periodic=tuple(loaded["joint_periodic"]),
            max_step_deg=5.0,
        )
        # Reload again — must produce an identical dense path.
        loaded2 = load_waypoint_archive(p)
        dense_b = interpolate_joint_sequence(
            loaded2["joint_values"],
            unit=loaded2["joint_unit"],
            order=tuple(loaded2["joint_order"]),
            periodic=tuple(loaded2["joint_periodic"]),
            max_step_deg=5.0,
        )
        assert len(dense_a) == len(dense_b)
        for pa, pb in zip(dense_a, dense_b, strict=True):
            np.testing.assert_allclose(pa, pb)

    def test_round_trip_preserves_arrays(self, tmp_path):
        jv = np.array([[0, 0], [5, 10], [10, 20]], dtype=float)
        p = tmp_path / "wp.npz"
        dump_waypoint_archive(
            p,
            space="joint",
            joint_values=jv,
            joint_unit="deg",
            joint_order=("j1", "j2"),
            joint_periodic=(False, False),
            adapter_module="m",
            mount="eye_to_hand",
            config=None,
        )
        loaded = load_waypoint_archive(p)
        np.testing.assert_allclose(loaded["joint_values"], jv)
        assert loaded["joint_unit"] == "deg"
        assert tuple(loaded["joint_order"]) == ("j1", "j2")


class TestStationArchiveRoundTrip:
    """Station archive round-trips stations + intrinsics."""

    def test_round_trip(self, tmp_path):
        stations = [
            Station(
                tf_base_flange=make_transform(np.eye(3), [i * 10.0, 0, 500.0]),
                detection=ViewDetection(ok=True, tf_cam_target=np.eye(4), reproj_rms_px=0.1),
            )
            for i in range(3)
        ]
        k = np.array([[800.0, 0, 320], [0, 800.0, 240], [0, 0, 1]])
        p = tmp_path / "s.npz"
        dump_station_archive(p, stations, k, adapter_module="m", mount="eye_to_hand")
        s2, k2, meta = load_station_archive(p)
        assert len(s2) == len(stations)
        np.testing.assert_allclose(k2, k)
        assert meta["mount"] == "eye_to_hand"

    @staticmethod
    def _rewrite(path, mutate):
        arrays = dict(np.load(path, allow_pickle=False))
        mutate(arrays)
        np.savez(path, **arrays)

    @staticmethod
    def _stations():
        return [
            Station(
                tf_base_flange=make_transform(np.eye(3), [i * 10.0, 0, 500.0]),
                detection=ViewDetection(ok=True, tf_cam_target=np.eye(4), reproj_rms_px=0.1),
            )
            for i in range(3)
        ]

    def _archive(self, tmp_path):
        path = tmp_path / "stations.npz"
        dump_station_archive(path, self._stations(), np.eye(3), adapter_module="m", mount="eye_to_hand")
        return path

    @pytest.mark.parametrize("field", ["tf_base_flange", "tf_cam_target"])
    def test_dump_rejects_invalid_station_se3(self, tmp_path, field):
        stations = self._stations()
        invalid = np.zeros((4, 4))
        if field == "tf_base_flange":
            stations[1].tf_base_flange = invalid
        else:
            stations[1].detection.tf_cam_target = invalid
        with pytest.raises(ValueError, match=field):
            dump_station_archive(tmp_path / "invalid.npz", stations, np.eye(3), adapter_module="m", mount="eye_to_hand")

    def test_load_rejects_metadata_n_mismatch(self, tmp_path):
        path = self._archive(tmp_path)

        def mutate(arrays):
            metadata = json.loads(str(arrays["metadata"]))
            metadata["n"] = 999
            arrays["metadata"] = np.array(json.dumps(metadata))

        self._rewrite(path, mutate)
        with pytest.raises(ValueError, match=r"metadata\.n=999 != archive n=3"):
            load_station_archive(path)

    def test_load_rejects_short_reprojection_array_with_value_error(self, tmp_path):
        path = self._archive(tmp_path)
        self._rewrite(path, lambda arrays: arrays.__setitem__("reproj", np.array([0.1, 0.2])))
        with pytest.raises(ValueError, match="reproj shape"):
            load_station_archive(path)

    @pytest.mark.parametrize("field", ["flange", "board_cam"])
    def test_load_rejects_invalid_station_se3(self, tmp_path, field):
        path = self._archive(tmp_path)

        def mutate(arrays):
            values = arrays[field].copy()
            values[1, 3] = [1.0, 2.0, 3.0, 4.0]
            arrays[field] = values

        self._rewrite(path, mutate)
        with pytest.raises(ValueError, match=field):
            load_station_archive(path)


class TestWaypointArchiveValidation:
    def test_dump_rejects_invalid_cartesian_se3(self, tmp_path):
        values = np.stack([np.eye(4), np.zeros((4, 4))])
        with pytest.raises(ValueError, match=r"cartesian_tfs\[1\]"):
            dump_waypoint_archive(
                tmp_path / "invalid.npz",
                space="cartesian",
                cartesian_tfs=values,
                adapter_module="m",
                mount="eye_to_hand",
                config=None,
            )

    def test_load_rejects_waypoint_metadata_n_mismatch(self, tmp_path):
        path = tmp_path / "waypoints.npz"
        dump_waypoint_archive(
            path,
            space="cartesian",
            cartesian_tfs=np.stack([np.eye(4), np.eye(4)]),
            adapter_module="m",
            mount="eye_to_hand",
            config=None,
        )
        arrays = dict(np.load(path, allow_pickle=False))
        metadata = json.loads(str(arrays["metadata"]))
        metadata["n"] = 10
        arrays["metadata"] = np.array(json.dumps(metadata))
        np.savez(path, **arrays)
        with pytest.raises(ValueError, match=r"metadata\.n=10"):
            load_waypoint_archive(path)
