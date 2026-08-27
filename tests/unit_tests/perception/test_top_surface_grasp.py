# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Tests for the opt-in top-surface (yaw-only bounding box) grasp point.

Covers the pure geometry helpers in ``perception.vision`` and the branch wiring in
the one body that implements the batch projection seam, SO-101 (enabled →
top-surface point + grasp_rz; disabled → byte-for-byte the previous centroid result).
"""

from __future__ import annotations

import numpy as np
import pytest

from jiuwensymbiosis.adapters.so101.api import So101Api
from jiuwensymbiosis.perception.vision import (
    _erode_mask_bool,
    mask_pixels_and_depths,
    select_top_surface_grasp,
)
from jiuwensymbiosis.utils.geometry import (
    pixel_and_depth_to_camera_xyz,
    pixels_and_depths_to_camera_xyz,
)


def _tall_box_cloud() -> np.ndarray:
    """Oblique view of a tall box: full top face at z=100 + a dense front face.

    Footprint x∈[180,220], y∈[-30,30]; top face at z=100; front face at y=-30
    spanning z∈[0,100]. The mask centroid of such a view lands mid front-face;
    the top-surface box must instead put the grasp at the true top (z≈100).
    """
    xs = np.linspace(180.0, 220.0, 21)
    top_y = np.linspace(-30.0, 30.0, 31)
    tx, ty = np.meshgrid(xs, top_y)
    top = np.column_stack([tx.ravel(), ty.ravel(), np.full(tx.size, 100.0)])

    front_z = np.linspace(0.0, 100.0, 51)
    fx, fz = np.meshgrid(xs, front_z)
    front = np.column_stack([fx.ravel(), np.full(fx.size, -30.0), fz.ravel()])
    return np.vstack([top, front])


def _banana_cloud() -> np.ndarray:
    """Low elongated object along x: length≈100 (x), width≈20 (y), flat top≈50."""
    xs = np.linspace(150.0, 250.0, 101)
    ys = np.linspace(-10.0, 10.0, 11)
    zs = np.linspace(45.0, 50.0, 3)
    gx, gy, gz = np.meshgrid(xs, ys, zs)
    return np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()])


class TestSelectTopSurfaceGrasp:
    def test_tall_box_grasps_top_not_front_face(self):
        cloud = _tall_box_cloud()
        res = select_top_surface_grasp(cloud, band_mm=15.0, top_percentile=90.0, min_points=30)
        assert res is not None
        # z_top is the true top (≈100), far above the cloud's mean Z (front face drags the mean down).
        assert res["z_top"] == pytest.approx(100.0, abs=1.0)
        assert res["z_top"] > float(np.mean(cloud[:, 2])) + 20.0
        # Horizontal centre stays on the box footprint, centred in x.
        assert res["x"] == pytest.approx(200.0, abs=5.0)
        assert -30.0 <= res["y"] <= 10.0

    def test_banana_yaw_is_short_axis_and_width_matches(self):
        res = select_top_surface_grasp(_banana_cloud(), min_points=30)
        assert res is not None
        assert res["x"] == pytest.approx(200.0, abs=2.0)
        assert res["y"] == pytest.approx(0.0, abs=2.0)
        assert res["z_top"] == pytest.approx(50.0, abs=1.0)
        # Gripper closes across the short (y) axis → rz points along y (|rz|≈90°).
        assert abs(abs(res["rz"]) - 90.0) < 10.0
        # Width is the extent along that short axis (≈20 mm), not the length.
        assert res["width_mm"] == pytest.approx(20.0, abs=3.0)

    def test_depth_flyers_do_not_lift_z_top(self):
        base = _banana_cloud()  # top ≈ 50
        flyers = np.array([[200.0, 0.0, 500.0]] * 5)
        res = select_top_surface_grasp(np.vstack([base, flyers]), min_points=30)
        assert res is not None
        assert res["z_top"] < 60.0

    def test_too_few_points_returns_none(self):
        cloud = np.array([[200.0, 0.0, 50.0]] * 10)
        assert select_top_surface_grasp(cloud, min_points=30) is None

    def test_rejects_non_point_cloud_shapes(self):
        assert select_top_surface_grasp(np.zeros((0, 3)), min_points=30) is None
        assert select_top_surface_grasp(np.zeros((100, 2)), min_points=30) is None


class TestErodeMask:
    def test_erode_peels_one_ring(self):
        mask = np.zeros((5, 5), dtype=bool)
        mask[:, :] = True
        eroded = _erode_mask_bool(mask, 1)
        expected = np.zeros((5, 5), dtype=bool)
        expected[1:4, 1:4] = True
        np.testing.assert_array_equal(eroded, expected)

    def test_zero_px_is_identity(self):
        mask = np.random.default_rng(0).random((8, 8)) > 0.5
        np.testing.assert_array_equal(_erode_mask_bool(mask, 0), mask)


class TestMaskPixelsAndDepths:
    def test_valid_pixels_inside_eroded_mask(self):
        mask = np.zeros((40, 40), dtype=bool)
        mask[10:30, 10:30] = True  # 20x20 block
        depth = np.full((40, 40), 0.5, dtype=np.float32)
        us, vs, depths = mask_pixels_and_depths({"mask": mask}, depth, erode_px=2)
        # 20x20 block eroded by 2 → 16x16 = 256 valid pixels, all at depth 0.5.
        assert us.size == 256
        assert np.all(depths == pytest.approx(0.5))
        assert us.min() >= 12 and us.max() <= 27
        assert vs.min() >= 12 and vs.max() <= 27

    def test_drops_zero_and_nan_depth(self):
        mask = np.zeros((40, 40), dtype=bool)
        mask[10:30, 10:30] = True
        depth = np.full((40, 40), 0.5, dtype=np.float32)
        depth[10:30, 10:20] = 0.0  # left half invalid
        depth[15, 25] = np.nan
        us, vs, depths = mask_pixels_and_depths({"mask": mask}, depth, erode_px=0)
        assert depths.size > 0
        assert np.all(depths > 0) and np.all(np.isfinite(depths))

    def test_mask_resolution_mismatch_resampled_to_depth_grid(self):
        mask = np.zeros((10, 10), dtype=bool)
        mask[3:7, 3:7] = True
        depth = np.full((20, 20), 0.4, dtype=np.float32)
        us, vs, depths = mask_pixels_and_depths({"mask": mask}, depth, erode_px=0)
        assert us.size > 0
        # Nearest upsample of the [3,7) block into a 20-wide grid keeps it interior.
        assert us.max() < 20 and vs.max() < 20


class TestBatchProjectionMatchesSinglePixel:
    def test_batch_equals_per_pixel(self):
        k = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
        us = np.array([100.0, 320.0, 500.0])
        vs = np.array([50.0, 240.0, 400.0])
        depths = np.array([0.4, 0.5, 0.6])
        batch = pixels_and_depths_to_camera_xyz(us, vs, depths, k)
        for i in range(us.size):
            single = pixel_and_depth_to_camera_xyz((us[i], vs[i]), depths[i], k)
            np.testing.assert_allclose(batch[i], single)


# --------------------------------------------------------------------------
# SO-101 branch wiring: enabled → top-surface point + grasp_rz; disabled →
# unchanged centroid result (no grasp_rz).
# --------------------------------------------------------------------------


class _FakeLowLevel:
    def __init__(self, rgb, depth, intrinsics):
        self._rgb = rgb
        self._depth = depth
        self.intrinsics = intrinsics
        self.calibration = None
        # Identity eye-to-hand extrinsic: base frame == camera frame, so the
        # centroid path's projected z is exactly depth_mm.
        self.tf_base_cam = np.eye(4)

    def grab_frames(self):
        return (self._rgb, self._depth)


class _FakeCfg:
    minimum_floor_margin_mm = 0.0


class _FakeEnv:
    cfg = _FakeCfg()

    def __init__(self, low_level, z_min_safe=0.0):
        self.low_level = low_level
        self._z_min_safe = z_min_safe

    @property
    def z_min_safe(self):
        return self._z_min_safe


class _Stub(So101Api):
    """So101Api with a fixed synthetic point cloud behind the batch projection seam."""

    def __init__(self, env, seg_fn, cloud, **kwargs):
        super().__init__(env, **kwargs)
        self._seg_fn = seg_fn
        self._cloud = cloud

    def _project_mask_pixels_to_base_raw(self, us, vs, depths_m):
        return self._cloud


class TestTopSurfaceGraspSo101:
    @pytest.fixture(autouse=True)
    def _no_debug_dump(self, monkeypatch):
        monkeypatch.setattr("jiuwensymbiosis.adapters.so101.api.dump_grasp_debug", lambda **_kwargs: None)

    def _build(self, *, enabled, cloud):
        from tests.mocks.mock_detector import make_mock_seg_fn

        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), 0.5, dtype=np.float32)
        intrinsics = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
        env = _FakeEnv(_FakeLowLevel(rgb, depth, intrinsics))
        return _Stub(env, make_mock_seg_fn(score=0.8), cloud, grasp_top_surface_enabled=enabled)

    def test_enabled_uses_cloud_top_surface_and_emits_yaw(self):
        api = self._build(enabled=True, cloud=_banana_cloud())
        result = api.get_grasp_info_simple("box")
        assert result["ok"] is True
        assert result["position"][0] == pytest.approx(200.0, abs=2.0)
        assert result["position"][1] == pytest.approx(0.0, abs=2.0)
        assert result["position"][2] == pytest.approx(50.0, abs=1.0)  # top face, not centroid depth
        assert result["grasp_z"] == pytest.approx(25.0, abs=1.0)  # z_top + grasp_z_offset (-25)
        assert "grasp_rz" in result and abs(abs(result["grasp_rz"]) - 90.0) < 10.0
        assert result["grasp_width_mm"] == pytest.approx(20.0, abs=3.0)

    def test_disabled_is_unchanged_centroid_result_without_yaw(self):
        api = self._build(enabled=False, cloud=_banana_cloud())
        result = api.get_grasp_info_simple("box")
        assert result["ok"] is True
        assert "grasp_rz" not in result
        assert "grasp_width_mm" not in result
        # Uniform-depth centroid path: principal-point-centred mask → x≈0, y≈0, z=depth*1000.
        assert result["position"][2] == pytest.approx(500.0, abs=1e-3)
        assert set(result) == {
            "ok",
            "object",
            "position",
            "grasp_z",
            "grasp_position",
            "place_z",
            "place_position",
            "score",
            "pixel_uv",
            "depth_m",
        }

    def test_too_few_cloud_points_falls_back_to_centroid(self):
        api = self._build(enabled=True, cloud=np.array([[200.0, 0.0, 50.0]] * 5))
        result = api.get_grasp_info_simple("box")
        assert result["ok"] is True
        assert "grasp_rz" not in result  # fell back to centroid path
        assert result["position"][2] == pytest.approx(500.0, abs=1e-3)

    def test_detection_overlay_stashed_and_popped_once(self):
        api = self._build(enabled=True, cloud=_banana_cloud())
        api.get_grasp_info_simple("box")
        overlay = api.pop_last_detection_overlay()
        assert overlay is not None and overlay.ndim == 3 and overlay.shape[2] == 3
        assert api.pop_last_detection_overlay() is None  # cleared after first pop

    def test_detection_overlay_built_in_centroid_mode_too(self):
        api = self._build(enabled=False, cloud=_banana_cloud())
        api.get_grasp_info_simple("box")
        assert api.pop_last_detection_overlay() is not None

    def test_tracking_path_does_not_build_overlay(self):
        api = self._build(enabled=True, cloud=_banana_cloud())
        api.get_grasp_tracking_sample("box")
        assert api.pop_last_detection_overlay() is None
