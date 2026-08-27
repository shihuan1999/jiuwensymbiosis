# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Jiuwen integration contracts around the in-tree calibration package."""

from __future__ import annotations

import importlib
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from jiuwensymbiosis.utils.geometry import make_transform


class TestReloadCallsSpecLoader:
    """Reload smoke calls the adapter spec loader directly through the bridge."""

    def test_uses_spec_loader_not_getattr(self, tmp_path):
        from jiuwensymbiosis.calibration.domain.models import Station, ViewDetection
        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )

        so101_spec = load_adapter_spec("jiuwensymbiosis.adapters.so101")

        loader_calls: list[str] = []

        def _tracking_loader(path, *, mount):
            loader_calls.append(f"{mount}:{path}")
            return so101_spec.load_calibration_artifact(path, mount=mount)

        spec = replace(so101_spec, load_calibration_artifact=_tracking_loader)
        x = np.eye(4)
        x[:3, 3] = [300.0, 0.0, 500.0]
        tf_ft = np.eye(4)
        tf_ft[:3, 3] = [0.0, 0.0, 150.0]
        stations = []
        for index in range(6):
            tf_bf = make_transform(np.eye(3), [index * 10.0, 0, 500.0])
            tf_cam_target = np.linalg.inv(x) @ tf_bf @ tf_ft
            stations.append(
                Station(
                    tf_base_flange=tf_bf,
                    detection=ViewDetection(ok=True, tf_cam_target=tf_cam_target, reproj_rms_px=0.1),
                )
            )
        validate_adapter_reload(
            spec,
            tmp_path / "reload.json",
            SolvedCalibration(x, np.eye(3) * 800, "eye_to_hand"),
            stations,
        )
        assert loader_calls

    def test_eye_in_hand_uses_flange_frame_artifact_field(self, tmp_path):
        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )

        calibration_adapter = load_adapter_spec("jiuwensymbiosis.adapters.piper")

        x = np.eye(4)
        x[:3, 3] = [15.0, -20.0, 100.0]
        validate_adapter_reload(
            calibration_adapter,
            tmp_path / "reload.json",
            SolvedCalibration(x, np.eye(3) * 800, "eye_in_hand"),
            [],
        )

    def test_eye_in_hand_reload_omits_unmeasured_object_anchor(self, tmp_path):
        import json
        from dataclasses import replace

        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )

        adapter = load_adapter_spec("jiuwensymbiosis.adapters.piper")
        seen: list[dict] = []

        def _loader(path, *, mount):
            payload = json.loads(path.read_text(encoding="utf-8"))
            seen.append(payload)
            return adapter.load_calibration_artifact(path, mount=mount)

        spec = replace(adapter, load_calibration_artifact=_loader)
        pose = np.eye(4)
        pose[:3, 3] = [15.0, -20.0, 100.0]
        validate_adapter_reload(
            spec,
            tmp_path / "reload.json",
            SolvedCalibration(pose, np.eye(3) * 800, "eye_in_hand"),
            [],
        )

        assert seen and "object" not in seen[0]

    def test_invalid_mount_fails_before_writing_reload_artifact(self, tmp_path):
        from jiuwensymbiosis.calibration.integration.integration import (
            SolvedCalibration,
            load_adapter_spec,
            validate_adapter_reload,
        )
        from jiuwensymbiosis.calibration.workflows.preflight import PreflightError

        calibration_adapter = load_adapter_spec("jiuwensymbiosis.adapters.piper")

        path = tmp_path / "reload.json"
        with pytest.raises(PreflightError, match="not a valid camera mount"):
            validate_adapter_reload(
                calibration_adapter,
                path,
                SolvedCalibration(np.eye(4), np.eye(3), "ceiling"),
                [],
            )
        assert not path.exists()


class TestAdapterSpecDiscovery:
    """The bridge discovers wrappers by the core adapter package name."""

    def test_derives_wrapper_name_without_central_registry(self, monkeypatch):
        import jiuwensymbiosis.calibration.integration.integration as integration

        spec = replace(
            integration.load_adapter_spec("jiuwensymbiosis.adapters.piper"),
            package="jiuwensymbiosis.adapters.new_arm",
        )
        requested: list[str] = []

        def _import(name: str):
            requested.append(name)
            return SimpleNamespace(CALIBRATION_ADAPTER_SPEC=spec)

        monkeypatch.setattr(integration, "import_module", _import)
        assert integration.load_adapter_spec("jiuwensymbiosis.adapters.new_arm") is spec
        assert requested == ["jiuwensymbiosis.calibration.adapters.new_arm"]

    def test_wrapper_spec_package_must_match_requested(self, monkeypatch):
        import jiuwensymbiosis.calibration.integration.integration as integration

        spec = integration.load_adapter_spec("jiuwensymbiosis.adapters.piper")
        monkeypatch.setattr(
            integration,
            "import_module",
            lambda _name: SimpleNamespace(CALIBRATION_ADAPTER_SPEC=spec),
        )
        with pytest.raises(ValueError, match=r"spec\.package|requested adapter package"):
            integration.load_adapter_spec("jiuwensymbiosis.adapters.new_arm")

    def test_missing_wrapper_is_a_clear_value_error(self, monkeypatch):
        import jiuwensymbiosis.calibration.integration.integration as integration

        def _missing(name: str):
            raise ModuleNotFoundError(f"No module named {name!r}", name=name)

        monkeypatch.setattr(integration, "import_module", _missing)
        with pytest.raises(ValueError, match="calibration adapter wrapper .*missing_arm"):
            integration.load_adapter_spec("jiuwensymbiosis.adapters.missing_arm")

    def test_wrapper_dependency_error_is_not_swallowed(self, monkeypatch):
        import jiuwensymbiosis.calibration.integration.integration as integration

        def _dependency_missing(_name: str):
            raise ModuleNotFoundError("optional vendor SDK", name="optional_vendor_sdk")

        monkeypatch.setattr(integration, "import_module", _dependency_missing)
        with pytest.raises(ModuleNotFoundError, match="optional vendor SDK") as exc_info:
            integration.load_adapter_spec("jiuwensymbiosis.adapters.new_arm")
        assert exc_info.value.name == "optional_vendor_sdk"

    def test_wrapper_must_expose_calibration_adapter_spec(self, monkeypatch):
        import jiuwensymbiosis.calibration.integration.integration as integration

        monkeypatch.setattr(
            integration,
            "import_module",
            lambda _name: SimpleNamespace(CALIBRATION_ADAPTER_SPEC=object()),
        )
        with pytest.raises(TypeError, match="CALIBRATION_ADAPTER_SPEC"):
            integration.load_adapter_spec("jiuwensymbiosis.adapters.new_arm")

    @pytest.mark.parametrize("adapter_name", ["piper", "so101"])
    def test_builtin_wrappers_expose_the_unified_spec_name(self, adapter_name):
        module = importlib.import_module(f"jiuwensymbiosis.calibration.adapters.{adapter_name}")
        from jiuwensymbiosis.calibration.integration.integration import CalibrationAdapterSpec

        assert isinstance(module.CALIBRATION_ADAPTER_SPEC, CalibrationAdapterSpec)
