# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Dispatch contract for the eye-in-hand compatibility facade."""

from __future__ import annotations

import importlib

import pytest

import scripts.calibrate.eye_in_hand_calib as facade


@pytest.mark.parametrize(
    "argv",
    [
        ["--collect-poses", "waypoints.npz"],
        ["--replay", "stations.npz"],
        ["--auto", "waypoints.npz"],
        ["--auto=waypoints.npz"],
    ],
)
def test_archive_modes_use_unified_workflow(monkeypatch, argv):
    calls: list[list[str]] = []
    monkeypatch.setattr(facade, "_run_unified", lambda args: calls.append(args) or 17)
    monkeypatch.setattr(facade, "_run_legacy", lambda _args: pytest.fail("legacy CLI selected"))

    assert facade.main(argv) == 17
    assert calls == [argv]


@pytest.mark.parametrize(
    "argv",
    [
        [],
        ["--help"],
        ["--selftest"],
        ["--generate-board", "board.png"],
        ["--object-xyz", "1", "2", "3"],
        ["--base", "calibration.json"],
        ["--verify"],
        ["--yes"],
        ["--auto"],
        ["--auto", "--config", "runtime.yaml"],
        ["--auto="],
    ],
)
def test_legacy_modes_remain_on_legacy_cli(monkeypatch, argv):
    calls: list[list[str]] = []
    monkeypatch.setattr(facade, "_run_legacy", lambda args: calls.append(args) or 23)
    monkeypatch.setattr(facade, "_run_unified", lambda _args: pytest.fail("unified workflow selected"))

    assert facade.main(argv) == 23
    assert calls == [argv]


def test_unified_runner_pins_eye_in_hand_mount(monkeypatch):
    unified = importlib.import_module("scripts.calibrate.hand_eye_calib")
    calls: list[tuple[list[str], str, str]] = []

    def _fake_main(argv, *, prog, require_mount):
        calls.append((argv, prog, require_mount))
        return 0

    monkeypatch.setattr(unified, "main", _fake_main)

    assert facade._run_unified(["--replay", "stations.npz"]) == 0
    assert calls == [(["--replay", "stations.npz"], "eye_in_hand_calib.py", "eye_in_hand")]


def test_legacy_runner_forwards_to_existing_cli(monkeypatch):
    legacy = importlib.import_module("scripts.calibrate.calibrate_hand_eye")
    calls: list[list[str]] = []
    monkeypatch.setattr(legacy, "main", lambda argv: calls.append(argv) or 0)

    assert facade._run_legacy(["--selftest"]) == 0
    assert calls == [["--selftest"]]


def test_legacy_dispatch_is_explicit_to_operator(monkeypatch, caplog):
    monkeypatch.setattr(facade, "_run_legacy", lambda _args: 0)

    with caplog.at_level("WARNING", logger="eye_in_hand_calib"):
        assert facade.main(["--selftest"]) == 0

    assert "compatibility mode" in caplog.text
    assert "--replay" in facade._LEGACY_NOTICE
