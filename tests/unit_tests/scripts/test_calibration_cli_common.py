# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Tests for the shared hand-eye calibration CLI plumbing."""

from __future__ import annotations

import logging


def test_unified_main_clears_proxy_before_workflow(monkeypatch, tmp_path):
    import scripts.calibrate.hand_eye_calib as cli

    calls: list[str] = []
    monkeypatch.setattr(cli, "clear_proxy_env", lambda: calls.append("proxy"))
    monkeypatch.setattr(
        cli,
        "import_waypoints",
        lambda _source, _output, *, expected_mount: calls.append(f"import:{expected_mount}"),
    )

    result = cli.main(["--collect-poses", str(tmp_path / "out.npz"), "--import-poses", str(tmp_path / "in.npz")])

    assert result == 0
    assert calls == ["proxy", "import:None"]


def test_configure_logging_uses_framework_handler_once():
    import scripts.calibrate._cli_common as cli_common
    from jiuwensymbiosis.utils.logging import _OWNED_TAG

    root = logging.getLogger()
    level_before = root.level
    owned_before = [handler for handler in root.handlers if getattr(handler, _OWNED_TAG, False)]
    logger_names = (
        "hand_eye_calib",
        "scripts.calibrate.hand_eye_calib",
        "calibrate.cli_common",
        "scripts.calibrate._cli_common",
    )
    logger_state = {name: (logging.getLogger(name).level, logging.getLogger(name).propagate) for name in logger_names}
    try:
        cli_common.configure_logging(debug=True)
        count_after_first = sum(getattr(handler, _OWNED_TAG, False) for handler in root.handlers)
        cli_common.configure_logging(debug=True)
        count_after_second = sum(getattr(handler, _OWNED_TAG, False) for handler in root.handlers)

        assert count_after_second == count_after_first
        assert count_after_first >= len(owned_before)
        assert logging.getLogger("hand_eye_calib").propagate is True
        assert logging.getLogger("hand_eye_calib").level == logging.DEBUG
    finally:
        for handler in list(root.handlers):
            if getattr(handler, _OWNED_TAG, False) and handler not in owned_before:
                root.removeHandler(handler)
                handler.close()
        root.setLevel(level_before)
        for name, (level, propagate) in logger_state.items():
            logger = logging.getLogger(name)
            logger.setLevel(level)
            logger.propagate = propagate
