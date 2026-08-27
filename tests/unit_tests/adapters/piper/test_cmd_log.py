# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""The Piper command log follows the per-run output directory.

``_attach_cmd_log_handler`` is called from the driver ctor on each connect; it
must target the current run dir (shared with grasp-debug) and re-point when a
new run begins so every run's motion lines land in that run's commands.log.
"""

from __future__ import annotations

import logging

import jiuwensymbiosis.adapters.piper.lowlevel as lowlevel
from jiuwensymbiosis.utils.logging import begin_run


class TestPiperCmdLogRedirect:
    def teardown_method(self):
        import jiuwensymbiosis.utils.logging as mod

        if lowlevel._CMD_LOG_HANDLER is not None:
            lowlevel.logger.removeHandler(lowlevel._CMD_LOG_HANDLER)
            lowlevel._CMD_LOG_HANDLER.close()
        lowlevel._CMD_LOG_HANDLER = None
        lowlevel._CMD_LOG_PATH = None
        mod._run_dir = None
        root = logging.getLogger()
        for handler in list(root.handlers):
            root.removeHandler(handler)
        root.setLevel(logging.NOTSET)

    def test_attach_targets_current_run_dir(self, tmp_path):
        run_dir = begin_run(tmp_path)
        path = lowlevel._attach_cmd_log_handler()
        assert path == run_dir / "commands.log"
        assert lowlevel._CMD_LOG_HANDLER in lowlevel.logger.handlers

    def test_attach_repoints_on_new_run(self, tmp_path):
        begin_run(tmp_path)
        first_path = lowlevel._attach_cmd_log_handler()
        first_handler = lowlevel._CMD_LOG_HANDLER

        second_run = begin_run(tmp_path)
        second_path = lowlevel._attach_cmd_log_handler()

        assert second_path == second_run / "commands.log"
        assert second_path != first_path
        assert first_handler not in lowlevel.logger.handlers  # old handler detached
        assert lowlevel._CMD_LOG_HANDLER in lowlevel.logger.handlers

    def test_attach_idempotent_within_one_run(self, tmp_path):
        begin_run(tmp_path)
        first_path = lowlevel._attach_cmd_log_handler()
        handler = lowlevel._CMD_LOG_HANDLER
        second_path = lowlevel._attach_cmd_log_handler()
        assert first_path == second_path
        assert lowlevel._CMD_LOG_HANDLER is handler  # same handler, not re-created

    def test_disabled_via_env(self, tmp_path, monkeypatch):
        monkeypatch.setenv("JIUWEN_PIPER_CMD_LOG", "0")
        begin_run(tmp_path)
        assert lowlevel._attach_cmd_log_handler() is None
