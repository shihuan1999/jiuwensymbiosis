# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Calibration workflows: collect / execute / replay orchestration.

Shared workflow types (``CalibrationRunOptions`` / ``RunOutcome`` /
``WorkflowDependencies``), the capture snapshot, the three workflow entry
points, preflight checks, publication orchestration and the YAML profile
reader. These modules compose the domain and artifact layers; they do not
import the integration bridge.

Import symbols from their defining module (e.g. ``from
jiuwensymbiosis.calibration.workflows.execute import execute_calibration``);
this package ``__init__`` intentionally re-exports nothing.
"""
