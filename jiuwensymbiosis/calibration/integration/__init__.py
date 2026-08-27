# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Calibration integration bridge: adapter discovery and runtime reload.

The deliberately small boundary connecting the body-agnostic domain/workflows
to a JiuwenSymbiosis session: ``CalibrationAdapterSpec`` discovery by name,
and the reload-smoke validator. Workflow contract preflight lives in
``calibration.workflows.preflight``. Core adapter packages do not import this
module.

Import symbols from their defining module (e.g. ``from
jiuwensymbiosis.calibration.integration.integration import load_adapter_spec``);
this package ``__init__`` intentionally re-exports nothing.
"""
