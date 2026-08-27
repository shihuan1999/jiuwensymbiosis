# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Calibration domain: frame-explicit data contracts and pure computation.

This subpackage holds the body-agnostic data model, hardware ports, quality
measurements, the OpenCV-backed solver, trajectory interpolation and data
validators. None of these modules perform I/O or workflow orchestration; they
depend only on :mod:`jiuwensymbiosis.utils` and each other.

Import symbols from their defining module (e.g. ``from
jiuwensymbiosis.calibration.domain.models import EyeInHandResult``); this
package ``__init__`` intentionally re-exports nothing so callers stay explicit.
"""
