# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Calibration artifacts: archives plus formal/candidate serialization.

Waypoint/station NPZ archives (atomic write, refuse pickle), the formal
schema-2 calibration and REVIEW/candidate JSON artifacts. This layer depends
on the domain layer; it does not import workflows or the integration bridge.

Import symbols from their defining module (e.g. ``from
jiuwensymbiosis.calibration.artifacts.artifacts import save_candidate_report``);
this package ``__init__`` intentionally re-exports nothing.
"""
