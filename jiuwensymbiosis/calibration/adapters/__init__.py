# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.
"""Calibration-owned wrappers for robot adapter hardware-port protocols.

Each wrapper module is discovered by convention from its matching core
``jiuwensymbiosis.adapters.<name>`` package and exposes
``CALIBRATION_ADAPTER_SPEC``.  The package itself deliberately has no registry
or lazy exports, so importing the body-agnostic calibration domain never
starts a robot adapter or loads optional hardware SDKs.
"""
