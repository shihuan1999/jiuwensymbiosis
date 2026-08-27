# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""Hand-eye calibration scripts sub-package.

Groups the vendor-specific calibration entry points with the board detection
helpers (``handeye_board``). The package is installed alongside
``jiuwensymbiosis`` (see ``[tool.setuptools.packages.find]``), so every module
here imports its siblings as ``scripts.calibrate.<name>`` — the same absolute
form works for the console entry points and for ``python scripts/calibrate/<x>.py``.

The body-agnostic domain/workflow API is published as
``jiuwensymbiosis.calibration``; these scripts remain JiuwenSymbiosis
integration entry points and keep the
``jiuwensymbiosis.calibration.integration.integration`` bridge for adapter composition.

* ``hand_eye_calib`` — mount-neutral collect/auto/replay application flow.
* ``eye_in_hand_calib`` / ``eye_to_hand_calib`` — mount-pinned compatibility
  facades; eye-in-hand keeps a documented fallback to ``calibrate_hand_eye``
  for its legacy wizard-only options.
"""
