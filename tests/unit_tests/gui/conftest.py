# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""没有 NiceGUI 时不收集页面级用例。"""

from __future__ import annotations

import importlib.util

# 只有 layout 与 pages/ 下的视图 import nicegui([gui] extra),引擎与纯逻辑模块不碰它。
# 缺它时这些模块在导入期就抛 ModuleNotFoundError，pytest 以 collection error 中断整轮,
# 所以要在收集阶段排除,不能靠 skip。
if importlib.util.find_spec("nicegui") is None:
    collect_ignore_glob = ["test_*_view.py", "test_layout.py"]
