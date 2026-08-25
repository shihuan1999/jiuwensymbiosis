# coding: utf-8
# Copyright (c) Huawei Technologies Co., Ltd. 2026. All rights reserved.

"""The api's sensing cache and ``memory.locations`` go stale together.

Two stores hold one fact: ``ExecutionMemory`` owns whether a reading is still valid,
the api cache owns the payload a grasp/place consumes. If only the memory is cleared
when the base moves, the planner sees "no location" while the acting step still reads
base-frame coordinates taken from a standpoint the body has left — and drives to them.
"""

from __future__ import annotations

from jiuwensymbiosis.agent.fast.runner import direct_executor
from jiuwensymbiosis.api.actions import APPROACH_FOR_GRASP, LOCATE_FOR_GRASP, NAVIGATE_RELATIVE, implements
from jiuwensymbiosis.api.base import BaseRobotApi
from jiuwensymbiosis.env.mock import MockArmEnv
from jiuwensymbiosis.tools.robot_control_tool import record_action


class CacheApi(BaseRobotApi):
    """Three real contracts: sense, move, and move-while-sensing."""

    @implements(LOCATE_FOR_GRASP)
    def locate_for_grasp(self, object_name: str = "box", reference: str | None = None, relation: str = "on") -> dict:
        result = {"ok": True, "object": object_name, "position": [100.0, 0.0, 50.0]}
        self.last_detection = result
        return result

    @implements(NAVIGATE_RELATIVE)
    def navigate_relative(self, dx_m: float, dy_m: float = 0.0, dyaw_rad: float = 0.0) -> dict:
        return {"ok": self.move_ok}

    @implements(APPROACH_FOR_GRASP)
    def approach_for_grasp(self, object_name: str = "box", reference: str | None = None, relation: str = "on") -> dict:
        # Mirrors the real approach loops: the move and the re-sense happen inside one
        # action, so the cache it leaves behind was measured from the NEW standpoint.
        fresh = {"ok": True, "object": object_name, "position": [200.0, 0.0, 50.0]}
        self.last_detection = fresh
        return fresh

    move_ok = True


def _sensed_api() -> CacheApi:
    api = CacheApi(MockArmEnv())
    record_action(api, api.locate_for_grasp, {"object_name": "box"}, api.locate_for_grasp("box"))
    return api


class TestBaseMotionStalesBothStores:
    def test_sensing_fills_both(self):
        api = _sensed_api()
        assert api.last_detection is not None
        assert api.memory.get("box") is not None

    def test_navigate_clears_both(self):
        api = _sensed_api()
        record_action(api, api.navigate_relative, {"dx_m": 1.0}, api.navigate_relative(1.0))
        assert api.memory.locations == {}
        assert api.last_detection is None, "stale base-frame geometry survived a base move"
        assert api.last_surface is None

    def test_a_failed_move_stales_nothing(self):
        api = _sensed_api()
        api.move_ok = False
        record_action(api, api.navigate_relative, {"dx_m": 1.0}, api.navigate_relative(1.0))
        assert api.memory.get("box") is not None
        assert api.last_detection is not None


class TestProducingActionsKeepWhatTheyMeasured:
    def test_approach_keeps_its_own_fresh_reading(self):
        api = _sensed_api()
        record_action(api, api.approach_for_grasp, {"object_name": "box"}, api.approach_for_grasp("box"))
        assert api.last_detection is not None, "the reading the action just took was thrown away"
        assert api.last_detection["position"] == [200.0, 0.0, 50.0]
        assert api.memory.get("box") is not None


class TestEveryDispatchPathAgrees:
    def test_direct_executor_stales_the_cache_too(self):
        api = _sensed_api()
        run = direct_executor(api)
        assert run("navigate_relative", {"dx_m": 1.0})["ok"] is True
        assert api.memory.locations == {}
        assert api.last_detection is None, "direct_executor left the cache the tool path clears"

    def test_direct_executor_records_a_sensing(self):
        api = CacheApi(MockArmEnv())
        run = direct_executor(api)
        run("locate_for_grasp", {"object_name": "box"})
        assert api.memory.get("box") is not None
