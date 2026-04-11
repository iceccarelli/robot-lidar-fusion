"""Tests for robot_hw.planning.mission_planner."""

from __future__ import annotations

from robot_hw.core.hazard_manager import HazardManager
from robot_hw.planning.mission_planner import MissionPlanner
from robot_hw.power.battery_management import BatteryManager, BatteryState
from robot_hw.power.thermal_management import ThermalManager
from robot_hw.robot_config import load as load_config


def _make_planner():
    """Create a MissionPlanner with all required dependencies."""
    cfg = load_config()
    bm = BatteryManager(cfg.battery_capacity_wh, cfg.low_battery_threshold)
    bm.update(
        BatteryState(
            voltage=48.0,
            current=1.0,
            temperature=25.0,
            soc=0.9,
            health=1.0,
            timestamp=0.0,
        )
    )
    tm = ThermalManager(cfg.max_temperature_c, cfg.cooling_hysteresis_c)
    tm.update({"cpu": 30.0})
    hm = HazardManager(cfg)
    return MissionPlanner(cfg, bm, tm, hm)


def test_mission_planner_creation():
    """MissionPlanner should instantiate with all dependencies."""
    mp = _make_planner()
    assert mp is not None


def test_add_goal():
    """add_goal should accept a dict with x and y keys."""
    mp = _make_planner()
    mp.add_goal({"x": 1.0, "y": 2.0})


def test_plan_tasks():
    """plan_tasks should return a list."""
    mp = _make_planner()
    mp.add_goal({"x": 1.0, "y": 2.0})
    tasks = mp.plan_tasks({"position": (0.0, 0.0)})
    assert isinstance(tasks, list)
