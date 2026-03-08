"""Tests for robot_hw.planning.navigation_manager."""

from __future__ import annotations

from robot_hw.planning.navigation_manager import NavigationManager
from robot_hw.robot_config import load as load_config


def test_navigation_manager_creation():
    """NavigationManager should instantiate without error."""
    cfg = load_config()
    nm = NavigationManager(cfg)
    assert nm is not None


def test_set_goal():
    """set_goal should accept a (x, y) tuple."""
    cfg = load_config()
    nm = NavigationManager(cfg)
    nm.set_goal((1.0, 2.0))


def test_update_returns_plan():
    """update() should return a list of waypoints."""
    cfg = load_config()
    nm = NavigationManager(cfg)
    nm.set_goal((1.0, 2.0))
    result = nm.update({"position": (0.0, 0.0)})
    assert isinstance(result, list)
