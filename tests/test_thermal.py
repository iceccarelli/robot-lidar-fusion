"""Tests for robot_hw.power.thermal_management."""

from __future__ import annotations

from robot_hw.power.thermal_management import ThermalManager


def test_thermal_manager_creation():
    """ThermalManager should instantiate without error."""
    tm = ThermalManager(60.0, 5.0)
    assert tm is not None


def test_thermal_within_limits():
    """Normal temperatures should be within limits."""
    tm = ThermalManager(60.0, 5.0)
    tm.update({"j1": 30.0, "j2": 35.0})
    assert tm.is_within_limits() is True


def test_thermal_over_limit():
    """Extreme temperatures should trigger a warning."""
    tm = ThermalManager(60.0, 5.0)
    tm.update({"j1": 200.0, "j2": 200.0})
    assert tm.is_within_limits() is False
