"""Tests for robot_hw.power.battery_management."""

from __future__ import annotations

from robot_hw.power.battery_management import BatteryManager, BatteryState


def _state(soc: float = 0.8) -> BatteryState:
    return BatteryState(
        voltage=48.0, current=5.0, temperature=30.0, soc=soc, health=1.0, timestamp=0.0
    )


def test_battery_manager_creation():
    """BatteryManager should instantiate with capacity and threshold."""
    bm = BatteryManager(100.0, 0.2)
    assert bm is not None


def test_battery_update():
    """update() should accept a BatteryState and not raise."""
    bm = BatteryManager(100.0, 0.2)
    bm.update(_state(0.95))


def test_battery_is_ok_when_healthy():
    """is_ok() should return True for a healthy battery state."""
    bm = BatteryManager(100.0, 0.2)
    bm.update(_state(0.95))
    assert bm.is_ok() is True


def test_battery_low_soc():
    """is_ok() should return False when SOC is critically low."""
    bm = BatteryManager(100.0, 0.2)
    bm.update(_state(0.02))
    assert bm.is_ok() is False
