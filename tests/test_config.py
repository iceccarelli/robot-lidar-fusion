"""Tests for robot_hw.robot_config — environment variable parsing."""

from __future__ import annotations

import os
from unittest import mock

import pytest


def test_load_returns_dataclass():
    """Default load() should return a RobotConfig with sensible defaults."""
    from robot_hw.robot_config import load

    cfg = load()
    assert hasattr(cfg, "cycle_time_s")
    assert hasattr(cfg, "battery_capacity_wh")
    assert cfg.cycle_time_s > 0


def test_cycle_time_override():
    """CYCLE_TIME_S env var should override the default."""
    from robot_hw.robot_config import load

    with mock.patch.dict(os.environ, {"CYCLE_TIME_S": "0.05"}):
        cfg = load()
    assert cfg.cycle_time_s == pytest.approx(0.05)


def test_boolean_parsing():
    """Boolean env vars should accept common truthy strings."""
    from robot_hw.robot_config import _b

    assert _b("_TEST_BOOL_TRUE", default=False) is False
    with mock.patch.dict(os.environ, {"_TEST_BOOL_TRUE": "yes"}):
        assert _b("_TEST_BOOL_TRUE", default=False) is True


def test_csv_parsing():
    """CSV env vars should split into a tuple of stripped strings."""
    from robot_hw.robot_config import _csv

    result = _csv("_TEST_CSV", "a, b , c")
    assert result == ("a", "b", "c")


def test_float_bounds():
    """_f should raise ValueError when value is out of bounds."""
    from robot_hw.robot_config import _f

    with mock.patch.dict(os.environ, {"_TEST_F": "-1.0"}), pytest.raises(ValueError):
        _f("_TEST_F", default=1.0, min=0.0)
