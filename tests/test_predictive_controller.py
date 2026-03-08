"""Tests for robot_hw.ai.predictive_controller."""

from __future__ import annotations

from robot_hw.ai.predictive_controller import PredictiveController
from robot_hw.robot_config import load as load_config


def test_predictive_controller_creation():
    """PredictiveController should instantiate without error."""
    cfg = load_config()
    pc = PredictiveController(cfg)
    assert pc is not None


def test_predict_orientation_returns_tuple():
    """predict_orientation() should return a 3-tuple."""
    cfg = load_config()
    pc = PredictiveController(cfg)
    result = pc.predict_orientation((0.0, 0.0, 0.0), {"gyro": (0.1, 0.0, 0.0)})
    assert isinstance(result, tuple)
    assert len(result) == 3
