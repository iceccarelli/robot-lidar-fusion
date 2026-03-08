"""Tests for robot_hw.control.locomotion_controller."""

from __future__ import annotations

from robot_hw.control.locomotion_controller import LocomotionController
from robot_hw.robot_config import load as load_config


def test_locomotion_controller_creation():
    """LocomotionController should instantiate without error."""
    cfg = load_config()
    lc = LocomotionController(cfg)
    assert lc is not None


def test_compute_commands_returns_dict():
    """compute_commands should return a dict of joint commands."""
    cfg = load_config()
    lc = LocomotionController(cfg)
    result = lc.compute_commands((0.0, 0.0, 0.0))
    assert isinstance(result, dict)
