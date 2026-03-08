"""Tests for robot_hw.core.hazard_manager."""

from __future__ import annotations

from robot_hw.core.hazard_manager import HazardManager
from robot_hw.robot_config import load as load_config


def test_hazard_manager_no_hazards():
    """When no signals are present, no hazards should be flagged."""
    cfg = load_config()
    hm = HazardManager(cfg)
    hm.update({})
    hazards = hm.current_hazards()
    assert isinstance(hazards, dict)
    assert len(hazards) == 0


def test_hazard_manager_proximity_alert():
    """A close proximity reading should trigger a hazard flag."""
    cfg = load_config()
    hm = HazardManager(cfg)
    # min_safety_margin defaults to 0.1, so 0.05 is below threshold
    hm.update({"proximity": 0.05})
    hazards = hm.current_hazards()
    assert "proximity" in hazards


def test_hazard_manager_safe_distance():
    """A far proximity reading should not trigger a hazard."""
    cfg = load_config()
    hm = HazardManager(cfg)
    hm.update({"proximity": 5.0})
    hazards = hm.current_hazards()
    assert "proximity" not in hazards
