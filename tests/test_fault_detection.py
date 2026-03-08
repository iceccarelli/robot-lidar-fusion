"""Tests for robot_hw.core.fault_detection."""

from __future__ import annotations

from robot_hw.core.fault_detection import FaultDetector
from robot_hw.robot_config import load as load_config


def test_fault_detector_no_faults():
    """A fresh FaultDetector should report no faults."""
    cfg = load_config()
    fd = FaultDetector(cfg)
    assert fd.has_fault() is False
    assert fd.get_faults() == []


def test_fault_detector_returns_list():
    """After updating with bad data, get_faults should return a non-empty list."""
    cfg = load_config()
    fd = FaultDetector(cfg)
    fd.update({"temperature": 999.0}, {})
    assert fd.has_fault() is True
    assert isinstance(fd.get_faults(), list)
    assert len(fd.get_faults()) > 0


def test_fault_detector_clear():
    """clear_faults should reset the fault list."""
    cfg = load_config()
    fd = FaultDetector(cfg)
    fd.update({"temperature": 999.0}, {})
    assert fd.has_fault() is True
    fd.clear_faults()
    assert fd.has_fault() is False
