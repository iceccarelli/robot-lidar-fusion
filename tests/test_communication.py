"""Tests for robot_hw.core.communication."""

from __future__ import annotations

from robot_hw.core.communication import CommunicationInterface
from robot_hw.robot_config import load as load_config


def test_communication_interface_creation():
    """CommunicationInterface should instantiate without error."""
    cfg = load_config()
    ci = CommunicationInterface(cfg)
    assert ci is not None


def test_send_telemetry():
    """send_telemetry should accept a dict without raising."""
    cfg = load_config()
    ci = CommunicationInterface(cfg)
    ci.send_telemetry({"battery_soc": 0.9, "temperature": 35.0})
