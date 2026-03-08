"""Tests for robot_hw.control.joint_synchronization."""

from __future__ import annotations


def test_joint_command_creation():
    """JointCommand should store position, velocity, and torque."""
    from robot_hw.control.joint_synchronization import JointCommand

    cmd = JointCommand(position=1.0, velocity=0.5, torque=0.1)
    assert cmd.position == 1.0
    assert cmd.velocity == 0.5
    assert cmd.torque == 0.1


def test_joint_command_defaults():
    """JointCommand should accept None for optional fields."""
    from robot_hw.control.joint_synchronization import JointCommand

    cmd = JointCommand(position=0.0)
    assert cmd.position == 0.0
