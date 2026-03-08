"""Tests for robot_hw.planning.task_hardware_mapping."""

from __future__ import annotations

from robot_hw.planning.task_hardware_mapping import JointInstruction, Task


def test_task_creation():
    """Task dataclass should store id and parameters."""
    t = Task(id="t1", parameters={"x": 1.0, "y": 2.0})
    assert t.id == "t1"
    assert t.parameters["x"] == 1.0


def test_joint_instruction_creation():
    """JointInstruction should store joint_id and command."""
    ji = JointInstruction(joint_id="j1", command={"position": 0.5})
    assert ji.joint_id == "j1"
    assert ji.command["position"] == 0.5
