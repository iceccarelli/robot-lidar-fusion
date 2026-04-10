"""Shared test fixtures for the robot-lidar-fusion test suite.

These fixtures provide pre-configured instances of the core subsystems
so that individual test modules can focus on behaviour rather than
boilerplate setup.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# Ensure the package root is importable regardless of working directory.
_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Also add robot_hw so that internal relative imports like
# ``from core.xxx import ...`` resolve correctly in tests.
_robot_hw = _project_root / "robot_hw"
if str(_robot_hw) not in sys.path:
    sys.path.insert(0, str(_robot_hw))


@pytest.fixture()
def robot_config():
    """Return a default RobotConfig built from environment defaults."""
    from robot_hw.robot_config import load

    return load()
