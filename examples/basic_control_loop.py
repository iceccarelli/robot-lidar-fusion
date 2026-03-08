#!/usr/bin/env python3
"""Basic control loop example.

This script demonstrates how to instantiate the RobotOrchestrator and
run a short control loop using the mock hardware backend.  It is the
simplest possible entry point for understanding the system.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure the project root and robot_hw are importable.
_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))
sys.path.insert(0, str(_root / "robot_hw"))

from robot_hw.robot_config import load as load_config  # noqa: E402
from robot_hw.robot_orchestrator import RobotOrchestrator  # noqa: E402


def main() -> None:
    config = load_config()
    orchestrator = RobotOrchestrator(
        cycle_time=config.cycle_time,
        total_memory_bytes=config.total_memory_bytes,
        battery_capacity_wh=config.battery_capacity_wh,
    )
    print("Running 10 control cycles with mock hardware...")
    orchestrator.run(num_cycles=10)
    print("Done.")


if __name__ == "__main__":
    main()
