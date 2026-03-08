#!/usr/bin/env python3
"""Stress test example.

This script runs the verbose simulation that injects random noise into
battery, thermal, and joint subsystems to exercise the full control
stack under load.  It demonstrates how to use the simulation module
for integration testing and digital-twin validation.
"""

from __future__ import annotations

import sys
from pathlib import Path

_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))
sys.path.insert(0, str(_root / "robot_hw"))

from robot_hw.robot_config import load as load_config  # noqa: E402
from robot_hw.simulation import VerboseRobotOrchestrator  # noqa: E402


def main() -> None:
    config = load_config()
    orchestrator = VerboseRobotOrchestrator(
        cycle_time=config.cycle_time,
        total_memory_bytes=config.total_memory_bytes,
        battery_capacity_wh=config.battery_capacity_wh,
    )
    print("Running 20-cycle stress test with verbose output...")
    orchestrator.run(num_cycles=20)
    print("Stress test complete.")


if __name__ == "__main__":
    main()
