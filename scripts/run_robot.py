#!/usr/bin/env python3
"""Run the robot control loop.

This is the primary entry point for starting the robot control system.
It loads configuration from environment variables, instantiates the
orchestrator, and runs the control loop for a configurable number of
cycles.

Usage::

    python scripts/run_robot.py              # default 100 cycles
    python scripts/run_robot.py --cycles 500 # custom cycle count
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))
sys.path.insert(0, str(_root / "robot_hw"))

from robot_hw.robot_config import load as load_config  # noqa: E402
from robot_hw.robot_orchestrator import RobotOrchestrator  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the robot control loop")
    parser.add_argument("--cycles", type=int, default=100, help="Number of control cycles to run")
    args = parser.parse_args()

    config = load_config()
    orchestrator = RobotOrchestrator(
        cycle_time=config.cycle_time,
        total_memory_bytes=config.total_memory_bytes,
        battery_capacity_wh=config.battery_capacity_wh,
    )
    print(f"Starting robot control loop for {args.cycles} cycles at {1/config.cycle_time:.0f} Hz...")
    orchestrator.run(num_cycles=args.cycles)
    print("Control loop finished.")


if __name__ == "__main__":
    main()
