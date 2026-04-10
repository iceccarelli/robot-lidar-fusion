"""Stress-simulation entrypoint for the robot control stack.

This module exercises the repository's current orchestrator-driven stack
across environment profiles by repeatedly submitting navigation goals,
running short control-loop bursts, and summarising emitted telemetry.
It is intentionally lightweight: the purpose is reproducible integration
validation rather than full physics simulation.
"""

from __future__ import annotations

import argparse
import os
import random
import time
from collections.abc import Sequence
from typing import Any

from robot_hw.robot_config import RobotConfig, load as load_config
from robot_hw.robot_orchestrator import RobotOrchestrator


def random_goal(radius: float = 5.0) -> dict[str, float]:
    """Generate a random planar goal within a square centered at the origin."""
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    return {
        "x": random.uniform(-radius, radius),  # nosec B311
        "y": random.uniform(-radius, radius),  # nosec B311
    }


def _max_or_default(values: tuple[float, ...], default: float) -> float:
    """Return the maximum configured value or a safe default when empty."""
    return max(values) if values else default


def _telemetry_log(orchestrator: RobotOrchestrator) -> list[dict[str, Any]]:
    """Return the communication telemetry log when present and well-typed."""
    raw_log = getattr(orchestrator.communication, "_telemetry_log", [])
    if not isinstance(raw_log, list):
        return []
    telemetry: list[dict[str, Any]] = []
    for item in raw_log:
        if isinstance(item, dict):
            telemetry.append(item)
    return telemetry


def run_simulation(profile: str, cycles: int = 100) -> None:
    """Run the orchestrator for a bounded number of simulated cycles."""
    if cycles < 0:
        raise ValueError("cycles must be non-negative")

    normalized_profile = profile.strip().upper() or "GENERAL"
    previous_profile = os.environ.get("ENVIRONMENT_PROFILE")
    os.environ["ENVIRONMENT_PROFILE"] = normalized_profile

    try:
        config: RobotConfig = load_config()
        orchestrator = RobotOrchestrator(
            cycle_time=config.cycle_time_s,
            total_memory_bytes=config.total_memory_bytes,
            battery_capacity_wh=config.battery_capacity_wh,
            max_temperature=config.max_temperature_c,
            max_velocity=_max_or_default(config.max_velocity_per_joint, 1.0),
            max_torque=_max_or_default(config.max_torque_per_joint, 1.0),
        )
        orchestrator.submit_goal(random_goal())

        for _ in range(cycles):
            if random.random() < 0.2:  # nosec B311
                orchestrator.submit_goal(random_goal())
            orchestrator.run(num_cycles=1)
            time.sleep(config.cycle_time_s)

        telemetry = _telemetry_log(orchestrator)
        print(f"\nSimulation complete for profile {normalized_profile}.")
        print(f"Cycles executed: {cycles}")
        print(f"Telemetry messages sent: {len(telemetry)}")
        if telemetry:
            print("Last telemetry message:")
            print(telemetry[-1])
    finally:
        if previous_profile is None:
            os.environ.pop("ENVIRONMENT_PROFILE", None)
        else:
            os.environ["ENVIRONMENT_PROFILE"] = previous_profile


def main(argv: Sequence[str] | None = None) -> None:
    """Parse CLI arguments and execute the stress simulation."""
    parser = argparse.ArgumentParser(description="Run robot stress simulation")
    parser.add_argument(
        "--profile",
        type=str,
        default="GENERAL",
        help="Environment profile to simulate (MINING, UNDERWATER, SPACE, FORESTRY, GENERAL)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=50,
        help="Number of control cycles to run",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)
    run_simulation(args.profile, args.cycles)


if __name__ == "__main__":
    main()

