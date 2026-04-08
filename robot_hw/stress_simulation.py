"""Digital Twin Stress Simulation for the Robot Control System
==============================================================

This script exercises the final robot control stack across a range of
environments by simulating mission goals, random goal injection and
telemetry generation.  It uses the `RobotOrchestrator` class from the
main system and relies on its environment adaptation, mission
planning, communication, battery, thermal and hazard management
capabilities.  While not a full physics‑based digital twin, the
simulation demonstrates end‑to‑end interactions between modules and
provides a foundation for more sophisticated environment models.

Usage
-----
Run ``python stress_simulation.py --profile <ENVIRONMENT> --cycles N``
to simulate ``N`` cycles in the specified environment profile.  The
environment profile is one of ``MINING``, ``UNDERWATER``, ``SPACE``,
``FORESTRY`` or ``GENERAL``.  The simulation will randomly inject
additional goals and print a summary of telemetry data at the end.

Limitations
-----------
This simplified simulation does not model detailed sensor physics or
environment‑specific hazards.  For full digital twin fidelity, the
hardware interface should be extended to simulate actuator dynamics,
sensor noise and environment interactions.
"""

from __future__ import annotations

import argparse
import os
import random
import time

from robot_hw.robot_config import load as load_config
from robot_hw.robot_orchestrator import RobotOrchestrator


def random_goal(radius: float = 5.0) -> dict[str, float]:
    """Generate a random (x, y) goal within a square of given radius."""
    return {"x": random.uniform(-radius, radius), "y": random.uniform(-radius, radius)}  # nosec B311


def run_simulation(profile: str, cycles: int = 100) -> None:
    """Run the robot orchestrator in the specified environment profile.

    A new orchestrator is created with parameters derived from the
    loaded configuration.  A random initial goal is added to the
    mission planner.  At each cycle, additional goals may be queued
    randomly.  Telemetry logs are collected via the communication
    interface and summarised at the end.

    Parameters
    ----------
    profile : str
        Environment profile to simulate (e.g. ``"MINING"``).
    cycles : int
        Number of control cycles to run.
    """
    # Override environment profile in the environment for this run
    os.environ["ENVIRONMENT_PROFILE"] = profile.upper()
    # Load configuration and instantiate orchestrator with derived parameters
    config = load_config()
    orch = RobotOrchestrator(
        cycle_time=config.cycle_time_s,
        total_memory_bytes=config.total_memory_bytes,
        battery_capacity_wh=config.battery_capacity_wh,
        max_temperature=config.max_temperature_c,
        max_velocity=max(config.max_velocity_per_joint) if config.max_velocity_per_joint else 1.0,
        max_torque=max(config.max_torque_per_joint) if config.max_torque_per_joint else 1.0,
    )
    # Submit an initial goal
    orch.submit_goal(random_goal())
    # Run the orchestrator for the desired number of cycles, injecting
    # random goals to exercise mission planning and task scheduling
    for _cycle in range(cycles):
        # With a small probability, add another goal
        if random.random() < 0.2:  # nosec B311
            orch.submit_goal(random_goal())
        orch.run(num_cycles=1)
        # Introduce a small delay between cycles to simulate real time
        time.sleep(config.cycle_time_s)
    # Summarise telemetry
    telem_log = getattr(orch.communication, "_telemetry_log", [])
    print(f"\nSimulation complete for profile {profile}.")
    print(f"Cycles executed: {cycles}")
    print(f"Telemetry messages sent: {len(telem_log)}")
    # Show last telemetry entry for inspection
    if telem_log:
        print("Last telemetry message:")
        print(telem_log[-1])


def main() -> None:
    parser = argparse.ArgumentParser(description="Run digital twin stress simulation")
    parser.add_argument(
        "--profile",
        type=str,
        default="GENERAL",
        help="Environment profile to simulate (MINING, UNDERWATER, SPACE, FORESTRY, GENERAL)",
    )
    parser.add_argument("--cycles", type=int, default=50, help="Number of control cycles to run")
    args = parser.parse_args()
    run_simulation(args.profile, args.cycles)


if __name__ == "__main__":
    main()
