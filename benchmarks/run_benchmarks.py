"""Benchmark runner for simulation-backed perception, mapping, and navigation workflows.

The benchmark runner executes deterministic repository scenarios using the live
orchestrator plus simulator adapter contracts. It emits JSON artifacts that can
be archived by CI for regression tracking and release-readiness evidence.
"""

from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from robot_hw.robot_orchestrator import RobotOrchestrator
from robot_hw.sim_adapters.gazebo_adapter import GazeboAdapter
from robot_hw.sim_adapters.isaac_sim_adapter import IsaacSimAdapter


@dataclass(frozen=True)
class BenchmarkScenario:
    name: str
    simulator: str
    goal_xy: tuple[float, float]
    cycles: int
    description: str


SCENARIOS = [
    BenchmarkScenario(
        name="gazebo_static_obstacles",
        simulator="gazebo",
        goal_xy=(1.5, 0.0),
        cycles=3,
        description="Replay-driven local navigation benchmark against static fused obstacles.",
    ),
    BenchmarkScenario(
        name="isaac_dynamic_replan",
        simulator="isaac_sim",
        goal_xy=(2.0, 0.5),
        cycles=3,
        description="Replay-driven navigation benchmark validating replanning-ready telemetry paths.",
    ),
]


def _adapter_payload(simulator: str) -> dict[str, Any]:
    if simulator == "gazebo":
        adapter = GazeboAdapter()
        return adapter.build_launch_hints()
    adapter = IsaacSimAdapter()
    return adapter.build_launch_hints()


def run_scenario(scenario: BenchmarkScenario) -> dict[str, Any]:
    orchestrator = RobotOrchestrator(
        cycle_time=0.001, total_memory_bytes=1024 * 1024, battery_capacity_wh=100.0
    )
    orchestrator.submit_goal({"x": scenario.goal_xy[0], "y": scenario.goal_xy[1]})
    orchestrator.run(num_cycles=scenario.cycles)

    navigation = orchestrator.current_state.get("navigation", {})
    artifact = {
        "scenario": asdict(scenario),
        "simulator_contract": _adapter_payload(scenario.simulator),
        "final_state": {
            "navigation": navigation,
            "map": orchestrator.current_state.get("map", {}),
            "nav2_costmap": orchestrator.current_state.get("nav2_costmap", {}),
            "global_plan": orchestrator.current_state.get("global_plan", []),
            "local_plan": orchestrator.current_state.get("local_plan", []),
        },
        "runtime_metrics": list(orchestrator.runtime_metrics_history),
        "health_events": list(orchestrator.health_events),
        "structured_logs": list(orchestrator.structured_logs),
        "telemetry_messages": list(getattr(orchestrator.communication, "_telemetry_log", [])),
    }
    return artifact


def main() -> None:
    repo_root = REPO_ROOT
    artifact_dir = repo_root / "benchmarks" / "artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)

    summary: list[dict[str, Any]] = []
    for scenario in SCENARIOS:
        artifact = run_scenario(scenario)
        output_path = artifact_dir / f"{scenario.name}.json"
        output_path.write_text(
            json.dumps(artifact, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        summary.append(
            {
                "name": scenario.name,
                "simulator": scenario.simulator,
                "cycles": scenario.cycles,
                "runtime_metrics": len(artifact["runtime_metrics"]),
                "health_events": len(artifact["health_events"]),
                "navigation_status": artifact["final_state"]["navigation"].get("status"),
            }
        )

    summary_path = artifact_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
