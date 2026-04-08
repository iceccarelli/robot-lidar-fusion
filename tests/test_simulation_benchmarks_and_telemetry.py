from __future__ import annotations

from pathlib import Path

from benchmarks.run_benchmarks import run_scenario, BenchmarkScenario
from robot_hw.sim_adapters.gazebo_adapter import GazeboAdapter
from robot_hw.sim_adapters.isaac_sim_adapter import IsaacSimAdapter
from robot_hw.telemetry.runtime_metrics import (
    build_health_events,
    build_runtime_metrics,
    build_structured_log,
)


def test_simulation_adapters_expose_expected_topic_contracts() -> None:
    gazebo = GazeboAdapter()
    isaac = IsaacSimAdapter()

    gazebo_topics = gazebo.describe_topics()
    isaac_topics = isaac.describe_topics()

    assert gazebo_topics["lidar"] == "/lidar/points"
    assert gazebo.build_launch_hints()["simulator"] == "gazebo"
    assert isaac_topics["tf"] == "/tf"
    assert isaac.build_launch_hints()["simulator"] == "isaac_sim"


def test_runtime_metrics_and_health_events_are_structured() -> None:
    state = {
        "timestamp": 1.23,
        "navigation": {"status": "planned"},
        "global_plan": [(0.0, 0.0), (1.0, 0.0)],
        "local_plan": [(0.0, 0.0), (0.5, 0.0)],
        "map": {"occupied_cells": 2, "inflated_cells": 8},
        "hazard_flags": {},
    }

    metrics = build_runtime_metrics(
        cycle=2,
        loop_duration_s=0.005,
        state=state,
        telemetry_count=1,
        fault_count=0,
        hazard_count=1,
    )
    events = build_health_events(
        cycle=2,
        timestamp=1.23,
        battery_ok=True,
        thermal_ok=False,
        consistency_ok=True,
        faults=[],
        hazards={"pedestrian": {"risk_level": "moderate"}},
    )
    structured = build_structured_log(cycle=2, state=state, metrics=metrics, events=events)

    assert metrics["navigation_status"] == "planned"
    assert metrics["global_plan_waypoints"] == 2
    assert len(events) == 2
    assert structured["metrics"]["occupied_cells"] == 2
    assert structured["events"][0]["source"] in {"thermal", "hazard_manager"}


def test_benchmark_scenario_produces_measurable_artifact_payload() -> None:
    scenario = BenchmarkScenario(
        name="unit_gazebo_benchmark",
        simulator="gazebo",
        goal_xy=(1.0, 0.0),
        cycles=2,
        description="Unit benchmark scenario",
    )

    artifact = run_scenario(scenario)

    assert artifact["scenario"]["name"] == "unit_gazebo_benchmark"
    assert artifact["simulator_contract"]["simulator"] == "gazebo"
    assert isinstance(artifact["runtime_metrics"], list)
    assert len(artifact["runtime_metrics"]) == 2
    assert isinstance(artifact["structured_logs"], list)
    assert len(artifact["telemetry_messages"]) == 2
    assert artifact["final_state"]["navigation"]["goal"] == [1.0, 0.0] or artifact["final_state"]["navigation"]["goal"] == (1.0, 0.0)
