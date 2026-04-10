# Simulation, Benchmarks, Telemetry, and Health Validation

## Purpose

Stage 6 turns `robot-lidar-fusion` from a replay-only perception and navigation repository into a platform with explicit simulation-facing contracts, measurable regression artifacts, and runtime observability. The objective is not to claim bundled simulator binaries; it is to make the integration boundaries **reproducible, testable, and release-auditable**.

## Simulation adapters

The repository now exposes two simulation adapter contracts:

| Adapter | Module | Purpose | Primary outputs |
|---|---|---|---|
| Gazebo | `robot_hw/sim_adapters/gazebo_adapter.py` | Defines the expected ROS 2 topic graph and bridge hints for Gazebo-backed validation. | Topic contract, launch hints, normalized runtime state |
| Isaac Sim | `robot_hw/sim_adapters/isaac_sim_adapter.py` | Defines the expected ROS 2 bridge and topic graph for Isaac Sim-backed validation. | Topic contract, launch hints, normalized runtime state |

These adapters are intentionally lightweight. They make the repository honest: integration is specified in code and covered by tests, while external simulator runtimes remain optional environment dependencies.

## Launch workflow

The simulation entry point is:

```bash
ros2 launch launch/bringup_simulation.launch.py simulator:=gazebo
```

or:

```bash
ros2 launch launch/bringup_simulation.launch.py simulator:=isaac_sim
```

This launch path composes the existing replay-driven fusion and navigation workflow with explicit configuration hooks for:

| Capability | Config file |
|---|---|
| Navigation baseline | `config/navigation/default.yaml` |
| Nav2 compatibility | `config/nav2/nav2_params.yaml` |
| SLAM compatibility | `config/slam/slam_toolbox.yaml` |

## Runtime telemetry

The orchestrator now records three measurable runtime artifact streams on every control cycle:

| Artifact stream | Location in memory | Description |
|---|---|---|
| Runtime metrics | `RobotOrchestrator.runtime_metrics_history` | Per-cycle metrics including loop duration, waypoint counts, hazard count, and map occupancy summaries |
| Structured logs | `RobotOrchestrator.structured_logs` | JSON-friendly log entries combining metrics, state summaries, and health events |
| Health events | `RobotOrchestrator.health_events` | Structured warning/error events generated from battery, thermal, consistency, fault, and hazard conditions |

The transport-facing telemetry payload now includes these artifacts so they can be archived, visualized, or validated in CI.

## Benchmark workflow

The benchmark runner executes deterministic scenarios and emits regression artifacts:

```bash
python3 benchmarks/run_benchmarks.py
```

Scenario definitions are documented under `benchmarks/scenarios/`, and generated artifacts are written to `benchmarks/artifacts/`.

| Scenario | Intent | Expected evidence |
|---|---|---|
| `gazebo_static_obstacles` | Validate static-obstacle navigation and telemetry through the Gazebo adapter contract | Runtime metrics, structured logs, final navigation state |
| `isaac_dynamic_replan` | Validate replanning-oriented navigation and telemetry through the Isaac Sim adapter contract | Runtime metrics, structured logs, final navigation state |

## Regression artifacts

The benchmark suite writes JSON artifacts suitable for CI archival and release review:

| Artifact | Contents |
|---|---|
| `benchmarks/artifacts/gazebo_static_obstacles.json` | Scenario definition, simulator contract, final state, runtime metrics, health events, structured logs, telemetry messages |
| `benchmarks/artifacts/isaac_dynamic_replan.json` | Same structure for the Isaac Sim contract path |
| `benchmarks/artifacts/summary.json` | Aggregate regression summary across all benchmark scenarios |

## Test coverage

Stage 6 is protected by automated tests in `tests/test_simulation_benchmarks_and_telemetry.py`, which verify:

1. Simulation adapter topic contracts.
2. Structured runtime metrics and health event generation.
3. Benchmark artifact generation with measurable telemetry outputs.

## Release-readiness interpretation

A Stage 6 feature is only considered complete when it satisfies all of the following:

| Requirement | Current status |
|---|---|
| Demonstrable | Yes — benchmark runner generates JSON artifacts |
| Testable | Yes — Stage 6 tests validate adapters, telemetry, and artifact generation |
| Typed | Yes — new modules follow the repository typing baseline |
| Scanable in CI | Yes — included in repository-wide quality gates |
| Containerizable | Yes — plain Python modules and JSON artifacts are compatible with the container workflow |
| Documented | Yes — this document and scenario manifests describe the workflow |

This keeps the repository aligned with the non-negotiable rule: **measurable robotics value takes priority over appearance**.
