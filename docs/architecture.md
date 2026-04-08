# Architecture

This document describes the **current architecture** of `robot-lidar-fusion` and highlights the boundaries where the next release stages will replace placeholder implementations with release-grade robotics workflows.

The repository should currently be understood as a **layered Python robotics foundation** composed of deterministic orchestration, typed perception contracts, safety and power managers, planning placeholders, and integration hooks for ROS 2 and direct sensor ingestion.

## Architectural intent

The platform is being shaped around four system-level goals.

| Goal | Architectural implication |
|---|---|
| Determinism | The main loop should execute in a predictable order and surface failures clearly |
| Replaceable integration points | Sensor, hardware, and planner implementations should be swappable without rewriting the whole stack |
| Measurable autonomy workflows | Perception and navigation outputs should be testable and benchmarkable |
| ROS 2-native evolution | The repository should align with TF, launch, bag replay, Nav2, and SLAM conventions as it matures |

## Current package layout

The Python package is organised into the following top-level areas.

| Package | Current responsibility | Maturity |
|---|---|---|
| `robot_hw.core` | Execution control, memory, concurrency, communication, hazard and fault handling, environment adaptation | Prototype to early usable |
| `robot_hw.control` | Joint synchronisation and locomotion helpers | Prototype |
| `robot_hw.perception` | Frame models, time sync, sensor ingestion, and early fusion logic | Prototype with clear extension points |
| `robot_hw.planning` | Mission and navigation placeholders | Placeholder |
| `robot_hw.power` | Battery and thermal state handling | Early usable |
| `robot_hw.ai` | Predictive estimation hooks | Experimental |

## Runtime flow today

The current control loop is orchestrated by `RobotOrchestrator`. Its high-level runtime order is shown below.

```text
Hardware / mock state read
    ↓
Sensor ingestion and early fusion
    ↓
Hazard, battery, thermal, and fault updates
    ↓
Mission planning and placeholder navigation
    ↓
Task-to-hardware mapping
    ↓
Joint command synchronisation
    ↓
Consistency verification and telemetry
```

This ordering is useful because it already provides a place for future mapping, costmaps, replanning, and structured telemetry. However, several steps still rely on simplified logic.

## Current perception architecture

The perception subsystem already contains the most important contracts needed for deeper work.

| Component | Current role | Limitation |
|---|---|---|
| `sensor_frames.py` | Defines `LidarFrame` and `CameraFrame` dataclasses | Does not enforce calibration or TF validity on its own |
| `time_sync.py` | Performs nearest-neighbour LiDAR-camera timestamp matching | No advanced sync policy evaluation yet |
| `sensor_io_ros2.py` | Converts ROS 2 messages into internal frame models | Does not yet provide the full launch, TF, and replay workflow |
| `sensor_io_direct.py` | Reads selected sensors directly via SDKs | Best viewed as an integration entry point, not a complete deployment path |
| `sensor_processing.py` | Produces a fused state dictionary | Current fusion is summary-oriented and not yet calibration-aware |

The next architecture milestone is to turn this subsystem into a geometry-aware fusion pipeline with calibration loading, TF-aware transforms, projective association, object-level fusion, and evaluation metrics.

## Current planning architecture

Planning is the part of the repository that most clearly remains a scaffold.

| Component | Current role | Limitation |
|---|---|---|
| `mission_planner.py` | Queues and emits simple goals | No rich execution semantics or reporting |
| `navigation_manager.py` | Generates toy waypoint paths | No occupancy map, costmap, or real planner interface |
| `task_hardware_mapping.py` | Converts high-level targets into joint instructions | Not yet integrated with Nav2-style planning outputs |

This package boundary will likely be refactored into a more explicit navigation layer with planner interfaces, replanning hooks, costmap inputs, and Nav2 interoperability.

## Current safety and operability architecture

The project already includes foundational subsystems that will remain important as the stack becomes more capable.

| Component | Current role |
|---|---|
| `HazardManager` | Aggregates safety-relevant signals such as proximity and environmental risks |
| `FaultDetector` | Watches joint and timestamp anomalies |
| `BatteryManager` | Tracks state-of-charge and energy constraints |
| `ThermalManager` | Monitors per-joint thermal behaviour |
| `CommunicationInterface` | Provides an integration point for telemetry and external messaging |

These modules are valuable because they establish the operational skeleton into which structured logs, health events, and runtime metrics can later be inserted.

## Integration boundaries

The following interfaces are already important extension points.

| Boundary | Purpose |
|---|---|
| Hardware backend contract | Allows replacement of mock hardware with robot-specific SDK integration |
| Perception frame contracts | Keeps ingestion separate from fusion and evaluation |
| Task mapping boundary | Isolates motion command generation from planning logic |
| Configuration loader | Centralises environment-driven runtime settings |

## What will change next

Several architectural changes are planned and should be expected by contributors.

| Planned addition | Why it belongs |
|---|---|
| `launch/` and `rviz/` assets | Needed for ROS 2-native reproducible demos |
| Calibration loader and projective fusion modules | Needed to replace summary-style fusion |
| Occupancy and costmap builders | Needed to turn perception into planning inputs |
| Navigation package or bridge layer | Needed to support replanning and Nav2 compatibility |
| Simulation adapters and benchmarks | Needed to make trust measurable across scenarios |
| Structured telemetry and health event schema | Needed for runtime observability |

## Architectural reading guidance

Contributors should read the repository in the following order.

| Document | Purpose |
|---|---|
| [`README.md`](../README.md) | Public scope and current support boundary |
| [`support_matrix.md`](support_matrix.md) | Honest capability status |
| [`quickstart.md`](quickstart.md) | What can be run today |
| [`integration_guide.md`](integration_guide.md) | How hardware and sensors connect |
| [`roadmap.md`](roadmap.md) | Which placeholders will be replaced next |

This document should evolve conservatively. It should describe the system as it exists, not as we hope it will be after future stages are implemented.
