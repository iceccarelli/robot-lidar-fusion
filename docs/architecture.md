# Architecture Guide

This document describes the internal architecture of the Robot LiDAR Fusion control stack. It is intended for contributors and integrators who need to understand how the subsystems fit together and how data flows through the system during each control cycle.

## Design Principles

The architecture follows three guiding principles. First, **determinism**: the control loop runs at a fixed frequency and every subsystem completes its work within a single cycle. Second, **hardware agnosticism**: no module depends on a specific robot platform; all hardware interaction goes through abstract interfaces. Third, **modularity**: each package (core, control, perception, planning, power, ai) can be developed, tested, and replaced independently.

## Package Overview

| Package | Responsibility |
|:---|:---|
| `core` | Foundational services: memory, concurrency, communication, hazard detection, fault detection, hardware sync, environment adaptation, consistency verification, execution stack |
| `control` | Actuator control: joint synchronisation with limits, locomotion with gait generation |
| `perception` | Sensor ingestion and fusion: LiDAR point clouds, camera frames, time synchronisation, obstacle distance computation |
| `planning` | High-level decision making: mission planning, navigation, task-to-hardware mapping |
| `power` | Power management: battery SOC tracking, thermal monitoring with hysteresis cooling |
| `ai` | AI and prediction: predictive state estimation for orientation forecasting |

## Control Loop

The `RobotOrchestrator` runs a deterministic loop at the configured frequency (default 100 Hz). Each cycle executes the following stages in order.

**Stage 1 — Sensor Read.** The hardware synchroniser reads joint positions, velocities, and torques from the hardware backend. The perception pipeline ingests LiDAR and camera frames and synchronises them by timestamp.

**Stage 2 — Manager Update.** The hazard manager aggregates safety signals. The battery manager updates state-of-charge. The thermal manager checks per-joint temperatures. The fault detector scans for anomalies.

**Stage 3 — Task Processing.** The execution stack processes scheduled tasks. The mission planner selects the next goal. The navigation manager computes a path.

**Stage 4 — Hardware Mapping.** The task-to-hardware mapper translates high-level goals into joint-level instructions through inverse kinematics.

**Stage 5 — Joint Synchronisation.** The joint synchroniser dispatches commands to all joints, enforcing position, velocity, and torque limits.

**Stage 6 — Consistency Verification.** The consistency verifier checks that commanded positions match actual positions within tolerance.

**Stage 7 — Telemetry.** The communication interface sends telemetry data to external systems.

## Data Flow

```
Sensors → HardwareSynchronizer → SensorProcessor → HazardManager
                                                  → FaultDetector
                                                  → BatteryManager
                                                  → ThermalManager

MissionPlanner → NavigationManager → TaskHardwareMapper → JointSynchronizer → Hardware

ExecutionStack → (scheduled tasks injected into the loop)

CommunicationInterface ← (telemetry from all managers)
```

## Hardware Abstraction

The `HardwareSynchronizer` in `core/hardware_synchronization.py` provides the abstraction layer between the control stack and the physical robot. It exposes `read_state()` and `write_commands()` methods. The default implementation uses a `MockHardware` backend that simulates joint positions and sensor readings. To connect a real robot, implement the hardware interface and pass it to the orchestrator.

## Perception Pipeline

The perception subsystem supports two ingestion modes. **ROS2 mode** subscribes to standard sensor topics (`/ouster/points`, `/camera/image_raw`) using `rclpy`. **Direct SDK mode** uses the Ouster Python SDK and OpenCV or pyrealsense2 to read sensors without ROS2. Both modes produce `LidarFrame` and `CameraFrame` dataclasses that the `TimeSync` module aligns by timestamp.

## Safety Architecture

Safety is enforced at multiple levels. The `HazardManager` aggregates signals from proximity sensors, voltage monitors, gas detectors, pedestrian detectors, and environmental sensors. The `FaultDetector` monitors joint state for position, velocity, and timestamp anomalies. The `JointSynchronizer` enforces per-joint limits on position, velocity, and torque. The orchestrator can trigger an emergency stop when any safety threshold is breached.

## Extending the Stack

To add a new subsystem, create a new package under `robot_hw/`, implement the module with a clear public API, and wire it into the orchestrator's control loop. Add tests in `tests/` and update the architecture documentation. See the [Integration Guide](integration_guide.md) for step-by-step instructions.
