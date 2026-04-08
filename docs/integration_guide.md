# Integration Guide

This guide explains how to integrate **robot-lidar-fusion** with your robot, sensors, and deployment environment while keeping expectations aligned with the current maturity of the repository.

Today, the repository is best used as a **Python robotics foundation** with experimental ROS 2 and direct-sensor integration points. It is not yet a complete turn-key deployment stack for calibrated perception, mapping, and navigation. The goal of this guide is therefore to help you integrate safely and incrementally.

## Integration strategy

A successful integration should proceed in stages rather than all at once.

| Stage | Objective | Recommended outcome |
|---|---|---|
| 1 | Validate the package locally | Run the orchestrator, tests, and synthetic perception demo |
| 2 | Replace mock hardware | Connect a real or simulated robot backend through the hardware abstraction |
| 3 | Connect sensor streams | Feed LiDAR and camera data through ROS 2 or direct SDK pathways |
| 4 | Validate calibration assets | Prepare intrinsics and extrinsics before enabling deeper fusion |
| 5 | Add ROS 2-native workflows | Adopt launch, TF, replay, and visualisation once they are fully shipped |

## Hardware backend integration

The control loop communicates with robot hardware through the hardware synchronisation layer. The default repository path uses mock hardware, which is useful for development but must be replaced for real deployment.

Your backend should expose methods equivalent to the following contract.

```python
class MyRobotBackend:
    def read_state(self) -> dict:
        return {
            "joint_1": {"position": 0.0, "velocity": 0.0, "torque": 0.0},
            "joint_2": {"position": 0.0, "velocity": 0.0, "torque": 0.0},
        }

    def write_commands(self, commands: dict) -> None:
        ...
```

The goal is to keep hardware-specific code outside the rest of the autonomy stack so that perception, planning, and power-management logic remain portable.

## Sensor integration paths

The perception subsystem currently supports two integration patterns.

| Path | Current maturity | Best use |
|---|---|---|
| ROS 2 ingestion | Experimental | Recommended direction for long-term deployment |
| Direct SDK ingestion | Experimental | Useful for lightweight prototyping and vendor-specific tests |

### ROS 2 ingestion

The ROS 2 integration layer converts standard ROS 2 sensor messages into internal `LidarFrame` and `CameraFrame` dataclasses. This is the preferred long-term direction because it aligns with broader ROS 2 tooling such as TF, launch files, RViz, bag replay, Nav2, and SLAM.

At the current stage, ROS 2 ingestion should be viewed as a **code-level integration point**, not yet the full end-user workflow. The repository still needs dedicated bringup launch files, replay workflows, TF validation, and RViz defaults.

### Direct SDK ingestion

Direct SDK ingestion is available for selected sensors, including Ouster and common camera pathways. This mode is helpful when you need to validate device connectivity or run lightweight experiments without a full ROS 2 environment.

It should not yet be treated as the final deployment recommendation for a release-ready autonomy stack.

## LiDAR integration

LiDAR frames are normalised into an internal structure containing timestamp, frame identifier, XYZ point data, optional intensity values, and metadata.

| Requirement | Why it matters |
|---|---|
| Stable timestamps | Synchronisation quality depends on consistent timing |
| Correct frame identifiers | Future TF-aware fusion depends on a coherent frame tree |
| Known sensor pose | Projective fusion requires valid extrinsics |
| Repeatable recordings | Bag replay and benchmark workflows need reproducible input data |

## Camera integration

Camera frames are normalised into an internal structure containing timestamp, frame identifier, image payload, intrinsics, and metadata.

| Requirement | Why it matters |
|---|---|
| Intrinsic calibration | Projection into image space is impossible without it |
| Consistent timestamp source | LiDAR-camera association quality depends on it |
| Image encoding awareness | Overlay and debug workflows rely on predictable image handling |
| Frame naming discipline | TF and launch workflows become easier to reason about |

## Calibration assets

The `calibration/` directory contains example calibration files and conventions. These files are important even before the runtime loader is expanded into a release-grade module.

| File | Purpose |
|---|---|
| `camera_intrinsics.yaml` | Stores camera matrix and distortion information |
| `extrinsics.yaml` | Stores rigid transforms between LiDAR, camera, and robot frames |
| `calibration/README.md` | Describes expected fields and collection guidance |

At the current stage, these files are best treated as **templates and integration references**. The next release stage will add stronger runtime loading, validation, and visual verification tooling.

## Configuration

Runtime configuration is currently environment-driven through `robot_hw/robot_config.py`, with `config/default.yaml` serving as a reference template rather than the sole runtime authority.

Important configuration themes include:

| Theme | Examples |
|---|---|
| Control-loop timing | Cycle time and update rate |
| Power constraints | Battery capacity and thresholds |
| Sensor enable flags | LiDAR and camera toggles |
| Communication settings | Telemetry endpoint and reporting interval |
| Logging settings | Log level and output path |

Future ROS 2-native bringup will extend this configuration surface with launch parameters, calibration file paths, TF settings, and replay or simulation profiles.

## Container deployment

A container build already exists and is suitable for Python-first validation workflows.

```bash
docker build -t robot-lidar-fusion .
docker run --rm robot-lidar-fusion
```

This container path is currently best for development validation, CI, and reproducible package builds. It is not yet a full ROS 2 desktop or simulation image.

## Current integration boundaries

The following statements are important to keep integrations realistic.

| Topic | Current boundary |
|---|---|
| Production-grade time synchronisation | Not yet complete |
| TF-aware projective fusion | Not yet complete |
| Launch and RViz-first onboarding | Not yet complete |
| Occupancy mapping and costmaps | Not yet complete |
| Nav2 execution pipeline | Not yet complete |
| Gazebo and Isaac Sim parity | Not yet complete |

## Recommended integrator workflow

If you are evaluating the repository for a real robot project, the safest order is shown below.

| Order | Action |
|---|---|
| 1 | Install the package and run the tests |
| 2 | Run the mock-hardware orchestrator |
| 3 | Validate your hardware backend independently |
| 4 | Feed LiDAR and camera streams into the internal frame contracts |
| 5 | Prepare calibration assets and verify timestamps |
| 6 | Wait for the bag replay + RViz workflow before evaluating user-facing ROS 2 demos |
| 7 | Adopt mapping, costmaps, and Nav2 only when those modules are demonstrably shipped |

## Related documents

| Document | Use it for |
|---|---|
| [`README.md`](../README.md) | Project scope and current status |
| [`support_matrix.md`](support_matrix.md) | Honest support boundary |
| [`quickstart.md`](quickstart.md) | Cold-start validation |
| [`architecture.md`](architecture.md) | Internal package structure |
| [`roadmap.md`](roadmap.md) | Planned release stages |

The guiding principle for every integration remains the same: **prefer demonstrable, tested workflows over aspirational claims**.
