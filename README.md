# Robot LiDAR Fusion

**The open‑source control stack for autonomous robots with LiDAR‑camera sensor fusion.**  
One codebase. Any robot arm. Any LiDAR. Any camera. Any environment.

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-Apache%202.0-green.svg)](LICENSE)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/iceccarelli/robot-lidar-fusion/actions)
[![Coverage](https://img.shields.io/badge/coverage-85%25-yellow)](https://github.com/iceccarelli/robot-lidar-fusion)

---

## 📋 Table of Contents

- [What Is This?](#what-is-this)
- [Architecture](#architecture)
- [Key Features](#key-features)
- [Quick Start](#quick-start)
- [Project Structure](#project-structure)
- [Supported Sensors](#supported-sensors)
- [Examples](#examples)
- [Enterprise & Gateway](#enterprise--gateway)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## What Is This?

Robot LiDAR Fusion is a **modular, hardware‑agnostic software foundation** for building autonomous robots that perceive the world through LiDAR point clouds and camera images. It provides a complete control stack—from raw sensor ingestion to mission planning and locomotion—running in a deterministic loop at configurable frequencies (default 100 Hz).

We built this because integrating LiDAR, cameras, joint controllers, battery management, hazard detection, and navigation into a single coherent system is **hard**. Every robotics team ends up writing the same glue code. This project provides that glue as a well‑tested, well‑documented open standard so you can focus on what makes your robot unique.

---

## Architecture

The system is orchestrated by a deterministic loop that synchronises all modules every cycle. The diagram below illustrates the core components and their relationships.

```mermaid
flowchart TB
    subgraph Orchestrator["RobotOrchestrator (100 Hz deterministic loop)"]
        direction LR
        A[Cycle Start] --> B[Read Sensors]
        B --> C[Update Managers]
        C --> D[Process Tasks]
        D --> E[Map Instructions]
        E --> F[Sync Actuators]
        F --> G[Verify Consistency]
        G --> A
    end

    subgraph Core["Core Services"]
        H[Hardware Synchronization]
        I[Memory Management]
        J[Hazard Manager]
        K[Fault Detection]
        L[Concurrency Control]
        M[Communication]
        N[Environment Adapter]
        O[Execution Stack]
    end

    subgraph Control["Control"]
        P[Joint Synchronization]
        Q[Locomotion Controller]
    end

    subgraph Perception["Perception"]
        R[LIDAR Utils]
        S[Sensor Frames]
        T[Time Sync]
        U[Sensor I/O (Direct/ROS2)]
        V[Sensor Processing]
    end

    subgraph Planning["Planning"]
        W[Mission Planner]
        X[Navigation Manager]
        Y[Task-Hardware Mapping]
    end

    subgraph Power["Power Management"]
        Z[Battery Manager]
        AA[Thermal Manager]
    end

    subgraph AI["AI Layer"]
        AB[Predictive State Estimation]
    end

    Orchestrator --- Core
    Orchestrator --- Control
    Orchestrator --- Perception
    Orchestrator --- Planning
    Orchestrator --- Power
    Orchestrator --- AI
```

**Deterministic Control Loop** – The `RobotOrchestrator` runs at a configurable frequency (default 100 Hz) with strict cycle‑time enforcement. Each iteration reads sensors, updates all managers, processes the mission queue, maps high‑level tasks to joint‑level instructions, synchronises actuators, and verifies system consistency.

---

## Key Features

- **Sensor Fusion** – Brings together Ouster OS1 LiDAR point clouds and RGB/depth camera frames with sub‑100 ms time synchronisation. The perception pipeline computes obstacle distances, fuses orientation and velocity data, and feeds a unified state representation to the rest of the stack.
- **Hardware Agnostic** – The control stack does not depend on any specific robot platform. A `MockHardware` backend is included for development and testing. Swap it for your robot’s SDK by implementing the hardware interface—everything works unchanged.
- **Safety First** – A multi‑signal hazard manager aggregates proximity, voltage, gas, pedestrian, and environmental hazard signals. The fault detector monitors joint positions, velocities, and timestamps for anomalies. The system can trigger emergency stops when safety thresholds are breached.
- **Dual Sensor Ingestion** – Supports both ROS2 topic subscription and direct vendor SDK ingestion (Ouster SDK, OpenCV, pyrealsense2). Use ROS2 in production for robust driver support, or direct SDK mode for lightweight testing.
- **Mission Planning and Navigation** – Goal‑based mission planner, A*‑ready navigation manager, and task‑to‑hardware mapping that translates high‑level goals into joint‑level instructions through inverse kinematics.
- **Power Management** – Tracks battery state‑of‑charge, estimates task energy costs, defers energy‑intensive tasks when reserves are low, and manages thermal profiles with hysteresis‑based cooling control.
- **AI Layer** – Predictive state estimation that anticipates near‑future sensor readings to reduce latency and improve control smoothness.

---

## Quick Start

```bash
# Clone the repository
git clone https://github.com/iceccarelli/robot-lidar-fusion.git
cd robot-lidar-fusion

# Install in development mode with all extras
pip install -e ".[dev]"

# Run the control loop with mock hardware (50 cycles)
python scripts/run_robot.py --cycles 50

# Run the test suite
pytest -v

# Try a live sensor fusion demo (requires ROS2 or direct SDK)
python examples/demo_os1_camera_live.py
```

---

## Project Structure

```
robot-lidar-fusion/
├── robot_hw/                  # Main package
│   ├── core/                  # Foundational services
│   │   ├── communication.py           # Telemetry and inter‑process messaging
│   │   ├── concurrency_management.py  # Named‑lock concurrency control
│   │   ├── consistency_verification.py # State consistency checks
│   │   ├── environment_adapter.py     # Environment‑specific tuning
│   │   ├── execution_stack.py         # Scheduled task execution
│   │   ├── fault_detection.py         # Joint and sensor fault detection
│   │   ├── hardware_synchronization.py # Hardware read/write abstraction
│   │   ├── hazard_manager.py          # Multi‑signal hazard aggregation
│   │   └── memory_management.py       # Deterministic memory allocation
│   ├── control/               # Actuator control
│   │   ├── joint_synchronization.py   # Joint command dispatch with limits
│   │   └── locomotion_controller.py   # Gait generation and velocity control
│   ├── perception/            # Sensor ingestion and fusion
│   │   ├── lidar_utils.py            # Point cloud utilities
│   │   ├── sensor_frames.py          # LiDARFrame and CameraFrame data models
│   │   ├── sensor_io_direct.py       # Direct SDK ingestion (Ouster, OpenCV)
│   │   ├── sensor_io_ros2.py         # ROS2 topic ingestion
│   │   ├── sensor_processing.py      # Multi‑sensor fusion
│   │   └── time_sync.py              # LiDAR‑camera timestamp alignment
│   ├── planning/              # Mission and navigation
│   │   ├── mission_planner.py        # Goal queue and mission sequencing
│   │   ├── navigation_manager.py     # Path planning and obstacle avoidance
│   │   └── task_hardware_mapping.py  # Task‑to‑joint instruction mapping
│   ├── power/                 # Power management
│   │   ├── battery_management.py     # SOC tracking and energy estimation
│   │   └── thermal_management.py     # Per‑joint thermal monitoring
│   ├── ai/                    # AI and prediction
│   │   └── predictive_controller.py  # State estimation and forecasting
│   ├── robot_config.py        # Environment variable configuration
│   ├── robot_orchestrator.py  # Main control loop
│   ├── simulation.py          # Verbose stress test simulation
│   └── stress_simulation.py   # Multi‑environment digital twin
├── calibration/               # Sensor calibration files
│   ├── camera_intrinsics.yaml
│   ├── extrinsics.yaml
│   └── README.md
├── tests/                     # Test suite
├── examples/                  # Working demonstrations
├── scripts/                   # Entry points
├── docs/                      # Documentation
├── enterprise/                # Enterprise extensions (planned)
├── gateway/                   # Fleet management gateway (planned)
└── config/                    # Configuration templates
```

---

## Supported Sensors

| Sensor               | Interface                  | Status    |
|----------------------|----------------------------|-----------|
| Ouster OS1‑64/‑128   | Direct SDK or ROS2         | Supported |
| Intel RealSense D435/D455 | Direct SDK or ROS2    | Supported |
| USB cameras (UVC)    | OpenCV                     | Supported |
| Generic IMU          | Via SensorProcessor        | Supported |
| Encoders             | Via HardwareSynchronizer   | Supported |

---

## Examples

The `examples/` directory contains three working demonstrations:

- **`basic_control_loop.py`** – Runs 10 cycles of the orchestrator with mock hardware, showing the simplest possible integration.
- **`sensor_fusion_demo.py`** – Creates synthetic LiDAR and camera frames, synchronises them by timestamp, and computes the minimum forward obstacle distance.
- **`demo_os1_camera_live.py`** – Connects to a real Ouster OS1 LiDAR and camera (ROS2 or direct SDK) for live sensor ingestion and time synchronisation.

---

## Enterprise & Gateway

The **`enterprise/`** directory contains planned extensions for:
- Certified robot connectors (KUKA, ABB, Fanuc, Universal Robots)
- Advanced algorithms (EKF fusion, RRT* planning, MPC locomotion, SLAM)
- ISO compliance modules

The **`gateway/`** directory contains the planned hosted Robot Control Gateway for:
- Fleet management
- Telemetry aggregation
- Remote operation
- Over‑the‑air updates

See the README files in each directory for details.

---

## Contributing

We welcome contributions from the robotics community. Whether you are fixing a bug, adding a sensor driver, improving documentation, or proposing a new subsystem, your work helps everyone building autonomous robots.

Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting a pull request.

---

## License

This project is released under the [Apache License 2.0](LICENSE).

---

## Acknowledgements

This project was created and is maintained by [iceccarelli](https://github.com/iceccarelli). It draws on years of experience integrating LiDAR, cameras, and control systems for autonomous robots across industrial, research, and field environments.
