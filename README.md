# Robot LiDAR Fusion

**The open-source control stack for autonomous robots with LiDAR-camera sensor fusion.**

One codebase. Any robot arm. Any LiDAR. Any camera. Any environment.

---

## What This Is

Robot LiDAR Fusion is a modular, hardware-agnostic software foundation for building autonomous robots that see the world through LiDAR point clouds and camera images. It provides a complete control stack, from raw sensor ingestion all the way up to mission planning and locomotion, running in a deterministic loop at configurable frequencies (default 100 Hz).

We built this because integrating LiDAR, cameras, joint controllers, battery management, hazard detection, and navigation into a single coherent system is hard. Every robotics team ends up writing the same glue code. This project provides that glue as a well-tested, well-documented open standard so you can focus on what makes your robot unique.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    RobotOrchestrator                    │
│         Deterministic control loop (100 Hz)             │
├──────────┬──────────┬──────────┬──────────┬─────────────┤
│   Core   │ Control  │Perception│ Planning │    Power    │
│          │          │          │          │             │
│ Hardware │ Joint    │ LiDAR    │ Mission  │ Battery     │
│  Sync    │  Sync    │  Utils   │ Planner  │ Manager     │
│ Memory   │Locomotion│ Sensor   │Navigation│ Thermal     │
│ Hazard   │Controller│  Frames  │ Manager  │ Manager     │
│ Fault    │          │ Time     │ Task-HW  │             │
│ Concur.  │          │  Sync    │ Mapping  │             │
│ Comms    │          │ Sensor   │          │             │
│ Env.     │          │  I/O     │          │             │
│ Consist. │          │ Sensor   │          │             │
│ Exec.    │          │  Proc.   │          │             │
│ HW Sync  │          │          │          │             │
├──────────┴──────────┴──────────┴──────────┴─────────────┤
│                        AI Layer                         │
│              Predictive State Estimation                │
└─────────────────────────────────────────────────────────┘
```

## Key Features

**Sensor Fusion** brings together Ouster OS1 LiDAR point clouds and RGB/depth camera frames with sub-100ms time synchronisation. The perception pipeline computes obstacle distances, fuses orientation and velocity data, and feeds a unified state representation to the rest of the stack.

**Hardware Agnostic** design means the control stack does not depend on any specific robot platform. A `MockHardware` backend is included for development and testing. Swap it for your robot's SDK by implementing the hardware interface, and the entire stack works unchanged.

**Deterministic Control Loop** runs at a configurable frequency (default 100 Hz) with strict cycle-time enforcement. Each cycle reads sensors, updates managers, processes tasks, maps instructions to joints, synchronises actuators, and verifies consistency.

**Safety First** with a multi-signal hazard manager that aggregates proximity, voltage, gas, pedestrian, and environmental hazard signals. The fault detector monitors joint positions, velocities, and timestamps for anomalies. The system can trigger emergency stops when safety thresholds are breached.

**Dual Sensor Ingestion** supports both ROS2 topic subscription and direct vendor SDK ingestion (Ouster SDK, OpenCV, pyrealsense2). Use ROS2 in production for robust driver support, or direct SDK mode for lightweight testing.

**Mission Planning and Navigation** with a goal-based mission planner, A*-ready navigation manager, and task-to-hardware mapping that translates high-level goals into joint-level instructions through inverse kinematics.

**Power Management** tracks battery state-of-charge, estimates task energy costs, defers energy-intensive tasks when reserves are low, and manages thermal profiles with hysteresis-based cooling control.

## Quick Start

```bash
# Clone the repository
git clone https://github.com/iceccarelli/robot-lidar-fusion.git
cd robot-lidar-fusion

# Install in development mode
pip install -e ".[dev]"

# Run the control loop with mock hardware
python scripts/run_robot.py --cycles 50

# Run the test suite
pytest -v
```

## Project Structure

```
robot-lidar-fusion/
├── robot_hw/                  # Main package
│   ├── core/                  # Foundational services
│   │   ├── communication.py       # Telemetry and inter-process messaging
│   │   ├── concurrency_management.py  # Named-lock concurrency control
│   │   ├── consistency_verification.py # State consistency checks
│   │   ├── environment_adapter.py     # Environment-specific tuning
│   │   ├── execution_stack.py         # Scheduled task execution
│   │   ├── fault_detection.py         # Joint and sensor fault detection
│   │   ├── hardware_synchronization.py # Hardware read/write abstraction
│   │   ├── hazard_manager.py          # Multi-signal hazard aggregation
│   │   └── memory_management.py       # Deterministic memory allocation
│   ├── control/               # Actuator control
│   │   ├── joint_synchronization.py   # Joint command dispatch with limits
│   │   └── locomotion_controller.py   # Gait generation and velocity control
│   ├── perception/            # Sensor ingestion and fusion
│   │   ├── lidar_utils.py            # Point cloud utilities
│   │   ├── sensor_frames.py          # LidarFrame and CameraFrame data models
│   │   ├── sensor_io_direct.py       # Direct SDK ingestion (Ouster, OpenCV)
│   │   ├── sensor_io_ros2.py         # ROS2 topic ingestion
│   │   ├── sensor_processing.py      # Multi-sensor fusion
│   │   └── time_sync.py              # LiDAR-camera timestamp alignment
│   ├── planning/              # Mission and navigation
│   │   ├── mission_planner.py        # Goal queue and mission sequencing
│   │   ├── navigation_manager.py     # Path planning and obstacle avoidance
│   │   └── task_hardware_mapping.py  # Task-to-joint instruction mapping
│   ├── power/                 # Power management
│   │   ├── battery_management.py     # SOC tracking and energy estimation
│   │   └── thermal_management.py     # Per-joint thermal monitoring
│   ├── ai/                    # AI and prediction
│   │   └── predictive_controller.py  # State estimation and forecasting
│   ├── robot_config.py        # Environment variable configuration
│   ├── robot_orchestrator.py  # Main control loop (700 lines)
│   ├── simulation.py          # Verbose stress test simulation
│   └── stress_simulation.py   # Multi-environment digital twin
├── calibration/               # Sensor calibration files
│   ├── camera_intrinsics.yaml     # Camera K matrix and distortion
│   ├── extrinsics.yaml            # LiDAR-camera-base transforms
│   └── README.md                  # Calibration guide
├── tests/                     # Test suite
├── examples/                  # Working demonstrations
├── scripts/                   # Entry points
├── docs/                      # Documentation
├── enterprise/                # Enterprise extensions (planned)
├── gateway/                   # Fleet management gateway (planned)
└── config/                    # Configuration templates
```

## Supported Sensors

| Sensor | Interface | Status |
|:---|:---|:---|
| Ouster OS1-64 / OS1-128 | Direct SDK or ROS2 | Supported |
| Intel RealSense D435/D455 | Direct SDK or ROS2 | Supported |
| USB cameras (UVC) | OpenCV | Supported |
| Generic IMU | Via SensorProcessor | Supported |
| Encoders | Via HardwareSynchronizer | Supported |

## Examples

The `examples/` directory contains three working demonstrations.

**basic_control_loop.py** runs 10 cycles of the orchestrator with mock hardware, showing the simplest possible integration.

**sensor_fusion_demo.py** creates synthetic LiDAR and camera frames, synchronises them by timestamp, and computes the minimum forward obstacle distance.

**demo_os1_camera_live.py** connects to a real Ouster OS1 LiDAR and camera (ROS2 or direct SDK) for live sensor ingestion and time synchronisation.

## Enterprise and Gateway

The `enterprise/` directory contains planned extensions for certified robot connectors (KUKA, ABB, Fanuc, Universal Robots), advanced algorithms (EKF fusion, RRT* planning, MPC locomotion, SLAM), and ISO compliance modules.

The `gateway/` directory contains the planned hosted Robot Control Gateway for fleet management, telemetry aggregation, remote operation, and over-the-air updates.

See the README files in each directory for details.

## Contributing

We welcome contributions from the robotics community. Whether you are fixing a bug, adding a sensor driver, improving documentation, or proposing a new subsystem, your work helps everyone building autonomous robots.

Please read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting a pull request.

## Licence

This project is released under the [Apache License 2.0](LICENSE).

## Acknowledgements

This project was created and is maintained by [iceccarelli](https://github.com/iceccarelli). It draws on years of experience integrating LiDAR, cameras, and control systems for autonomous robots across industrial, research, and field environments.
