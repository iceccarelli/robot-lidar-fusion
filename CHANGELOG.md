# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2025-03-08

### Added

- Complete control stack with 7 modular packages: core, control, perception, planning, power, ai, and orchestrator.
- RobotOrchestrator with deterministic 100 Hz control loop and mock hardware backend.
- LiDAR-camera sensor fusion with sub-100ms time synchronisation.
- Ouster OS1 LiDAR and Intel RealSense camera support via direct SDK and ROS2.
- Multi-signal hazard manager aggregating proximity, voltage, gas, pedestrian, and environmental signals.
- Fault detection for joint positions, velocities, and timestamps.
- LocomotionController with gait generation and velocity-based control.
- JointSynchronizer with per-joint position, velocity, and torque limits.
- MissionPlanner with priority-based goal queue.
- NavigationManager with A*-ready path planning.
- Task-to-hardware mapping with inverse kinematics.
- BatteryManager with state-of-charge tracking and energy estimation.
- ThermalManager with per-joint temperature monitoring and hysteresis cooling.
- PredictiveController for AI-based state estimation.
- VerboseRobotOrchestrator for stress testing with noise injection.
- Multi-environment stress simulation digital twin.
- Calibration files for camera intrinsics and LiDAR-camera extrinsics.
- 14 test files with comprehensive coverage across all modules.
- 3 working examples: basic control loop, sensor fusion demo, stress test.
- Docker and docker-compose support.
- GitHub Actions CI/CD for testing, Docker publishing, and PyPI publishing.
- Full documentation: architecture guide, integration guide, quickstart.
- Enterprise extension placeholders for certified robot connectors and advanced algorithms.
- Gateway placeholder for hosted fleet management service.
