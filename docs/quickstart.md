# Quick Start Guide

This guide will have you running the Robot LiDAR Fusion control stack in under five minutes using mock hardware. No physical robot or sensors required.

## Prerequisites

You need Python 3.11 or later and `pip`. We recommend using a virtual environment.

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## Installation

Clone the repository and install in development mode.

```bash
git clone https://github.com/iceccarelli/robot-lidar-fusion.git
cd robot-lidar-fusion
pip install -e ".[dev]"
```

## Run the Control Loop

The simplest way to see the system in action is to run the control loop with mock hardware.

```bash
python scripts/run_robot.py --cycles 50
```

This starts the orchestrator, runs 50 control cycles at 100 Hz using the mock hardware backend, and prints a summary when complete.

## Run the Test Suite

Verify that everything is working correctly by running the full test suite.

```bash
pytest -v
```

All tests should pass. If any test fails, please open an issue with the full output.

## Try the Examples

The `examples/` directory contains three demonstrations.

**Basic control loop** runs 10 cycles with mock hardware.

```bash
python examples/basic_control_loop.py
```

**Sensor fusion demo** creates synthetic LiDAR and camera frames, synchronises them, and computes obstacle distances.

```bash
python examples/sensor_fusion_demo.py
```

**Stress test** runs 20 cycles with verbose output and noise injection to exercise the full stack under load.

```bash
python examples/stress_test.py
```

## Run with Docker

If you prefer Docker, build and run the container.

```bash
docker compose up --build
```

## Next Steps

Once you are comfortable with the mock hardware, read the [Integration Guide](integration_guide.md) to connect your physical robot and sensors. Read the [Architecture Guide](architecture.md) to understand how the subsystems fit together.
