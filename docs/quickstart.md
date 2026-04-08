# Quickstart

This guide helps you validate **robot-lidar-fusion** from a cold start and then move into the new reproducible perception demo. The project now supports two complementary entry paths. The first is a Python-first validation path that checks installation, tests, and the deterministic control-loop scaffold. The second is a **one-command replay workflow** designed to become the default onboarding experience for LiDAR-camera fusion without requiring custom hardware.

## Prerequisites

Use Python 3.11 or later. A virtual environment is strongly recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

If you intend to run the ROS 2 launch-based demo path with RViz, make sure a ROS 2 desktop environment with `rviz2` and `launch` support is available in your shell.

## Installation

Clone the repository and install the package in editable mode.

```bash
git clone https://github.com/iceccarelli/robot-lidar-fusion.git
cd robot-lidar-fusion
pip install -e ".[dev]"
```

Optional extras are available for integration-specific workflows.

| Extra | Purpose |
|---|---|
| `dev` | Formatting, linting, typing, testing, build tooling, and security scanning |
| `ros2` | Python-side ROS 2 integration hooks |
| `sim` | Simulation and benchmark helpers |
| `cv` | Camera and image utilities |
| `ouster` | Ouster SDK integration |
| `realsense` | Intel RealSense integration |

Examples:

```bash
pip install -e ".[dev,cv]"
pip install -e ".[dev,ros2,sim]"
```

## Workflow 1: Validate the Python package locally

The fastest supported validation path remains the Python-only control-loop scaffold.

```bash
python scripts/run_robot.py --cycles 50
pytest -v
python examples/sensor_fusion_demo.py
```

These commands confirm that the package installs correctly, the current test suite runs, and the early perception prototype is available.

## Workflow 2: Run the reproducible demo dataset replay

The repository now includes a bundled LiDAR-camera replay fixture under `datasets/sample_bags/`. This workflow is intended to provide visible perception value before custom hardware is introduced.

### Python-first replay

If you want a hardware-free smoke test without ROS 2 launch, run:

```bash
python scripts/replay_demo_dataset.py --dataset datasets/sample_bags/fusion_demo_sequence.json
```

This command replays the bundled sequence, matches LiDAR and camera frames by timestamp, and prints synchronisation and proximity summaries for each frame.

### ROS 2 launch bringup with RViz

If your environment already has ROS 2 and RViz available, use the one-command bringup:

```bash
ros2 launch launch/bringup_fusion.launch.py
```

This entry point wraps the repository-local replay dataset and opens the default RViz layout from `rviz/default.rviz`. The workflow is designed so that a new user can inspect LiDAR points, camera imagery, and fusion-debug topics from a known deterministic input.

You can also launch the replay entry point directly and control runtime arguments explicitly.

```bash
ros2 launch launch/bringup_bag_replay.launch.py replay_rate:=1.0 max_offset:=0.05 open_rviz:=true
```

## Demo assets

The replay workflow is backed by the following repository-local assets.

| Path | Role |
|---|---|
| `datasets/sample_bags/fusion_demo_sequence.json` | Deterministic LiDAR-camera replay sequence |
| `datasets/sample_bags/metadata.yaml` | Dataset metadata, topics, frame IDs, and expected outputs |
| `launch/bringup_bag_replay.launch.py` | Replay-first bringup entry point |
| `launch/bringup_fusion.launch.py` | Top-level fusion demo bringup |
| `rviz/default.rviz` | Default operator visualisation |
| `scripts/replay_demo_dataset.py` | Python-first replay utility |

## Current scope of the demo

The current demo is intentionally focused on reproducibility and observability rather than claiming a complete autonomy stack.

| Included today | Still in progress |
|---|---|
| Deterministic bundled dataset | Calibration-aware projective fusion |
| LiDAR-camera timestamp matching | TF-aware transforms |
| Replay summary output | Object-level fusion and tracking |
| RViz configuration scaffold | Occupancy maps and costmaps |
| Launch entry points for demo bringup | Nav2 execution path |

## Container workflow

A container build is already available for Python-first validation.

```bash
docker build -t robot-lidar-fusion .
docker run --rm robot-lidar-fusion
```

You can also use Docker Compose if that better fits your local workflow.

```bash
docker compose up --build
```

## Recommended reading order

To understand the repository correctly, continue with these documents.

| Document | Why it matters |
|---|---|
| [`docs/support_matrix.md`](support_matrix.md) | Shows what is supported, experimental, and planned |
| [`docs/architecture.md`](architecture.md) | Explains current package boundaries and data flow |
| [`docs/integration_guide.md`](integration_guide.md) | Describes how to connect custom hardware and sensors |
| [`docs/roadmap.md`](roadmap.md) | Defines the staged plan toward a ROS 2-native release-ready platform |

The guiding principle remains consistent across every workflow: if a capability cannot be demonstrated, tested, typed, scanned, packaged, containerised, and documented, it is not finished.
