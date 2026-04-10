# robot-lidar-fusion

**robot-lidar-fusion** is a Python-first robotics platform for developing and validating LiDAR-camera perception workflows before integrating them into a broader autonomy stack. The repository already contains a useful skeleton for deterministic orchestration, basic perception data models, mock-hardware execution, and early ROS 2 sensor ingestion. It should currently be treated as a **foundation for a ROS 2-native autonomy platform**, not as a finished end-to-end navigation product.

The immediate goal of this repository is to become a **reproducible, measurable, release-ready bridge** between raw LiDAR-camera plumbing and real robotics workflows for perception, mapping, navigation, simulation, and operational trust.

## Repository status

The table below separates what the repository can do today from what is planned for the next release stages.

| Area | Current status | Notes |
|---|---|---|
| Python packaging | Present, but needs stronger release discipline | `pyproject.toml` exists and installs the package, but release validation and extras need to be expanded |
| Deterministic control loop | Available | `RobotOrchestrator` runs a configurable control loop with mock hardware |
| LiDAR and camera frame models | Available | `LidarFrame` and `CameraFrame` provide a stable typed boundary for perception work |
| Time synchronisation | Basic | Nearest-neighbour timestamp matching exists, but hardware-grade sync policies and evaluation are still needed |
| Sensor fusion | Early prototype | Current fusion exposes summaries and proximity estimates, not calibration-aware projective fusion |
| ROS 2 sensor ingestion | Partial | ROS 2 topic ingestion exists, but launch files, bag replay, TF workflows, and RViz bringup are not yet the primary experience |
| Mapping and costmaps | Not yet implemented | No planning-grade occupancy or voxel mapping pipeline is currently shipped |
| Navigation | Placeholder | Current navigation logic is a demo path generator, not a planning-ready world-model pipeline |
| Simulation profiles | Minimal | Mock and stress simulation exist, but Gazebo and Isaac Sim adapters are still on the roadmap |
| CI/CD | Present, but incomplete for release protection | CI exists, but it must be expanded into a strict multi-job release gate |

## What this project is for

This project is for teams who need to move from raw sensor ingestion toward a reproducible autonomy workflow without starting from a blank repository. It is especially useful if you want to:

| Use case | Why this repository helps |
|---|---|
| Validate LiDAR-camera data flow | The perception package already defines stable frame contracts and a place to add synchronisation and fusion logic |
| Prototype a robotics control loop | The orchestrator, hazard management, power logic, and task mapping provide an executable scaffold |
| Build a ROS 2-native stack incrementally | ROS 2 ingestion hooks already exist and will be expanded with launch, TF, Nav2, and SLAM workflows |
| Standardise testing and release discipline | The package structure is suitable for typed Python development, CI, container builds, and future publishing workflows |

## What this project is not yet

At the time of this revision, **robot-lidar-fusion is not yet a complete autonomy stack**. In particular, users should not assume that the repository already provides production-grade calibration validation, perception benchmarking, mapping, costmaps, Nav2 execution, Gazebo scenarios, Isaac Sim parity, or fleet telemetry. Those capabilities are part of the release plan and are tracked explicitly in the roadmap below.

## Supported workflows

The repository supports the following workflows today.

| Workflow | Status | Entry point |
|---|---|---|
| Local editable install | Supported | `pip install -e .` or `pip install -e ".[dev]"` |
| Mock-hardware control-loop execution | Supported | `python scripts/run_robot.py --cycles 50` |
| Synthetic sensor-fusion demonstration | Supported | `python examples/sensor_fusion_demo.py` |
| Basic ROS 2 sensor ingestion in Python | Experimental | `robot_hw/perception/sensor_io_ros2.py` |
| Direct SDK ingestion for selected sensors | Experimental | `robot_hw/perception/sensor_io_direct.py` |
| Container build | Supported for Python-only workflows | `docker build -t robot-lidar-fusion .` |
| Bag replay plus RViz bringup | Planned for the next release stage | Will be added under `launch/` and `rviz/` |
| Nav2, SLAM, and planning-grade world modelling | Planned | Will be introduced after reproducible fusion outputs exist |

## Supported hardware and interfaces

The project aims to support multiple robot and sensor archetypes, but current support is intentionally scoped.

| Category | Current support level | Notes |
|---|---|---|
| Differential-drive and generic mobile robot logic | Prototype | Basic abstractions exist, but no Nav2-ready execution path is shipped yet |
| Manipulator and jointed robot control abstractions | Prototype | Task mapping and joint synchronisation exist, but hardware backends remain integrator-specific |
| Ouster LiDAR via ROS 2 or direct SDK | Experimental | Ingestion path exists; release-ready calibration and projection workflow is still in progress |
| RealSense or USB cameras | Experimental | Camera ingestion exists; calibration validation and image-overlay tooling are still pending |
| Custom robot backends | Supported by interface pattern | Integrators can replace mock hardware by implementing the expected hardware contract |

## Installation

A Python 3.11+ environment is required.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -e ".[dev]"
```

Optional dependency groups are available for more specialised workflows.

| Extra | Purpose |
|---|---|
| `dev` | Linting, formatting, typing, testing, security, and build tooling |
| `ros2` | Python-side ROS 2 dependencies used by ROS 2 integration layers |
| `sim` | Dependencies used by simulation adapters and benchmark tooling |
| `cv` | Camera and image-processing helpers |
| `ouster` | Ouster SDK support |
| `realsense` | Intel RealSense SDK support |

Examples:

```bash
pip install -e ".[dev,cv]"
pip install -e ".[dev,ros2,sim]"
```

## Quick start

The fastest currently supported workflow is still the Python-only skeleton path.

```bash
python scripts/run_robot.py --cycles 50
pytest -v
python examples/sensor_fusion_demo.py
```

If you are evaluating the repository for ROS 2-native perception work, read the following documents in order:

| Document | Purpose |
|---|---|
| [`docs/quickstart.md`](docs/quickstart.md) | Installation, local validation, and current demo workflow |
| [`docs/architecture.md`](docs/architecture.md) | Current package boundaries and data flow |
| [`docs/integration_guide.md`](docs/integration_guide.md) | Hardware, sensor, container, and ROS 2 integration notes |
| [`docs/support_matrix.md`](docs/support_matrix.md) | Honest matrix of supported, experimental, and planned features |
| [`docs/roadmap.md`](docs/roadmap.md) | Release roadmap for perception, mapping, navigation, simulation, and telemetry |

## Repository structure

```text
robot-lidar-fusion/
├── .github/workflows/        # CI validation and release publishing
├── calibration/              # Calibration templates and examples
├── config/                   # Runtime configuration templates
├── docs/                     # User and contributor documentation
├── examples/                 # Small executable demos
├── robot_hw/                 # Python package
│   ├── ai/                   # Predictive estimation helpers
│   ├── control/              # Joint and locomotion control
│   ├── core/                 # Execution, safety, communication, consistency
│   ├── perception/           # Frame models, sync, ingestion, fusion
│   ├── planning/             # Current planning placeholders to be replaced
│   ├── power/                # Battery and thermal management
│   ├── robot_config.py       # Runtime configuration loader
│   └── robot_orchestrator.py # Main deterministic loop
├── scripts/                  # CLI entry points
└── tests/                    # Unit and integration tests
```

The structure above reflects the **current codebase**. Planned ROS 2 launch assets, RViz configurations, datasets, navigation packages, and simulation adapters will be added in dedicated directories as they become demonstrable and tested.

## Engineering principles

The repository is being developed under four non-negotiable rules.

| Principle | Meaning |
|---|---|
| Honesty over hype | Documentation must separate current capability from roadmap |
| Reproducibility first | Demos must work from bag replay or simulation before hardware-specific claims are expanded |
| Measurable robotics value | Perception, mapping, and navigation work must be benchmarked, tested, and typed |
| Release discipline | Packaging, CI, security scanning, container builds, and documentation must all agree before release |

## Roadmap summary

The next major workstreams are intentionally ordered so that visible robotics value arrives before cosmetic expansion.

| Stage | Focus | Outcome |
|---|---|---|
| 1 | Documentation, packaging, repository honesty | Clear public scope, versioned metadata, and release-ready structure |
| 2 | Integrated CI | Strict multi-job validation and release readiness checks |
| 3 | Reproducible demo | One-command bag replay, RViz, and sample dataset workflow |
| 4 | Real fusion | Calibration-aware projection, timestamp policies, TF-aware transforms, and object fusion |
| 5 | Mapping and navigation | Occupancy or voxel maps, costmaps, replanning, and Nav2 compatibility |
| 6 | Simulation and trust | Gazebo and Isaac Sim adapters, telemetry, benchmarks, regression artifacts, and health events |

The detailed execution plan is documented in [`docs/roadmap.md`](docs/roadmap.md).

## Development workflow

A clean local workflow should remain straightforward.

```bash
pip install -e ".[dev]"
ruff check .
black --check .
mypy robot_hw
pytest -v
python -m build
```

As the repository matures, CI will enforce the same workflow together with security scanning, Docker validation, and release-readiness gates.

## Contributing

Contributions are welcome, especially in the areas of ROS 2 integration, calibration tooling, mapping, costmaps, simulation adapters, reproducible demos, and benchmark automation. Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) before opening a pull request.

## License

This project is distributed under the terms of the [Apache License 2.0](LICENSE).

## Acknowledgements

The repository is maintained by [iceccarelli](https://github.com/iceccarelli). The current direction is to make the project a practical open foundation for reproducible robotics workflows rather than a collection of disconnected demos.
