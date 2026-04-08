# Roadmap

This roadmap defines how **robot-lidar-fusion** will evolve from a promising Python robotics skeleton into a reproducible, ROS 2-native, release-ready platform. The ordering is intentional. We prioritise capabilities that create immediate, measurable robotics value before adding optional expansion layers.

## Delivery philosophy

A capability is not complete merely because code exists in the repository. It is complete only when a user can install it, run it, verify it, test it, type-check it, security-scan it, package it, build it in a container, and understand it from the documentation.

The roadmap therefore follows one rule throughout: **if a feature cannot be demonstrated and validated, it is not finished**.

## Stage 1 — Truth, packaging, and workflow discipline

The first stage focuses on aligning repository claims with actual implementation.

| Deliverable | Why it matters |
|---|---|
| Rewrite `README.md` and core docs | Users need an honest boundary between current capability and roadmap |
| Standardise packaging metadata | Release automation depends on clean versioning and extras |
| Add a support matrix | Contributors need a shared understanding of what is supported versus planned |
| Align changelog and repository structure | Public release notes must match demonstrable capability |

**Exit criterion:** the repository presents itself honestly and can be installed, understood, and prepared for stricter CI and release automation.

## Stage 2 — Perfect integrated CI from day one

The second stage turns quality checks into release protection.

| Deliverable | Why it matters |
|---|---|
| Multi-job CI workflow | Linting, typing, testing, security, packaging, and Docker validation must run together |
| Python test matrix for 3.11, 3.12, and 3.13 | The package should stay reproducible across supported interpreters |
| Release-readiness automation | Versioning, changelog, and artifact consistency must be enforced |
| Release publishing workflows | PyPI and container publication should be triggered only from validated releases |

**Exit criterion:** the repository has a strict validation graph that protects releases and surfaces regressions early.

## Stage 3 — Reproducible demo first

The third stage creates the first user-visible robotics workflow that does not depend on custom hardware.

| Deliverable | Why it matters |
|---|---|
| Bag replay bringup | New users need a deterministic way to reproduce sensor flow |
| RViz configuration | Visual trust is critical for robotics adoption |
| Sample dataset metadata and quickstart | Users must know what data is included and how to use it |
| Fusion bringup launch path | The demo should show synchronised LiDAR, camera, and debug outputs with one command |

**Exit criterion:** a new user can clone the repository, run a documented command, and see a reproducible demo with synchronised sensor data and fusion outputs.

## Stage 4 — Real fusion, not placeholders

This stage replaces summary-style fusion with geometry- and timing-aware perception outputs.

| Deliverable | Why it matters |
|---|---|
| Calibration loader | Intrinsics and extrinsics must be first-class runtime inputs |
| Projective fusion | LiDAR points must be projected into the camera frame using calibration-aware geometry |
| Synchronisation policy improvements | Approximate and strict timestamp matching should be measurable and configurable |
| TF-aware transforms | Sensor fusion must respect frame transforms instead of assuming aligned coordinates |
| Object-level fusion and evaluation | Downstream planning needs actionable obstacles, tracks, and metrics rather than summaries |

**Exit criterion:** perception outputs are calibration-aware, synchronised, measurable, and useful for planning and debugging.

## Stage 5 — Mapping and navigation credibility

Once perception is actionable, the stack can generate planning-grade world models and real navigation behaviours.

| Deliverable | Why it matters |
|---|---|
| Occupancy or voxel mapping | The robot needs a persistent spatial world model |
| Costmap generation | Fused perception must become planning-ready inputs |
| Navigation package boundary | Placeholder planning code should be replaced by explicit abstractions |
| Replanning hooks and execution reports | Users need observable navigation behaviour and failure handling |
| Nav2 compatibility | Integration with the ROS 2 navigation ecosystem expands practical adoption |

**Exit criterion:** fused perception can be converted into maps and costmaps that support credible planning and replanning workflows.

## Stage 6 — Simulation, benchmarks, telemetry, and trust

The final stage broadens trust and operability.

| Deliverable | Why it matters |
|---|---|
| Gazebo adapters | Researchers and developers need simulator-first onboarding |
| Isaac Sim adapters | High-fidelity simulation broadens industrial relevance |
| Benchmark scenarios and regression artifacts | Performance claims must be comparable over time |
| Structured logs, runtime metrics, and health events | Operators need observability and fault reporting |
| GitHub-ready release archive | The final repository package should contain only the validated latest files |

**Exit criterion:** the platform can be demonstrated in simulation, benchmarked reproducibly, observed operationally, and released as a clean distribution artifact.

## Workstream ordering

The development order below should remain stable unless a blocking technical dependency appears.

| Order | Workstream | Expected output |
|---|---|---|
| 1 | Repository honesty and packaging discipline | Clear public scope and metadata alignment |
| 2 | Integrated CI and release gating | Strict automation and protected releases |
| 3 | Bag replay and RViz onboarding | Reproducible user-visible demo |
| 4 | Calibration and transform foundations | Trustworthy perception geometry |
| 5 | Real fusion outputs | Actionable obstacles and tracked observations |
| 6 | Mapping and costmaps | Planning-ready world model |
| 7 | Navigation and Nav2 integration | Credible goal execution pipeline |
| 8 | Simulation profiles and benchmarks | Repeatable evaluation baselines |
| 9 | Telemetry and health reporting | Operational maturity |
| 10 | Final release packaging | GitHub-ready distribution ZIP |

## Definition of done

For any roadmap item, the minimum acceptable completion bar is shown below.

| Requirement | Description |
|---|---|
| Demonstrated | There is a documented runnable entry point |
| Tested | Automated tests cover the core behaviour |
| Typed | Static typing covers the public implementation path |
| Scanned | Security checks run successfully or exceptions are documented |
| Packaged | The feature is included correctly in wheel and sdist builds |
| Containerised | The default image build still succeeds |
| Documented | The workflow, scope, and limits are written clearly |

Anything short of that bar should remain labelled as **experimental**, **in progress**, or **planned**.
