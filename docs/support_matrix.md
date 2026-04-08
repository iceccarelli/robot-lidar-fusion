# Support Matrix

This document defines what **robot-lidar-fusion** supports today, what is available only as an experimental integration point, and what remains on the roadmap. It is intended to keep repository claims aligned with demonstrable capability.

## Capability matrix

| Capability | Status | Scope | Notes |
|---|---|---|---|
| Editable Python installation | Supported | Python 3.11, 3.12, 3.13 | Managed through `pyproject.toml` |
| Deterministic mock-hardware control loop | Supported | Local development and tests | `RobotOrchestrator` can run without physical hardware |
| Unit-test execution with Pytest | Supported | Local and CI workflows | Test suite already covers core modules |
| Basic LiDAR and camera frame contracts | Supported | Python package API | `LidarFrame` and `CameraFrame` are stable enough for extension |
| Nearest-neighbour time synchronisation | Supported | Early perception prototyping | Suitable for demos, not yet for production-grade validation |
| ROS 2 sensor ingestion | Experimental | Topic subscription and frame conversion | Requires additional bringup, launch, and workflow hardening |
| Direct SDK sensor ingestion | Experimental | Selected LiDAR and camera backends | Intended for prototyping and integrator adaptation |
| Calibration file templates | Supported | Documentation and sample YAML files | Runtime loading and validation logic still need expansion |
| Projective LiDAR-to-camera fusion | Planned | Calibration-aware projection | Not yet shipped as a release-ready workflow |
| Object-level fusion and tracking | Planned | Detection association and tracks | Current state only exposes summaries and proximity |
| Occupancy mapping and costmaps | Planned | Planning-ready world models | No released implementation yet |
| Nav2 compatibility | Planned | ROS 2 navigation interoperability | Placeholder planning code will be replaced |
| Bag replay plus RViz demo | Planned | Reproducible onboarding workflow | Will be introduced as a first-class demo path |
| Gazebo and Isaac Sim adapters | Planned | Simulation workflows | Mock simulation exists, full simulator adapters do not |
| Structured telemetry and health events | Planned | Runtime observability | Communication hooks exist, but structured ops outputs are not complete |
| Release-grade CI protection | In progress | Lint, type, test, security, packaging, container validation | CI exists but will be strengthened into a strict release gate |

## Supported workflows

The table below defines which workflows users should currently rely on.

| Workflow | Recommendation | Rationale |
|---|---|---|
| Python package development | Recommended | This is the strongest current experience |
| Mock-hardware execution | Recommended | Useful for validating control-loop wiring and packaging |
| Synthetic sensor-fusion demo | Recommended | Good for smoke-testing perception data flow |
| Direct use in production robots | Use with caution | Integration points exist, but demonstrable production workflows are still incomplete |
| ROS 2-native evaluation with launch, TF, RViz, and bag replay | Wait for next release stage | This is a priority roadmap item, not a finished workflow yet |
| Planning-grade navigation deployment | Not yet recommended | Navigation currently remains a placeholder implementation |

## Supported hardware profiles

Current hardware support should be interpreted as interface compatibility rather than turnkey deployment.

| Hardware profile | Status | Clarification |
|---|---|---|
| Mock hardware backend | Supported | Primary development path today |
| Ouster LiDAR family | Experimental | Ingestion path exists; validation and calibration workflow are still maturing |
| Intel RealSense cameras | Experimental | Direct and ROS 2 pathways exist, but full demo and calibration workflow are pending |
| Generic USB cameras | Experimental | Best used for prototyping |
| Custom robot SDK backends | Supported by pattern | Integrators are expected to implement their own backend adapter |
| Gazebo robots | Planned | Launch and adapter layer not yet shipped |
| Isaac Sim robots | Planned | Bridge work remains on roadmap |

## Quality gates

A capability is considered complete only when it satisfies the full engineering bar below.

| Requirement | Meaning |
|---|---|
| Demonstrated | A user can run it from a documented entry point |
| Tested | Automated tests verify core behavior |
| Typed | Static typing covers the public code path |
| Scanned | Security tooling runs cleanly or with documented exceptions |
| Packaged | The feature is included in the install and build flow |
| Containerised | The default container build remains valid |
| Documented | Usage, limits, and support level are clearly written |

Anything below that threshold should remain labelled as **experimental**, **in progress**, or **planned**.
