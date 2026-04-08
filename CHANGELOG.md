# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2026-04-08

### Changed

- Rewrote the repository documentation so that current capability, experimental integrations, and roadmap items are clearly separated.
- Reframed the project as a Python-first robotics foundation evolving toward a ROS 2-native autonomy platform, rather than presenting it as a finished autonomy stack.
- Standardised package metadata in `pyproject.toml`, including richer optional extras for development, ROS 2 integration, simulation, and release workflows.
- Added explicit support and roadmap documentation under `docs/support_matrix.md` and `docs/roadmap.md`.
- Updated the quickstart, architecture, and integration guides to reflect the current maturity of perception, planning, and ROS 2 workflows.

### Added

- Expanded packaging and tooling metadata for formatting, type checking, security scanning, build validation, and release preparation.
- Added project URL metadata for documentation, changelog, issues, repository, and container publication targets.

### Fixed

- Corrected over-optimistic release language that previously implied production-grade fusion, mapping, and navigation were already complete.
- Aligned the documented repository structure with the actual current codebase and staged delivery plan.

## [0.1.0] - 2025-03-08

### Added

- Initial Python package structure for deterministic orchestration, perception, planning, control, power, and AI-related modules.
- Mock-hardware execution path through `RobotOrchestrator` and `scripts/run_robot.py`.
- Typed LiDAR and camera frame contracts together with basic time synchronisation helpers.
- Early ROS 2 and direct sensor-ingestion integration points.
- Initial unit-test suite, examples, container files, and contributor documentation.

### Notes

- Version `0.1.0` established the repository skeleton, but several capabilities described at that time remained prototype-level or planned. Later versions of the changelog use stricter wording so that release notes match demonstrable implementation status.
