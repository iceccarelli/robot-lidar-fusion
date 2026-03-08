# Contributing to Robot LiDAR Fusion

Thank you for your interest in contributing. This project is built by and for the robotics community, and we genuinely appreciate every contribution, whether it is a bug fix, a new sensor driver, improved documentation, or a thoughtful issue report.

## How to Contribute

### Reporting Issues

If you find a bug or have a feature request, please open an issue on GitHub. Include as much context as you can: what you expected, what happened instead, your Python version, operating system, and any relevant logs or stack traces.

### Submitting Pull Requests

1. Fork the repository and create a feature branch from `main`.
2. Install the project in development mode: `pip install -e ".[dev]"`
3. Make your changes, ensuring they follow the coding standards below.
4. Add or update tests to cover your changes.
5. Run the full test suite: `pytest -v`
6. Run the linter: `ruff check .`
7. Commit with a clear, descriptive message.
8. Open a pull request against `main`.

### Coding Standards

We use modern Python (3.11+) conventions throughout the project.

Use `dict[str, Any]` instead of `Dict[str, Any]`. Use `str | None` instead of `Optional[str]`. Use `list[float]` instead of `List[float]`.

All code must pass `ruff check .` with zero errors before merging. The CI pipeline enforces this automatically.

Write docstrings for all public classes and functions using NumPy-style formatting. Include parameter descriptions and return types.

### Testing

Every new feature or bug fix should include tests. We use `pytest` as the test framework. Place tests in the `tests/` directory with filenames matching `test_*.py`.

Run the full suite with:

```bash
pytest -v
```

### Areas Where We Need Help

We are particularly interested in contributions in the following areas:

**New sensor drivers** for LiDAR units beyond Ouster (Velodyne, Livox, Hesai) and cameras beyond RealSense (ZED, OAK-D).

**Navigation algorithms** including A*, RRT, and potential field methods for the navigation manager.

**ROS2 integration** improvements, especially launch files, parameter servers, and lifecycle node support.

**Documentation** including tutorials, architecture diagrams, and deployment guides for specific robot platforms.

**Testing** to increase coverage, especially integration tests that exercise the full orchestrator loop.

## Code of Conduct

All contributors are expected to follow our [Code of Conduct](CODE_OF_CONDUCT.md). We are committed to providing a welcoming and inclusive environment for everyone.

## Licence

By contributing to this project, you agree that your contributions will be licensed under the Apache License 2.0.
