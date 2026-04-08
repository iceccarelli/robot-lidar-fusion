#!/usr/bin/env python3
"""Validate release-readiness invariants for robot-lidar-fusion.

The script intentionally checks only repository-local facts so it can run
both in CI and in local development without external credentials.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PYPROJECT = ROOT / "pyproject.toml"
PACKAGE_INIT = ROOT / "robot_hw" / "__init__.py"
CHANGELOG = ROOT / "CHANGELOG.md"
README = ROOT / "README.md"
DOCKERFILE = ROOT / "Dockerfile"
CI_WORKFLOW = ROOT / ".github" / "workflows" / "ci.yml"
PYPI_WORKFLOW = ROOT / ".github" / "workflows" / "pypi-publish.yml"
DOCKER_WORKFLOW = ROOT / ".github" / "workflows" / "docker-publish.yml"


def fail(message: str) -> None:
    print(f"release-readiness: FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def require_file(path: Path) -> None:
    if not path.is_file():
        fail(f"required file is missing: {path.relative_to(ROOT)}")


def read_text(path: Path) -> str:
    require_file(path)
    return path.read_text(encoding="utf-8")


def extract_version(pattern: str, text: str, source_name: str) -> str:
    match = re.search(pattern, text, re.MULTILINE)
    if not match:
        fail(f"could not extract version from {source_name}")
    return match.group(1)


def main() -> None:
    for required in [
        PYPROJECT,
        PACKAGE_INIT,
        CHANGELOG,
        README,
        DOCKERFILE,
        CI_WORKFLOW,
        PYPI_WORKFLOW,
        DOCKER_WORKFLOW,
    ]:
        require_file(required)

    pyproject_text = read_text(PYPROJECT)
    package_init_text = read_text(PACKAGE_INIT)
    changelog_text = read_text(CHANGELOG)
    readme_text = read_text(README)

    pyproject_version = extract_version(
        r'^version\s*=\s*"([^"]+)"',
        pyproject_text,
        "pyproject.toml",
    )
    package_version = extract_version(
        r'^__version__\s*=\s*"([^"]+)"',
        package_init_text,
        "robot_hw/__init__.py",
    )
    changelog_version = extract_version(
        r'^## \[([^\]]+)\]',
        changelog_text,
        "CHANGELOG.md",
    )

    if pyproject_version != package_version:
        fail(
            "package version mismatch between pyproject.toml "
            f"({pyproject_version}) and robot_hw/__init__.py ({package_version})"
        )

    if pyproject_version != changelog_version:
        fail(
            "release version mismatch between pyproject.toml "
            f"({pyproject_version}) and CHANGELOG.md ({changelog_version})"
        )

    if "## Repository status" not in readme_text:
        fail("README.md must contain the repository status section")

    if "docs/support_matrix.md" not in readme_text:
        fail("README.md must link to docs/support_matrix.md")

    dist_dir = ROOT / "dist"
    if not dist_dir.is_dir():
        fail("dist/ directory is missing; run the package build before release-readiness")

    wheels = sorted(dist_dir.glob("*.whl"))
    sdists = sorted(dist_dir.glob("*.tar.gz"))
    if not wheels:
        fail("no wheel artifact found in dist/")
    if not sdists:
        fail("no source distribution artifact found in dist/")

    print("release-readiness: PASS")
    print(f"version: {pyproject_version}")
    print(f"wheel: {wheels[0].name}")
    print(f"sdist: {sdists[0].name}")


if __name__ == "__main__":
    main()
