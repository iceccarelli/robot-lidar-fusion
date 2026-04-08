"""Calibration loading and rigid-transform helpers for perception fusion.

This module turns the repository's YAML calibration assets into explicit,
validated runtime objects. It keeps the schema simple enough to match the
existing repository templates while providing the geometry operations needed
for projective LiDAR-camera fusion and TF-style transform chaining.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class CameraCalibration:
    """Camera intrinsic calibration in a fusion-friendly form."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...]
    distortion_model: str = "plumb_bob"


@dataclass(frozen=True)
class RigidTransform:
    """A simple rigid transform between two named coordinate frames."""

    parent_frame: str
    child_frame: str
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]

    def rotation_matrix(self) -> list[list[float]]:
        x, y, z, w = self.rotation_xyzw
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    def inverse(self) -> "RigidTransform":
        rotation = self.rotation_matrix()
        rotation_t = [
            [rotation[0][0], rotation[1][0], rotation[2][0]],
            [rotation[0][1], rotation[1][1], rotation[2][1]],
            [rotation[0][2], rotation[1][2], rotation[2][2]],
        ]
        tx, ty, tz = self.translation
        inv_translation = (
            -(rotation_t[0][0] * tx + rotation_t[0][1] * ty + rotation_t[0][2] * tz),
            -(rotation_t[1][0] * tx + rotation_t[1][1] * ty + rotation_t[1][2] * tz),
            -(rotation_t[2][0] * tx + rotation_t[2][1] * ty + rotation_t[2][2] * tz),
        )
        x, y, z, w = self.rotation_xyzw
        return RigidTransform(
            parent_frame=self.child_frame,
            child_frame=self.parent_frame,
            translation=inv_translation,
            rotation_xyzw=(-x, -y, -z, w),
        )

    def transform_point(self, point_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        rotation = self.rotation_matrix()
        px, py, pz = point_xyz
        tx, ty, tz = self.translation
        return (
            rotation[0][0] * px + rotation[0][1] * py + rotation[0][2] * pz + tx,
            rotation[1][0] * px + rotation[1][1] * py + rotation[1][2] * pz + ty,
            rotation[2][0] * px + rotation[2][1] * py + rotation[2][2] * pz + tz,
        )


class CalibrationStore:
    """Container for intrinsics and a small TF-like transform graph."""

    def __init__(self, camera: CameraCalibration, transforms: list[RigidTransform]) -> None:
        self.camera = camera
        self.transforms = list(transforms)

    def get_transform(self, source_frame: str, target_frame: str) -> RigidTransform:
        if source_frame == target_frame:
            return RigidTransform(
                parent_frame=target_frame,
                child_frame=source_frame,
                translation=(0.0, 0.0, 0.0),
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            )

        direct = self._find_direct(source_frame, target_frame)
        if direct is not None:
            return direct

        path = self._find_path(source_frame, target_frame)
        if path is None:
            raise KeyError(f"no transform path from {source_frame!r} to {target_frame!r}")
        return _compose_transforms(path)

    def _find_direct(self, source_frame: str, target_frame: str) -> RigidTransform | None:
        for transform in self.transforms:
            if transform.parent_frame == target_frame and transform.child_frame == source_frame:
                return transform
            if transform.parent_frame == source_frame and transform.child_frame == target_frame:
                return transform.inverse()
        return None

    def _find_path(self, source_frame: str, target_frame: str) -> list[RigidTransform] | None:
        adjacency: dict[str, list[tuple[str, RigidTransform]]] = {}
        for transform in self.transforms:
            adjacency.setdefault(transform.child_frame, []).append((transform.parent_frame, transform))
            adjacency.setdefault(transform.parent_frame, []).append((transform.child_frame, transform.inverse()))

        queue: list[tuple[str, list[RigidTransform]]] = [(source_frame, [])]
        visited = {source_frame}
        while queue:
            current, path = queue.pop(0)
            for neighbour, edge in adjacency.get(current, []):
                if neighbour in visited:
                    continue
                next_path = [*path, edge]
                if neighbour == target_frame:
                    return next_path
                visited.add(neighbour)
                queue.append((neighbour, next_path))
        return None


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    payload = _read_yaml(path)
    width = int(payload.get("width", 0))
    height = int(payload.get("height", 0))
    distortion = tuple(float(value) for value in payload.get("D", []))

    if "K" in payload:
        matrix = payload["K"]
        if not isinstance(matrix, list) or len(matrix) != 9:
            raise ValueError("camera intrinsics K must contain 9 row-major values")
        fx = float(matrix[0])
        fy = float(matrix[4])
        cx = float(matrix[2])
        cy = float(matrix[5])
    else:
        fx = float(payload["fx"])
        fy = float(payload["fy"])
        cx = float(payload["cx"])
        cy = float(payload["cy"])

    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    return CameraCalibration(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        distortion=distortion,
        distortion_model=str(payload.get("distortion_model", "plumb_bob")),
    )


def load_extrinsics(path: str | Path) -> list[RigidTransform]:
    payload = _read_yaml(path)
    transforms_payload = payload.get("transforms", [])
    if not isinstance(transforms_payload, list):
        raise ValueError("extrinsics file must define a transforms list")

    transforms: list[RigidTransform] = []
    for item in transforms_payload:
        if not isinstance(item, dict):
            raise ValueError("each extrinsic transform entry must be a mapping")
        translation = tuple(float(v) for v in item.get("translation", (0.0, 0.0, 0.0)))
        rotation = tuple(float(v) for v in item.get("rotation", (0.0, 0.0, 0.0, 1.0)))
        if len(translation) != 3:
            raise ValueError("transform translation must have exactly 3 values")
        if len(rotation) != 4:
            raise ValueError("transform rotation must have exactly 4 values")
        transforms.append(
            RigidTransform(
                parent_frame=str(item["parent_frame"]),
                child_frame=str(item["child_frame"]),
                translation=translation,
                rotation_xyzw=_normalise_quaternion(rotation),
            )
        )
    return transforms


def load_calibration_store(
    camera_intrinsics_path: str | Path,
    extrinsics_path: str | Path,
) -> CalibrationStore:
    return CalibrationStore(
        camera=load_camera_calibration(camera_intrinsics_path),
        transforms=load_extrinsics(extrinsics_path),
    )


def _compose_transforms(transforms: list[RigidTransform]) -> RigidTransform:
    if not transforms:
        raise ValueError("cannot compose an empty transform path")
    result = transforms[0]
    for next_transform in transforms[1:]:
        first_rotation = result.rotation_matrix()
        second_translation = next_transform.translation
        translated = (
            first_rotation[0][0] * second_translation[0]
            + first_rotation[0][1] * second_translation[1]
            + first_rotation[0][2] * second_translation[2]
            + result.translation[0],
            first_rotation[1][0] * second_translation[0]
            + first_rotation[1][1] * second_translation[1]
            + first_rotation[1][2] * second_translation[2]
            + result.translation[1],
            first_rotation[2][0] * second_translation[0]
            + first_rotation[2][1] * second_translation[1]
            + first_rotation[2][2] * second_translation[2]
            + result.translation[2],
        )
        result = RigidTransform(
            parent_frame=next_transform.parent_frame,
            child_frame=result.child_frame,
            translation=translated,
            rotation_xyzw=_multiply_quaternions(result.rotation_xyzw, next_transform.rotation_xyzw),
        )
    return result


def _multiply_quaternions(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return _normalise_quaternion(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def _normalise_quaternion(
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = rotation_xyzw
    magnitude_sq = x * x + y * y + z * z + w * w
    if magnitude_sq <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    magnitude = magnitude_sq ** 0.5
    return (x / magnitude, y / magnitude, z / magnitude, w / magnitude)


def _read_yaml(path: str | Path) -> dict[str, Any]:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected a YAML mapping in {path}")
    return payload
