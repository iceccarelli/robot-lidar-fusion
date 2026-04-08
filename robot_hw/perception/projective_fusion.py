"""Calibration-aware LiDAR-to-camera projection utilities.

The functions in this module convert synchronised LiDAR points into camera
image coordinates using explicit camera intrinsics and TF-style rigid
transforms loaded from calibration assets. The implementation is intentionally
lightweight and dependency-free so it remains easy to test and integrate into
both Python-first and ROS 2-native workflows.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .calibration_loader import CalibrationStore, CameraCalibration
from .sensor_frames import CameraFrame, LidarFrame
from .time_sync import SyncResult, TimeSync


@dataclass(frozen=True)
class ProjectedPoint:
    """A single LiDAR point projected into image space."""

    lidar_point_xyz: tuple[float, float, float]
    camera_point_xyz: tuple[float, float, float]
    pixel_uv: tuple[float, float]
    depth_m: float
    intensity: float | None
    in_image_bounds: bool


@dataclass(frozen=True)
class ProjectionResult:
    """Result of projecting a LiDAR frame into a camera image."""

    lidar_frame: LidarFrame
    camera_frame: CameraFrame
    projected_points: list[ProjectedPoint]
    timestamp_offset_s: float
    used_transform: str


class ProjectiveFusion:
    """Perform timestamp-aware and calibration-aware LiDAR-camera projection."""

    def __init__(self, calibration_store: CalibrationStore, max_offset_s: float = 0.05) -> None:
        self._calibration_store = calibration_store
        self._time_sync = TimeSync(max_offset=max_offset_s)
        self._max_offset_s = float(max_offset_s)

    def add_camera_frame(self, camera_frame: CameraFrame) -> None:
        self._time_sync.add_camera_frame(camera_frame)

    def synchronize_and_project(self, lidar_frame: LidarFrame) -> ProjectionResult | None:
        sync_result = self._time_sync.match(lidar_frame)
        return self.project_sync_result(sync_result)

    def project_sync_result(self, sync_result: SyncResult) -> ProjectionResult | None:
        if sync_result.camera_frame is None or sync_result.offset is None:
            return None
        return self.project_pair(
            lidar_frame=sync_result.lidar_frame,
            camera_frame=sync_result.camera_frame,
            timestamp_offset_s=sync_result.offset,
        )

    def project_pair(
        self,
        lidar_frame: LidarFrame,
        camera_frame: CameraFrame,
        timestamp_offset_s: float | None = None,
    ) -> ProjectionResult:
        timestamp_offset = (
            abs(lidar_frame.timestamp - camera_frame.timestamp)
            if timestamp_offset_s is None
            else float(timestamp_offset_s)
        )
        if timestamp_offset > self._max_offset_s:
            raise ValueError(
                f"timestamp offset {timestamp_offset:.6f}s exceeds configured maximum {self._max_offset_s:.6f}s"
            )

        camera_calibration = _resolve_camera_calibration(
            camera_frame, self._calibration_store.camera
        )
        transform = self._calibration_store.get_transform(
            lidar_frame.frame_id, camera_frame.frame_id
        )
        projected_points: list[ProjectedPoint] = []
        intensities = lidar_frame.intensities or []

        for index, point_xyz in enumerate(lidar_frame.points_xyz):
            camera_point = transform.transform_point(point_xyz)
            cx, cy, cz = camera_point
            if cz <= 0.0:
                continue
            pixel_u = camera_calibration.fx * (cx / cz) + camera_calibration.cx
            pixel_v = camera_calibration.fy * (cy / cz) + camera_calibration.cy
            intensity = float(intensities[index]) if index < len(intensities) else None
            in_bounds = (
                0.0 <= pixel_u < camera_calibration.width
                and 0.0 <= pixel_v < camera_calibration.height
            )
            projected_points.append(
                ProjectedPoint(
                    lidar_point_xyz=(
                        float(point_xyz[0]),
                        float(point_xyz[1]),
                        float(point_xyz[2]),
                    ),
                    camera_point_xyz=(float(cx), float(cy), float(cz)),
                    pixel_uv=(float(pixel_u), float(pixel_v)),
                    depth_m=float(cz),
                    intensity=intensity,
                    in_image_bounds=in_bounds,
                )
            )

        return ProjectionResult(
            lidar_frame=lidar_frame,
            camera_frame=camera_frame,
            projected_points=projected_points,
            timestamp_offset_s=timestamp_offset,
            used_transform=f"{lidar_frame.frame_id}->{camera_frame.frame_id}",
        )


def _resolve_camera_calibration(
    camera_frame: CameraFrame,
    default_calibration: CameraCalibration,
) -> CameraCalibration:
    intrinsics = dict(camera_frame.intrinsics or {})
    if not intrinsics:
        return default_calibration

    width, height = _infer_image_shape(camera_frame.image)
    return CameraCalibration(
        width=int(
            intrinsics.get("width", width if width is not None else default_calibration.width)
        ),
        height=int(
            intrinsics.get("height", height if height is not None else default_calibration.height)
        ),
        fx=float(intrinsics.get("fx", default_calibration.fx)),
        fy=float(intrinsics.get("fy", default_calibration.fy)),
        cx=float(intrinsics.get("cx", default_calibration.cx)),
        cy=float(intrinsics.get("cy", default_calibration.cy)),
        distortion=tuple(
            float(value) for value in intrinsics.get("D", default_calibration.distortion)
        ),
        distortion_model=str(
            intrinsics.get("distortion_model", default_calibration.distortion_model)
        ),
    )


def _infer_image_shape(image: Any) -> tuple[int | None, int | None]:
    if isinstance(image, dict):
        width = image.get("width")
        height = image.get("height")
        if width is not None and height is not None:
            return int(width), int(height)
    shape = getattr(image, "shape", None)
    if isinstance(shape, tuple) and len(shape) >= 2:
        return int(shape[1]), int(shape[0])
    return None, None
