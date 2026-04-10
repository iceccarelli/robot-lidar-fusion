"""Sensor processing and fusion for robot-lidar-fusion.

This module preserves the existing deterministic state-estimation scaffold
while upgrading the LiDAR-camera pathway from a shallow summary into a
calibration-aware fusion pipeline. When both a LiDAR frame and a camera frame
are available, the processor now attempts to:

1. validate timestamp alignment,
2. load camera intrinsics and TF-style extrinsics,
3. project LiDAR points into the camera frame,
4. cluster projected evidence into object-level observations, and
5. emit measurable fusion metrics for downstream debugging and evaluation.

The implementation is intentionally dependency-light so it remains usable in
local Python workflows, CI, and ROS 2-native bringup paths.
"""

from __future__ import annotations

import logging
import random
from pathlib import Path
from typing import Any, cast

from robot_hw.robot_config import RobotConfig

from .calibration_loader import CalibrationStore, load_calibration_store
from .evaluation import evaluate_projection, metrics_to_dict
from .lidar_utils import compute_proximity
from .object_fusion import ObjectFusion
from .projective_fusion import ProjectiveFusion
from .sensor_frames import CameraFrame, LidarFrame

try:
    from ai.predictive_controller import PredictiveController  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    PredictiveController = None  # type: ignore


class SensorProcessor:
    """Fuse raw sensor inputs into a unified state.

    The processor keeps the original orientation, velocity, and acceleration
    estimation logic for broad compatibility with the current control stack.
    In addition, when LiDAR and camera frames are present it now emits a
    structured ``fusion`` section containing timestamp, projection,
    object-level, and evaluation outputs suitable for debugging and later
    mapping or navigation stages.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._state: dict[str, Any] = {}
        self._prev_lin_vel: tuple[float, float, float] | None = None
        self._prev_timestamp: float | None = None
        self._predictor = PredictiveController(config) if PredictiveController is not None else None
        self._logger = logging.getLogger(self.__class__.__name__)
        try:
            self._noise_std = float(getattr(config, "sensor_noise_std", 0.0))
        except Exception:
            self._noise_std = 0.0
        self._sensor_types = tuple(getattr(config, "environment_sensor_types", ()))
        self._fusion_max_offset_s = 0.05
        self._calibration_store = self._load_default_calibration_store()
        self._projective_fusion = (
            ProjectiveFusion(self._calibration_store, max_offset_s=self._fusion_max_offset_s)
            if self._calibration_store is not None
            else None
        )
        self._object_fusion = ObjectFusion(pixel_radius=24.0, min_cluster_size=1)

    def _load_default_calibration_store(self) -> CalibrationStore | None:
        repo_root = Path(__file__).resolve().parents[2]
        intrinsics_path = repo_root / "calibration" / "camera_intrinsics.yaml"
        extrinsics_path = repo_root / "calibration" / "extrinsics.yaml"
        if not intrinsics_path.is_file() or not extrinsics_path.is_file():
            self._logger.debug("Calibration files not found; projective fusion disabled")
            return None
        try:
            return load_calibration_store(intrinsics_path, extrinsics_path)
        except Exception as exc:
            self._logger.warning("Failed to load calibration assets: %s", exc)
            return None

    def read_sensors(self) -> dict[str, Any]:
        raise NotImplementedError("Sensor reading is provided externally in this implementation")

    def fuse_sensors(self, raw_data: dict[str, Any]) -> dict[str, Any]:
        fused = dict(raw_data)
        timestamp = self._extract_timestamp(raw_data)
        orientation = self._fuse_orientation(raw_data)
        lin_vel = self._fuse_linear_velocity(raw_data)
        acc = self._fuse_acceleration(timestamp, lin_vel)

        fused["orientation"] = orientation
        fused["linear_velocity"] = lin_vel
        fused["acceleration"] = acc

        self._prev_lin_vel = lin_vel
        self._prev_timestamp = timestamp

        for sensor_name in self._sensor_types:
            key = str(sensor_name).lower()
            if key not in fused and key in raw_data:
                fused[key] = raw_data.get(key)

        lidar_frame = raw_data.get("lidar_frame")
        if isinstance(lidar_frame, LidarFrame):
            fused["lidar"] = self._summarize_lidar_frame(lidar_frame)
            proximity = compute_proximity(lidar_frame.points_xyz, fov_degrees=60.0)
            if proximity is not None:
                fused["proximity"] = self._merge_proximity(fused.get("proximity"), proximity)

        camera_frame = raw_data.get("camera_frame")
        if isinstance(camera_frame, CameraFrame):
            fused["camera"] = self._summarize_camera_frame(camera_frame)

        if isinstance(lidar_frame, LidarFrame) and isinstance(camera_frame, CameraFrame):
            fused["fusion"] = self._fuse_lidar_camera(lidar_frame, camera_frame)

        return fused

    def _extract_timestamp(self, raw_data: dict[str, Any]) -> float | None:
        try:
            ts_val = raw_data.get("timestamp")
            return float(ts_val) if ts_val is not None else None
        except Exception:
            return None

    def _fuse_orientation(self, raw_data: dict[str, Any]) -> tuple[float, float, float]:
        orientation: tuple[float, float, float] = (0.0, 0.0, 0.0)
        try:
            raw_orientation = raw_data.get("orientation")
            if isinstance(raw_orientation, (list, tuple)) and len(raw_orientation) >= 3:
                orientation = cast(
                    tuple[float, float, float],
                    (
                        float(raw_orientation[0]),
                        float(raw_orientation[1]),
                        float(raw_orientation[2]),
                    ),
                )
        except Exception:
            orientation = (0.0, 0.0, 0.0)

        calib = self._config.sensor_calibration_factors
        if calib and len(calib) >= 3:
            orientation = (
                float(calib[0]) * orientation[0],
                float(calib[1]) * orientation[1],
                float(calib[2]) * orientation[2],
            )

        if self._predictor is not None:
            try:
                orientation = self._predictor.predict_orientation(orientation, raw_data)
            except Exception as exc:  # pragma: no cover - optional path
                self._logger.debug("Predictive controller error in orientation: %s", exc)

        if self._noise_std > 0.0:
            orientation = (
                orientation[0] + random.gauss(0.0, self._noise_std),
                orientation[1] + random.gauss(0.0, self._noise_std),
                orientation[2] + random.gauss(0.0, self._noise_std),
            )
        return orientation

    def _fuse_linear_velocity(self, raw_data: dict[str, Any]) -> tuple[float, float, float]:
        lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        velocities = raw_data.get("velocities")
        if isinstance(velocities, dict) and velocities:
            try:
                vals = [abs(float(v)) for v in velocities.values() if v is not None]
            except Exception:
                vals = []
            if vals:
                mean_vel = sum(vals) / len(vals)
                lin_vel = (mean_vel, 0.0, 0.0)

        calib = self._config.sensor_calibration_factors
        if calib and len(calib) >= 6:
            lin_vel = (
                float(calib[3]) * lin_vel[0],
                float(calib[4]) * lin_vel[1],
                float(calib[5]) * lin_vel[2],
            )

        if self._noise_std > 0.0:
            lin_vel = (
                lin_vel[0] + random.gauss(0.0, self._noise_std),
                lin_vel[1] + random.gauss(0.0, self._noise_std),
                lin_vel[2] + random.gauss(0.0, self._noise_std),
            )
        return lin_vel

    def _fuse_acceleration(
        self,
        timestamp: float | None,
        lin_vel: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        acc: tuple[float, float, float] = (0.0, 0.0, 0.0)
        if (
            self._prev_lin_vel is not None
            and self._prev_timestamp is not None
            and timestamp is not None
        ):
            dt = timestamp - self._prev_timestamp
            if dt > 0.0:
                try:
                    acc = (
                        (lin_vel[0] - self._prev_lin_vel[0]) / dt,
                        (lin_vel[1] - self._prev_lin_vel[1]) / dt,
                        (lin_vel[2] - self._prev_lin_vel[2]) / dt,
                    )
                except Exception:
                    acc = (0.0, 0.0, 0.0)

        calib = self._config.sensor_calibration_factors
        if calib and len(calib) >= 9:
            acc = (
                float(calib[6]) * acc[0],
                float(calib[7]) * acc[1],
                float(calib[8]) * acc[2],
            )

        if self._noise_std > 0.0:
            acc = (
                acc[0] + random.gauss(0.0, self._noise_std),
                acc[1] + random.gauss(0.0, self._noise_std),
                acc[2] + random.gauss(0.0, self._noise_std),
            )
        return acc

    def _summarize_lidar_frame(self, frame: LidarFrame) -> dict[str, Any]:
        return {
            "timestamp": frame.timestamp,
            "frame_id": frame.frame_id,
            "num_points": len(frame.points_xyz),
            "metadata": frame.metadata,
        }

    def _summarize_camera_frame(self, frame: CameraFrame) -> dict[str, Any]:
        return {
            "timestamp": frame.timestamp,
            "frame_id": frame.frame_id,
            "image": frame.image,
            "intrinsics": frame.intrinsics,
            "metadata": frame.metadata,
        }

    def _merge_proximity(self, current: Any, candidate: float) -> float:
        try:
            return (
                min(float(current), float(candidate)) if current is not None else float(candidate)
            )
        except Exception:
            return float(candidate)

    def _fuse_lidar_camera(
        self, lidar_frame: LidarFrame, camera_frame: CameraFrame
    ) -> dict[str, Any]:
        timestamp_offset = abs(lidar_frame.timestamp - camera_frame.timestamp)
        fusion: dict[str, Any] = {
            "status": (
                "synchronized"
                if timestamp_offset <= self._fusion_max_offset_s
                else "unsynchronized"
            ),
            "timestamp_offset_s": timestamp_offset,
            "max_offset_s": self._fusion_max_offset_s,
            "frame_pair": f"{lidar_frame.frame_id}->{camera_frame.frame_id}",
        }

        if self._projective_fusion is None or self._calibration_store is None:
            fusion["status"] = "calibration_unavailable"
            fusion["reason"] = "camera intrinsics or extrinsics could not be loaded"
            return fusion

        try:
            projection_result = self._projective_fusion.project_pair(
                lidar_frame=lidar_frame,
                camera_frame=camera_frame,
                timestamp_offset_s=timestamp_offset,
            )
        except Exception as exc:
            fusion["status"] = "projection_failed"
            fusion["reason"] = str(exc)
            return fusion

        fused_objects = self._object_fusion.fuse(projection_result)
        metrics = evaluate_projection(projection_result, fused_objects)

        fusion.update(
            {
                "status": "projective_fusion",
                "transform": projection_result.used_transform,
                "projected_points_total": len(projection_result.projected_points),
                "projected_points_in_bounds": sum(
                    1 for point in projection_result.projected_points if point.in_image_bounds
                ),
                "projected_points": [
                    {
                        "lidar_point_xyz": point.lidar_point_xyz,
                        "camera_point_xyz": point.camera_point_xyz,
                        "pixel_uv": point.pixel_uv,
                        "depth_m": point.depth_m,
                        "intensity": point.intensity,
                        "in_image_bounds": point.in_image_bounds,
                    }
                    for point in projection_result.projected_points
                ],
                "objects": [
                    {
                        "object_id": obj.object_id,
                        "point_count": obj.point_count,
                        "centroid_camera_xyz": obj.centroid_camera_xyz,
                        "centroid_pixel_uv": obj.centroid_pixel_uv,
                        "min_depth_m": obj.min_depth_m,
                        "max_depth_m": obj.max_depth_m,
                        "bbox_uv": obj.bbox_uv,
                        "mean_intensity": obj.mean_intensity,
                        "source_frame_pair": obj.source_frame_pair,
                    }
                    for obj in fused_objects
                ],
                "metrics": metrics_to_dict(metrics),
            }
        )
        return fusion

    def update(self) -> dict[str, Any]:
        raw = self.read_sensors()
        fused = self.fuse_sensors(raw)
        self._state = fused
        return fused

    def get_state(self) -> dict[str, Any]:
        return dict(self._state)
