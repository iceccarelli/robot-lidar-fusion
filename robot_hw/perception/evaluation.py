"""Evaluation helpers for calibration-aware LiDAR-camera fusion.

This module turns projection and object-fusion outputs into measurable
artifacts that can be tested locally, surfaced in CI, and consumed by later
benchmark and telemetry stages.
"""

from __future__ import annotations

from dataclasses import dataclass

from .object_fusion import FusedObject
from .projective_fusion import ProjectionResult


@dataclass(frozen=True)
class FusionMetrics:
    """Summary metrics for one synchronised LiDAR-camera fusion step."""

    timestamp_offset_s: float
    lidar_points_total: int
    projected_points_total: int
    projected_points_in_bounds: int
    projection_coverage_ratio: float
    object_count: int
    nearest_object_depth_m: float | None


def evaluate_projection(
    projection_result: ProjectionResult,
    fused_objects: list[FusedObject],
) -> FusionMetrics:
    total_points = len(projection_result.lidar_frame.points_xyz)
    projected_points_total = len(projection_result.projected_points)
    projected_points_in_bounds = sum(
        1 for point in projection_result.projected_points if point.in_image_bounds
    )
    projection_coverage_ratio = (
        projected_points_in_bounds / total_points if total_points > 0 else 0.0
    )
    nearest_object_depth = (
        min(obj.min_depth_m for obj in fused_objects) if fused_objects else None
    )
    return FusionMetrics(
        timestamp_offset_s=projection_result.timestamp_offset_s,
        lidar_points_total=total_points,
        projected_points_total=projected_points_total,
        projected_points_in_bounds=projected_points_in_bounds,
        projection_coverage_ratio=projection_coverage_ratio,
        object_count=len(fused_objects),
        nearest_object_depth_m=nearest_object_depth,
    )


def metrics_to_dict(metrics: FusionMetrics) -> dict[str, float | int | None]:
    return {
        "timestamp_offset_s": metrics.timestamp_offset_s,
        "lidar_points_total": metrics.lidar_points_total,
        "projected_points_total": metrics.projected_points_total,
        "projected_points_in_bounds": metrics.projected_points_in_bounds,
        "projection_coverage_ratio": metrics.projection_coverage_ratio,
        "object_count": metrics.object_count,
        "nearest_object_depth_m": metrics.nearest_object_depth_m,
    }
