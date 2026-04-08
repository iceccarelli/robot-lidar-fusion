"""Object-level fusion built on calibration-aware LiDAR-camera projection.

This module converts projected LiDAR evidence into lightweight fused objects
that downstream mapping and navigation stages can consume. The current design
keeps the representation simple and testable: points are clustered in camera
space and summarised into obstacle-like observations with image-space and
metric attributes.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from .projective_fusion import ProjectedPoint, ProjectionResult


@dataclass(frozen=True)
class FusedObject:
    """A lightweight fused obstacle candidate."""

    object_id: str
    point_count: int
    centroid_camera_xyz: tuple[float, float, float]
    centroid_pixel_uv: tuple[float, float]
    min_depth_m: float
    max_depth_m: float
    bbox_uv: tuple[float, float, float, float]
    mean_intensity: float | None
    source_frame_pair: str


class ObjectFusion:
    """Cluster projected points into object-like fused observations."""

    def __init__(self, pixel_radius: float = 24.0, min_cluster_size: int = 1) -> None:
        self._pixel_radius = float(pixel_radius)
        self._min_cluster_size = int(min_cluster_size)

    def fuse(self, projection_result: ProjectionResult) -> list[FusedObject]:
        candidates = [point for point in projection_result.projected_points if point.in_image_bounds]
        clusters = _cluster_projected_points(candidates, pixel_radius=self._pixel_radius)

        fused_objects: list[FusedObject] = []
        for index, cluster in enumerate(clusters, start=1):
            if len(cluster) < self._min_cluster_size:
                continue
            fused_objects.append(
                _build_fused_object(
                    object_index=index,
                    cluster=cluster,
                    frame_pair=projection_result.used_transform,
                )
            )
        return fused_objects


def _cluster_projected_points(
    projected_points: list[ProjectedPoint],
    pixel_radius: float,
) -> list[list[ProjectedPoint]]:
    clusters: list[list[ProjectedPoint]] = []
    for point in projected_points:
        assigned = False
        for cluster in clusters:
            if any(_pixel_distance(point, existing) <= pixel_radius for existing in cluster):
                cluster.append(point)
                assigned = True
                break
        if not assigned:
            clusters.append([point])
    return clusters


def _build_fused_object(
    object_index: int,
    cluster: list[ProjectedPoint],
    frame_pair: str,
) -> FusedObject:
    count = len(cluster)
    sum_x = sum(point.camera_point_xyz[0] for point in cluster)
    sum_y = sum(point.camera_point_xyz[1] for point in cluster)
    sum_z = sum(point.camera_point_xyz[2] for point in cluster)
    sum_u = sum(point.pixel_uv[0] for point in cluster)
    sum_v = sum(point.pixel_uv[1] for point in cluster)
    depths = [point.depth_m for point in cluster]
    intensities = [point.intensity for point in cluster if point.intensity is not None]
    u_values = [point.pixel_uv[0] for point in cluster]
    v_values = [point.pixel_uv[1] for point in cluster]

    return FusedObject(
        object_id=f"obj_{object_index:03d}",
        point_count=count,
        centroid_camera_xyz=(sum_x / count, sum_y / count, sum_z / count),
        centroid_pixel_uv=(sum_u / count, sum_v / count),
        min_depth_m=min(depths),
        max_depth_m=max(depths),
        bbox_uv=(min(u_values), min(v_values), max(u_values), max(v_values)),
        mean_intensity=(sum(intensities) / len(intensities)) if intensities else None,
        source_frame_pair=frame_pair,
    )


def _pixel_distance(first: ProjectedPoint, second: ProjectedPoint) -> float:
    du = first.pixel_uv[0] - second.pixel_uv[0]
    dv = first.pixel_uv[1] - second.pixel_uv[1]
    return sqrt(du * du + dv * dv)
