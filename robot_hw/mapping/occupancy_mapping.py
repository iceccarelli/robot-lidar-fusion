"""Occupancy-grid and costmap generation for planning.

This module converts fused perception outputs into deterministic 2D planning
products. The implementation intentionally stays lightweight and testable:
obstacle-like fused objects are rasterised into an occupancy grid, then
inflated into a navigation costmap suitable for local planning and Nav2-style
consumers.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot
from typing import Any


@dataclass(frozen=True)
class GridMap:
    """A simple 2D grid map with metric metadata."""

    width: int
    height: int
    resolution_m: float
    origin_xy: tuple[float, float]
    data: list[list[int]]

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_grid(self, x_m: float, y_m: float) -> tuple[int, int]:
        gx = int((x_m - self.origin_xy[0]) / self.resolution_m)
        gy = int((y_m - self.origin_xy[1]) / self.resolution_m)
        return gx, gy


class OccupancyMapper:
    """Build occupancy grids and inflated costmaps from fused perception."""

    def __init__(
        self,
        *,
        width: int = 80,
        height: int = 80,
        resolution_m: float = 0.1,
        origin_xy: tuple[float, float] = (-4.0, -4.0),
        inflation_radius_m: float = 0.4,
        obstacle_cost: int = 100,
    ) -> None:
        self._width = int(width)
        self._height = int(height)
        self._resolution_m = float(resolution_m)
        self._origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self._inflation_radius_m = float(inflation_radius_m)
        self._obstacle_cost = int(obstacle_cost)

    def build_occupancy_grid(self, state: dict[str, Any]) -> GridMap:
        grid = self._empty_grid(default_value=0)
        for x_m, y_m in self._iter_obstacle_points(state):
            gx, gy = grid.world_to_grid(x_m, y_m)
            if grid.in_bounds(gx, gy):
                grid.data[gy][gx] = self._obstacle_cost
        return grid

    def build_costmap(self, occupancy_grid: GridMap) -> GridMap:
        inflated = [row[:] for row in occupancy_grid.data]
        cells = max(0, ceil(self._inflation_radius_m / occupancy_grid.resolution_m))
        occupied_cells = [
            (ix, iy)
            for iy, row in enumerate(occupancy_grid.data)
            for ix, value in enumerate(row)
            if value >= self._obstacle_cost
        ]

        for ox, oy in occupied_cells:
            for dy in range(-cells, cells + 1):
                for dx in range(-cells, cells + 1):
                    gx = ox + dx
                    gy = oy + dy
                    if not occupancy_grid.in_bounds(gx, gy):
                        continue
                    distance = hypot(
                        dx * occupancy_grid.resolution_m, dy * occupancy_grid.resolution_m
                    )
                    if distance > self._inflation_radius_m:
                        continue
                    cost = self._inflation_cost(distance)
                    if cost > inflated[gy][gx]:
                        inflated[gy][gx] = cost

        return GridMap(
            width=occupancy_grid.width,
            height=occupancy_grid.height,
            resolution_m=occupancy_grid.resolution_m,
            origin_xy=occupancy_grid.origin_xy,
            data=inflated,
        )

    def to_nav2_costmap_dict(self, costmap: GridMap) -> dict[str, Any]:
        return {
            "metadata": {
                "width": costmap.width,
                "height": costmap.height,
                "resolution": costmap.resolution_m,
                "origin": {"x": costmap.origin_xy[0], "y": costmap.origin_xy[1]},
            },
            "data": [value for row in costmap.data for value in row],
            "encoding": "costmap_2d",
        }

    def summarize(self, occupancy_grid: GridMap, costmap: GridMap) -> dict[str, Any]:
        occupied = sum(
            1 for row in occupancy_grid.data for value in row if value >= self._obstacle_cost
        )
        lethal = sum(1 for row in costmap.data for value in row if value >= self._obstacle_cost)
        inflated = sum(
            1 for row in costmap.data for value in row if 0 < value < self._obstacle_cost
        )
        return {
            "grid": {
                "width": occupancy_grid.width,
                "height": occupancy_grid.height,
                "resolution_m": occupancy_grid.resolution_m,
                "origin_xy": occupancy_grid.origin_xy,
            },
            "occupied_cells": occupied,
            "lethal_cells": lethal,
            "inflated_cells": inflated,
        }

    def _empty_grid(self, *, default_value: int) -> GridMap:
        return GridMap(
            width=self._width,
            height=self._height,
            resolution_m=self._resolution_m,
            origin_xy=self._origin_xy,
            data=[[default_value for _ in range(self._width)] for _ in range(self._height)],
        )

    def _iter_obstacle_points(self, state: dict[str, Any]) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        fusion = state.get("fusion") if isinstance(state, dict) else None
        if isinstance(fusion, dict):
            objects = fusion.get("objects", [])
            if isinstance(objects, list):
                for obj in objects:
                    if not isinstance(obj, dict):
                        continue
                    centroid = obj.get("centroid_camera_xyz")
                    if isinstance(centroid, (list, tuple)) and len(centroid) >= 3:
                        try:
                            x_cam = float(centroid[2])
                            y_cam = -float(centroid[0])
                            points.append((x_cam, y_cam))
                        except Exception:
                            continue
        if points:
            return points

        lidar = state.get("lidar_frame") if isinstance(state, dict) else None
        if lidar is not None:
            cloud = getattr(lidar, "points_xyz", None)
            if isinstance(cloud, list):
                for point in cloud:
                    if isinstance(point, (list, tuple)) and len(point) >= 2:
                        try:
                            points.append((float(point[0]), float(point[1])))
                        except Exception:
                            continue
        return points

    def _inflation_cost(self, distance_m: float) -> int:
        if distance_m <= 0.0:
            return self._obstacle_cost
        ratio = max(0.0, 1.0 - (distance_m / self._inflation_radius_m))
        return int(round(ratio * (self._obstacle_cost - 1)))
