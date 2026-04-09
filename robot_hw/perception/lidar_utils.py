"""Utility functions for processing LiDAR point clouds.

This module provides helper functions to derive higher-level cues from
raw LiDAR data. The first and simplest cue required for safe
operation is the distance to the nearest obstacle in front of the
robot. The ``compute_proximity`` function computes this value from
a list of 3D points by selecting points within a forward field of
view and returning the minimum planar distance.

Future extensions may include clustering, obstacle detection, ground
plane segmentation and 3D reconstruction. These operations should
be placed in this module or similar modules under ``perception``.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any


def _coerce_point(point: Any) -> tuple[float, float, float] | None:
    """Convert a raw point-like value into a numeric 3D tuple."""
    if not isinstance(point, (list, tuple)) or len(point) < 3:
        return None
    try:
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
    except (TypeError, ValueError):
        return None
    return (x, y, z)


def compute_proximity(
    points: Iterable[tuple[float, float, float]], fov_degrees: float = 60.0
) -> float | None:
    """Compute the distance to the nearest obstacle in the forward field of view.

    Parameters
    ----------
    points : iterable of (x, y, z)
        LiDAR points in the sensor frame. The x-axis is assumed to
        point forward, the y-axis left and the z-axis upwards. Only
        points with positive x (in front of the sensor) are considered.
    fov_degrees : float
        Total horizontal field of view in degrees over which to search
        for obstacles. The function considers points within ±fov/2
        around the x-axis. For example, a value of 60° selects points
        between −30° and +30° relative to the forward direction.

    Returns
    -------
    float or None
        The smallest planar distance (in metres) to any point within
        the forward field of view. Returns ``None`` if no points
        satisfy the criteria.
    """
    try:
        half_fov_rad = math.radians(float(fov_degrees) / 2.0)
    except (TypeError, ValueError):
        half_fov_rad = math.radians(30.0)
    cos_half_fov = math.cos(half_fov_rad)
    min_dist: float | None = None
    for raw_point in points:
        point = _coerce_point(raw_point)
        if point is None:
            continue
        x, y, _z = point
        if x <= 0.0:
            continue
        dist = math.hypot(x, y)
        if dist == 0.0:
            continue
        cos_angle = x / dist
        if cos_angle >= cos_half_fov and (min_dist is None or dist < min_dist):
            min_dist = dist
    return min_dist
