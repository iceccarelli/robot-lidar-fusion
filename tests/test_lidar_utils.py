"""Tests for robot_hw.perception.lidar_utils."""

from __future__ import annotations

from robot_hw.perception.lidar_utils import compute_proximity


def test_compute_proximity_empty():
    """An empty point cloud should return None."""
    result = compute_proximity([])
    assert result is None


def test_compute_proximity_with_forward_points():
    """Should return the closest point in the forward direction."""
    points = [(2.0, 0.0, 0.0), (5.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    result = compute_proximity(points)
    assert result is not None
    assert abs(result - 1.0) < 0.01


def test_compute_proximity_behind_sensor():
    """Points behind the sensor (negative x) should be ignored."""
    points = [(-1.0, 0.0, 0.0), (-5.0, 0.0, 0.0)]
    result = compute_proximity(points)
    assert result is None


def test_compute_proximity_outside_fov():
    """Points outside the field of view should be ignored."""
    # Point at 90 degrees to the side (y >> x)
    points = [(0.1, 10.0, 0.0)]
    result = compute_proximity(points, fov_degrees=30.0)
    assert result is None
