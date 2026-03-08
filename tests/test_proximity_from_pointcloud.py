"""Tests for the compute_proximity utility used to derive obstacle distances."""

from robot_hw.perception.lidar_utils import compute_proximity


def test_proximity_computation_basic() -> None:
    """Ensure compute_proximity returns the minimum forward distance."""
    # Points in front of the sensor at distances 3, 1 and 2 metres
    pts = [
        (3.0, 0.0, 0.0),  # 3 m ahead
        (1.0, 0.0, 0.0),  # 1 m ahead, closest
        (2.0, 1.0, 0.0),  # 2 m ahead, y=1
    ]
    prox = compute_proximity(pts, fov_degrees=90.0)
    assert prox is not None
    assert abs(prox - 1.0) < 1e-6


def test_proximity_ignores_negative_x() -> None:
    """Points behind the sensor should be ignored (negative x)."""
    pts = [
        (-1.0, 0.0, 0.0),  # behind sensor
        (0.5, 0.1, 0.0),  # ahead, well within 90-degree FOV
    ]
    prox = compute_proximity(pts, fov_degrees=90.0)
    assert prox is not None
    # Distance is hypot(0.5, 0.1) ≈ 0.5099
    assert abs(prox - 0.5099019513592785) < 1e-4
