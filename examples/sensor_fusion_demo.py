#!/usr/bin/env python3
"""Sensor fusion demonstration.

This script shows how to use the perception subsystem to create LiDAR
and camera frames, synchronise them by timestamp, and compute the
minimum forward obstacle distance from a point cloud.
"""

from __future__ import annotations

import sys
from pathlib import Path

_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_root))
sys.path.insert(0, str(_root / "robot_hw"))

from robot_hw.perception.lidar_utils import min_forward_distance  # noqa: E402
from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame  # noqa: E402
from robot_hw.perception.time_sync import TimeSync  # noqa: E402


def main() -> None:
    ts = TimeSync(max_offset=0.1)

    # Simulate camera frames arriving at 30 fps
    for i in range(10):
        cam = CameraFrame(
            timestamp=i * 0.033,
            frame_id="camera_rgb",
            image=None,
            intrinsics={},
            metadata={},
        )
        ts.add_camera_frame(cam)

    # Simulate a LiDAR frame at t=0.15 s with a few 3D points
    lidar = LidarFrame(
        timestamp=0.15,
        frame_id="os1_lidar",
        points_xyz=[(3.0, 0.0, 0.5), (1.2, 0.3, 0.4), (5.0, -1.0, 0.0)],
        intensities=[100, 200, 50],
        metadata={"ring_count": 64},
    )

    result = ts.match(lidar)
    print(f"Matched camera frame: {result.camera_frame}")
    print(f"Time offset: {result.offset:.4f} s" if result.offset else "No match")

    dist = min_forward_distance(lidar)
    print(f"Minimum forward obstacle distance: {dist:.2f} m")


if __name__ == "__main__":
    main()
