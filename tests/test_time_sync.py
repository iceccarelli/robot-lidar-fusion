"""Tests for the TimeSync helper used to match LiDAR and camera frames."""

from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame
from robot_hw.perception.time_sync import TimeSync


def test_match_nearest_camera_frame() -> None:
    ts = TimeSync(max_offset=0.1)
    # Add two camera frames at 1.0 s and 2.0 s
    cam1 = CameraFrame(
        timestamp=1.0, frame_id="cam", image=None, intrinsics={}, metadata={}
    )
    cam2 = CameraFrame(
        timestamp=2.0, frame_id="cam", image=None, intrinsics={}, metadata={}
    )
    ts.add_camera_frame(cam1)
    ts.add_camera_frame(cam2)
    # LiDAR frame at 1.05 s should match cam1
    lidar = LidarFrame(
        timestamp=1.05, frame_id="lidar", points_xyz=[], intensities=None, metadata={}
    )
    result = ts.match(lidar)
    assert result.camera_frame is cam1
    assert result.offset is not None and abs(result.offset - 0.05) < 1e-6


def test_match_returns_none_when_offset_too_large() -> None:
    ts = TimeSync(max_offset=0.05)
    cam = CameraFrame(
        timestamp=1.0, frame_id="cam", image=None, intrinsics={}, metadata={}
    )
    ts.add_camera_frame(cam)
    # LiDAR frame 0.2 s away should not match
    lidar_far = LidarFrame(
        timestamp=1.2, frame_id="lidar", points_xyz=[], intensities=None, metadata={}
    )
    result = ts.match(lidar_far)
    assert result.camera_frame is None
    assert result.offset is None
