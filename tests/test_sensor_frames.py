"""Unit tests for sensor frame dataclasses."""

from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame


def test_lidar_frame_creation() -> None:
    pts = [(1.0, 0.0, 0.0), (0.5, 1.0, 0.0)]
    intensities = [100.0, 150.0]
    lf = LidarFrame(
        timestamp=1.234,
        frame_id="os_sensor",
        points_xyz=pts,
        intensities=intensities,
        metadata={},
    )
    assert lf.timestamp == 1.234
    assert lf.frame_id == "os_sensor"
    assert lf.points_xyz == pts
    assert lf.intensities == intensities


def test_camera_frame_creation() -> None:
    img = b"\x00\x01\x02"
    intrinsics = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "D": [0.0] * 5}
    cf = CameraFrame(
        timestamp=2.345,
        frame_id="camera",
        image=img,
        intrinsics=intrinsics,
        metadata={},
    )
    assert cf.timestamp == 2.345
    assert cf.frame_id == "camera"
    assert cf.image == img
    assert cf.intrinsics == intrinsics
