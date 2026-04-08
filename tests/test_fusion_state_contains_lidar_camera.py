"""Tests for sensor fusion with LiDAR and camera frames."""

from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame
from robot_hw.perception.sensor_processing import SensorProcessor
from robot_hw.robot_config import RobotConfig


def test_fuse_sensors_includes_lidar_and_camera() -> None:
    """Verify that SensorProcessor.fuse_sensors copies LiDAR and camera data."""
    # Minimal configuration; leave defaults for calibration and safety
    cfg = RobotConfig(
        robot_environment="test",
        operation_mode="test",
        simulator_enabled=True,
        skip_hardware_check=True,
        joint_ids=("joint",),
        actuator_types=("servo",),
        max_torque_util=1.0,
        default_torque_mult=1.0,
        max_velocity_per_joint=(1.0,),
        max_torque_per_joint=(1.0,),
        min_safety_margin=0.0,
        fault_temperature_buffer=0.0,
        max_heat_accumulation=0.0,
        max_cyclic_wear=0.0,
        emergency_stop_cooldown_s=0.0,
        battery_capacity_wh=1.0,
        low_battery_threshold=0.0,
        max_temperature_c=100.0,
        cooling_hysteresis_c=1.0,
        total_memory_bytes=1024,
        lock_names=(),
        enable_monitoring_ui=False,
        monitor_port=0,
        robot_log_level="INFO",
        enable_aws_metrics=False,
        aws_region="us-east-1",
        robot_control_key="",
        robot_telemetry_key="",
        cycle_time_s=0.01,
        enable_power_saving_mode=False,
        power_budget_w=10.0,
        environment_profile="lab",
        environment_hardware_ids=(),
        sensor_calibration_factors=(1.0, 1.0),
        safety_thresholds=(0.0,),
        environment_power_budget_w=10.0,
        environment_thermal_limit_c=100.0,
        path_planning_algorithm="none",
        locomotion_mode="stationary",
        sensor_noise_std=0.0,
        environment_sensor_types=("lidar", "camera"),
        max_position_error=0.0,
        max_velocity_error=0.0,
        enable_lidar=True,
        enable_camera=True,
        use_ros2=False,
        lidar_topic="/lidar",
        camera_topic="/camera",
        camera_info_topic="/camera_info",
        lidar_ip="127.0.0.1",
        host_ip="127.0.0.1",
        lidar_port=0,
        imu_port=0,
        camera_device=0,
    )
    sp = SensorProcessor(cfg)
    # Construct synthetic LiDAR frame with two points at distances 2.0 and 0.5 m
    pts = [(2.0, 0.0, 0.0), (0.5, 0.0, 0.0)]
    intensities = [10.0, 20.0]
    lf = LidarFrame(
        timestamp=1.0, frame_id="os", points_xyz=pts, intensities=intensities, metadata={}
    )
    # Camera frame with dummy image bytes
    cf = CameraFrame(
        timestamp=1.0, frame_id="cam", image=b"\x01\x02", intrinsics={"fx": 1.0}, metadata={}
    )
    # Raw data used by SensorProcessor; include a timestamp and velocities for acceleration
    raw_data = {
        "timestamp": 1.0,
        "positions": {"joint": 0.0},
        "velocities": {"joint": 0.0},
        "lidar_frame": lf,
        "camera_frame": cf,
    }
    fused = sp.fuse_sensors(raw_data)
    # Ensure fused state has lidar summary, camera summary and proximity estimate
    assert "lidar" in fused
    assert fused["lidar"]["num_points"] == 2
    assert "camera" in fused
    assert fused["camera"]["image"] == b"\x01\x02"
    # The proximity should be the minimum positive x‑distance (0.5)
    assert "proximity" in fused
    assert abs(fused["proximity"] - 0.5) < 1e-6
