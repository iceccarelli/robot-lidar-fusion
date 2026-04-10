from __future__ import annotations

import pytest

from robot_hw.perception.calibration_loader import load_calibration_store
from robot_hw.perception.evaluation import evaluate_projection
from robot_hw.perception.object_fusion import ObjectFusion
from robot_hw.perception.projective_fusion import ProjectiveFusion
from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame
from robot_hw.perception.sensor_processing import SensorProcessor
from robot_hw.robot_config import RobotConfig


def _make_config() -> RobotConfig:
    return RobotConfig(
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
        sensor_calibration_factors=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
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


def _make_lidar_frame() -> LidarFrame:
    return LidarFrame(
        timestamp=1.0,
        frame_id="os_sensor",
        points_xyz=[
            (2.0, 0.2, 0.0),
            (2.1, 0.25, 0.0),
            (3.0, -0.2, 0.2),
            (0.5, -0.5, -2.0),
        ],
        intensities=[0.8, 0.7, 0.6, 0.5],
        metadata={"source": "unit-test"},
    )


def _make_camera_frame() -> CameraFrame:
    return CameraFrame(
        timestamp=1.01,
        frame_id="camera_color_optical_frame",
        image={"width": 640, "height": 480, "encoding": "rgb8", "data": []},
        intrinsics={"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "D": [0.0] * 5},
        metadata={"source": "unit-test"},
    )


def test_load_calibration_store_resolves_optical_frame_path() -> None:
    store = load_calibration_store(
        "calibration/camera_intrinsics.yaml", "calibration/extrinsics.yaml"
    )
    transform = store.get_transform("os_sensor", "camera_color_optical_frame")
    point = transform.transform_point((2.0, 0.2, 0.0))

    assert store.camera.fx == 600.0
    assert transform.child_frame == "os_sensor"
    assert transform.parent_frame == "camera_color_optical_frame"
    assert point[2] > 0.0


def test_projective_fusion_projects_points_into_image_space() -> None:
    store = load_calibration_store(
        "calibration/camera_intrinsics.yaml", "calibration/extrinsics.yaml"
    )
    fusion = ProjectiveFusion(store, max_offset_s=0.05)

    result = fusion.project_pair(_make_lidar_frame(), _make_camera_frame())

    assert result.timestamp_offset_s == pytest.approx(0.01, abs=1e-9)
    assert result.used_transform == "os_sensor->camera_color_optical_frame"
    assert len(result.projected_points) == 4
    assert any(point.in_image_bounds for point in result.projected_points)


def test_object_fusion_and_evaluation_produce_measurable_outputs() -> None:
    store = load_calibration_store(
        "calibration/camera_intrinsics.yaml", "calibration/extrinsics.yaml"
    )
    projection = ProjectiveFusion(store, max_offset_s=0.05).project_pair(
        _make_lidar_frame(),
        _make_camera_frame(),
    )

    fused_objects = ObjectFusion(pixel_radius=40.0, min_cluster_size=1).fuse(projection)
    metrics = evaluate_projection(projection, fused_objects)

    assert len(fused_objects) >= 1
    assert metrics.lidar_points_total == 4
    assert metrics.projected_points_total == 4
    assert metrics.projected_points_in_bounds >= 1
    assert metrics.object_count == len(fused_objects)
    assert metrics.nearest_object_depth_m is not None


def test_sensor_processor_emits_structured_fusion_outputs() -> None:
    processor = SensorProcessor(_make_config())
    fused = processor.fuse_sensors(
        {
            "timestamp": 1.0,
            "positions": {"joint": 0.0},
            "velocities": {"joint": 0.2},
            "lidar_frame": _make_lidar_frame(),
            "camera_frame": _make_camera_frame(),
        }
    )

    assert "fusion" in fused
    assert fused["fusion"]["status"] == "projective_fusion"
    assert fused["fusion"]["projected_points_total"] == 4
    assert fused["fusion"]["metrics"]["object_count"] >= 1
    assert fused["fusion"]["frame_pair"] == "os_sensor->camera_color_optical_frame"
    assert fused["proximity"] > 0.0


def test_sensor_processor_flags_unsynchronized_frames() -> None:
    processor = SensorProcessor(_make_config())
    late_camera = CameraFrame(
        timestamp=1.2,
        frame_id="camera_color_optical_frame",
        image={"width": 640, "height": 480, "encoding": "rgb8", "data": []},
        intrinsics={"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "D": [0.0] * 5},
        metadata={},
    )

    fused = processor.fuse_sensors(
        {
            "timestamp": 1.0,
            "positions": {"joint": 0.0},
            "velocities": {"joint": 0.2},
            "lidar_frame": _make_lidar_frame(),
            "camera_frame": late_camera,
        }
    )

    assert fused["fusion"]["status"] == "projection_failed"
    assert "timestamp offset" in fused["fusion"]["reason"]
