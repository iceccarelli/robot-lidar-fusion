"""
Robot configuration parsing and validation.

This module provides the single typed configuration entrypoint for the
robot platform. Environment variables are normalised into an immutable
``RobotConfig`` instance so the rest of the package can depend on a
stable, validated configuration object.

The implementation is intentionally environment-first and side-effect
free. It performs no network or hardware I/O and is therefore safe to
import in local development, CI, and packaging workflows.
"""

from __future__ import annotations

import os
from dataclasses import dataclass


def _b(name: str, default: bool = False) -> bool:
    """Parse an environment variable into a boolean."""
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "t", "yes", "y", "on"}


def _f(
    name: str,
    default: float,
    *,
    min: float | None = None,
    max: float | None = None,
) -> float:
    """Parse an environment variable into a float with optional bounds."""
    raw = os.getenv(name)
    try:
        value = default if raw is None or raw.strip() == "" else float(raw)
    except ValueError:
        value = default
    if min is not None and value < min:
        raise ValueError(f"{name}={value} is below minimum {min}")
    if max is not None and value > max:
        raise ValueError(f"{name}={value} is above maximum {max}")
    return value


def _i(
    name: str,
    default: int,
    *,
    min: int | None = None,
    max: int | None = None,
) -> int:
    """Parse an environment variable into an integer with optional bounds."""
    raw = os.getenv(name)
    try:
        value = default if raw is None or raw.strip() == "" else int(raw)
    except ValueError:
        value = default
    if min is not None and value < min:
        raise ValueError(f"{name}={value} is below minimum {min}")
    if max is not None and value > max:
        raise ValueError(f"{name}={value} is above maximum {max}")
    return value


def _optional_i(name: str) -> int | None:
    """Parse an optional integer environment variable.

    The values ``None``, an empty string, and the case-insensitive token
    ``none`` are all treated as the absence of a value.
    """
    raw = os.getenv(name)
    if raw is None:
        return None
    stripped = raw.strip()
    if stripped == "" or stripped.lower() == "none":
        return None
    return int(stripped)


def _csv(name: str, default: str) -> tuple[str, ...]:
    """Parse a comma-separated environment variable into a tuple of strings."""
    raw = os.getenv(name, default)
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def _floats_csv(name: str, default: str) -> tuple[float, ...]:
    """Parse a comma-separated environment variable into a tuple of floats."""
    raw = os.getenv(name, default)
    values: list[float] = []
    for part in raw.split(","):
        stripped = part.strip()
        if stripped == "":
            continue
        try:
            values.append(float(stripped))
        except ValueError as err:
            raise ValueError(f"Invalid float in {name}: {stripped}") from err
    return tuple(values)


@dataclass(frozen=True)
class RobotConfig:
    """Typed runtime configuration for the robot platform."""

    robot_environment: str
    operation_mode: str
    simulator_enabled: bool
    skip_hardware_check: bool
    joint_ids: tuple[str, ...]
    actuator_types: tuple[str, ...]
    max_torque_util: float
    default_torque_mult: float
    max_velocity_per_joint: tuple[float, ...]
    max_torque_per_joint: tuple[float, ...]
    min_safety_margin: float
    fault_temperature_buffer: float
    max_heat_accumulation: float
    max_cyclic_wear: float
    emergency_stop_cooldown_s: float
    battery_capacity_wh: float
    low_battery_threshold: float
    max_temperature_c: float
    cooling_hysteresis_c: float
    total_memory_bytes: int
    lock_names: tuple[str, ...]
    enable_monitoring_ui: bool
    monitor_port: int
    robot_log_level: str
    enable_aws_metrics: bool
    aws_region: str
    robot_control_key: str | None
    robot_telemetry_key: str | None
    cycle_time_s: float
    enable_power_saving_mode: bool
    power_budget_w: float
    environment_profile: str
    environment_hardware_ids: tuple[str, ...]
    sensor_calibration_factors: tuple[float, ...]
    safety_thresholds: tuple[float, ...]
    environment_power_budget_w: float
    environment_thermal_limit_c: float
    path_planning_algorithm: str
    locomotion_mode: str
    sensor_noise_std: float
    environment_sensor_types: tuple[str, ...]
    max_position_error: float
    max_velocity_error: float
    enable_lidar: bool
    enable_camera: bool
    use_ros2: bool
    lidar_topic: str
    camera_topic: str
    camera_info_topic: str
    lidar_ip: str
    host_ip: str
    lidar_port: int
    imu_port: int
    camera_device: int | None


def load() -> RobotConfig:
    """Load and validate runtime configuration from environment variables."""
    power_budget_w = _f("POWER_BUDGET_W", 100.0, min=0.0)
    max_temperature_c = _f("MAX_TEMPERATURE_C", 60.0)

    return RobotConfig(
        robot_environment=os.getenv("ROBOT_ENVIRONMENT", "production"),
        operation_mode=os.getenv("OPERATION_MODE", "operational"),
        simulator_enabled=_b("SIMULATOR_ENABLED", False),
        skip_hardware_check=_b("SKIP_HARDWARE_CHECK", False),
        joint_ids=_csv("JOINT_IDS", "joint1,joint2,joint3,joint4"),
        actuator_types=_csv("ACTUATOR_TYPES", "servo,servo,servo,servo"),
        max_torque_util=_f("MAX_TORQUE_UTIL", 1.0, min=0.0, max=1.0),
        default_torque_mult=_f("DEFAULT_TORQUE_MULT", 0.5, min=0.0),
        max_velocity_per_joint=_floats_csv(
            "MAX_VELOCITY_PER_JOINT",
            "1.0,1.0,1.0,1.0",
        ),
        max_torque_per_joint=_floats_csv(
            "MAX_TORQUE_PER_JOINT",
            "1.5,1.5,1.5,1.5",
        ),
        min_safety_margin=_f("MIN_SAFETY_MARGIN", 0.1, min=0.0, max=1.0),
        fault_temperature_buffer=_f("FAULT_TEMPERATURE_BUFFER", 5.0, min=0.0),
        max_heat_accumulation=_f("MAX_HEAT_ACCUMULATION", 0.8, min=0.0, max=1.0),
        max_cyclic_wear=_f("MAX_CYCLIC_WEAR", 0.7, min=0.0, max=1.0),
        emergency_stop_cooldown_s=_f("EMERGENCY_STOP_COOLDOWN_S", 5.0, min=0.0),
        battery_capacity_wh=_f("BATTERY_CAPACITY_WH", 100.0, min=0.0),
        low_battery_threshold=_f("LOW_BATTERY_THRESHOLD", 0.2, min=0.0, max=1.0),
        max_temperature_c=max_temperature_c,
        cooling_hysteresis_c=_f("COOLING_HYSTERESIS_C", 5.0, min=0.0),
        total_memory_bytes=_i("TOTAL_MEMORY_BYTES", 1_048_576, min=0),
        lock_names=_csv("LOCK_NAMES", "hardware,memory,tasks"),
        enable_monitoring_ui=_b("ENABLE_MONITORING_UI", False),
        monitor_port=_i("MONITOR_PORT", 8051, min=0),
        robot_log_level=os.getenv("ROBOT_LOG_LEVEL", "INFO"),
        enable_aws_metrics=_b("ENABLE_AWS_METRICS", False),
        aws_region=os.getenv("AWS_REGION", "eu-central-1"),
        robot_control_key=os.getenv("ROBOT_CONTROL_KEY"),
        robot_telemetry_key=os.getenv("ROBOT_TELEMETRY_KEY"),
        cycle_time_s=_f("CYCLE_TIME_S", 0.01, min=1e-4),
        enable_power_saving_mode=_b("ENABLE_POWER_SAVING_MODE", False),
        power_budget_w=power_budget_w,
        environment_profile=os.getenv("ENVIRONMENT_PROFILE", "GENERAL").upper(),
        environment_hardware_ids=_csv("ENVIRONMENT_HARDWARE_IDS", ""),
        sensor_calibration_factors=_floats_csv("SENSOR_CALIBRATION_FACTORS", ""),
        safety_thresholds=_floats_csv("SAFETY_THRESHOLDS", ""),
        environment_power_budget_w=_f(
            "ENVIRONMENT_POWER_BUDGET_W",
            power_budget_w,
            min=0.0,
        ),
        environment_thermal_limit_c=_f(
            "ENVIRONMENT_THERMAL_LIMIT_C",
            max_temperature_c,
        ),
        path_planning_algorithm=os.getenv("PATH_PLANNING_ALGORITHM", "A_STAR").upper(),
        locomotion_mode=os.getenv("LOCOMOTION_MODE", "WHEELS").upper(),
        sensor_noise_std=_f("SENSOR_NOISE_STD", 0.0, min=0.0),
        environment_sensor_types=_csv("ENVIRONMENT_SENSOR_TYPES", ""),
        max_position_error=_f("MAX_POSITION_ERROR", 5.0, min=0.0),
        max_velocity_error=_f("MAX_VELOCITY_ERROR", 2.0, min=0.0),
        enable_lidar=_b("ENABLE_LIDAR", False),
        enable_camera=_b("ENABLE_CAMERA", False),
        use_ros2=_b("USE_ROS2", True),
        lidar_topic=os.getenv("LIDAR_TOPIC", "/os_cloud_node/points"),
        camera_topic=os.getenv("CAMERA_TOPIC", "/camera/color/image_raw"),
        camera_info_topic=os.getenv("CAMERA_INFO_TOPIC", "/camera/color/camera_info"),
        lidar_ip=os.getenv("LIDAR_IP", "192.168.1.120"),
        host_ip=os.getenv("HOST_IP", "192.168.1.10"),
        lidar_port=_i("LIDAR_PORT", 7502, min=0),
        imu_port=_i("IMU_PORT", 7503, min=0),
        camera_device=_optional_i("CAMERA_DEVICE"),
    )


__all__ = ["RobotConfig", "load"]
