"""
Robot Configuration Parser
==========================

This module centralises the parsing of environment variables for the
humanoid robot control system.  It is analogous to the trading bot’s
`config.py` but adapted to the robot domain.  Environment variables are
parsed into a typed `RobotConfig` dataclass using helper functions to
normalise booleans, lists, floats and fractions.  Defaults are
specified for each field to allow reasonable out‑of‑the‑box behaviour.

Secrets (e.g. `ROBOT_CONTROL_KEY`) are treated as opaque strings and
should be supplied via a secrets manager in production.  This parser
does not perform any network I/O; it merely reads from the local
environment.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

# ----------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------

def _b(name: str, default: bool = False) -> bool:
    """Parse an environment variable into a boolean.

    Accepted true values: ``1, true, t, yes, y, on`` (case-insensitive).
    Accepted false values: everything else.  If the variable is not
    present, returns ``default``.
    """
    val = os.getenv(name)
    if val is None:
        return default
    return str(val).strip().lower() in {"1", "true", "t", "yes", "y", "on"}


def _f(name: str, default: float, *, min: float | None = None, max: float | None = None) -> float:
    """Parse an environment variable into a float with optional bounds."""
    raw = os.getenv(name)
    try:
        val = float(raw) if raw not in (None, "") else default
    except Exception:
        val = default
    if min is not None and val < min:
        raise ValueError(f"{name}={val} is below minimum {min}")
    if max is not None and val > max:
        raise ValueError(f"{name}={val} is above maximum {max}")
    return val


def _csv(name: str, default: str) -> tuple[str, ...]:
    """Parse a comma‑separated list into a tuple of strings.

    Whitespace around entries is stripped; empty entries are ignored.
    """
    raw = os.getenv(name, default)
    return tuple(s.strip() for s in raw.split(",") if s.strip())


def _floats_csv(name: str, default: str) -> tuple[float, ...]:
    """Parse a comma‑separated list into a tuple of floats."""
    raw = os.getenv(name, default)
    out: list[float] = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            out.append(float(part))
        except Exception as err:
            raise ValueError(f"Invalid float in {name}: {part}") from err
    return tuple(out)


# ----------------------------------------------------------------------
# Data class for robot configuration
# ----------------------------------------------------------------------

@dataclass(frozen=True)
class RobotConfig:
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

    # ------------------------------------------------------------------
    # Extreme environment configuration
    # ------------------------------------------------------------------
    environment_profile: str
    """Identifier for the operating environment.

    Examples include ``"MINING"``, ``"UNDERWATER"``, ``"SPACE"``,
    ``"FORESTRY"`` and ``"GENERAL"``.  Different profiles allow the
    system to automatically select appropriate hardware, safety
    thresholds and control parameters for the environment in which
    the robot is deployed.
    """
    environment_hardware_ids: tuple[str, ...]
    """Additional hardware identifiers associated with the selected
    environment.

    These might include specialised sensors (e.g. radiation or gas
    detectors), actuators (thrusters, tracks) or tools (drills,
    cutting saws).  Empty by default.
    """
    sensor_calibration_factors: tuple[float, ...]
    """Calibration factors for primary sensors.

    The list corresponds to offsets or scale factors applied during
    sensor fusion.  Values are parsed from the
    ``SENSOR_CALIBRATION_FACTORS`` environment variable as a
    comma‑separated list of floats.  Defaults to an empty tuple.
    """
    safety_thresholds: tuple[float, ...]
    """Generic safety thresholds for hazards.

    A list of floats representing maximum allowable levels for
    domain‑specific hazards such as radiation (mSv), gas concentration
    (ppm), high voltage (V) or pressure (bar).  The exact mapping of
    indices to hazard types should be documented in the hazard
    manager.  Defaults to an empty tuple.
    """
    environment_power_budget_w: float
    """Per‑environment power budget in watts.

    This value allows the system to constrain energy usage when
    operating in different environments that impose tighter power
    limits (e.g. underwater operation may rely on limited battery
    capacity).  Defaults to the global power budget if unset.
    """
    environment_thermal_limit_c: float
    """Per‑environment maximum safe temperature in degrees Celsius.

    Used to override the global ``MAX_TEMPERATURE_C`` when the
    environment imposes stricter thermal constraints (e.g. vacuum of
    space or explosive atmospheres).  Defaults to the global maximum.
    """

    # ------------------------------------------------------------------
    # Additional tunables for advanced control and simulation fidelity
    # ------------------------------------------------------------------
    path_planning_algorithm: str
    """Algorithm used by the navigation manager.

    Accepted values include ``"A_STAR"`` for a grid‑based A* search,
    ``"RRT"`` for rapidly‑exploring random trees, and other names
    recognised by the navigation manager.  The default is ``"A_STAR"``.
    """

    locomotion_mode: str
    """Preferred locomotion mode for the robot.

    This value influences the locomotion controller.  Supported modes
    include ``"WHEELS"``, ``"LEGS"``, ``"TRACKS"``, and
    ``"THRUSTERS"``.  The environment adapter may override this
    setting based on the current environment profile.  Defaults to
    ``"WHEELS"``.
    """

    sensor_noise_std: float
    """Standard deviation of Gaussian noise to apply to sensor readings.

    A non‑negative float controlling the magnitude of synthetic noise
    injected into simulated sensor data.  This parameter can be
    adjusted per environment to reflect sensor accuracy.  Defaults to
    0.0 (no noise).
    """

    environment_sensor_types: tuple[str, ...]
    """Comma‑separated list of sensor types enabled in the current environment.

    Examples include ``"IMU"``, ``"LIDAR"``, ``"CAMERA"``,
    ``"SONAR"``, ``"GAS"`` and ``"RADIATION"``.  The sensor
    processor uses this list to determine which optional sensors to
    expect and fuse.  Defaults to an empty tuple.
    """

    max_position_error: float
    """Maximum allowable position error (difference) between commanded
    and measured joint positions before a fault is reported.

    This threshold is used by the fault detector to flag actuator or
    kinematic failures.  A positive float; defaults to 5.0 units.
    """

    max_velocity_error: float
    """Maximum allowable velocity error between commanded and measured
    joint velocities before a fault is reported.

    Similar to ``max_position_error`` but applied to velocities.
    Defaults to 2.0 units.
    """

    # ------------------------------------------------------------------
    # Sensor integration configuration (LiDAR and camera)
    # ------------------------------------------------------------------
    enable_lidar: bool
    """Enable LiDAR ingestion.  When True, the orchestrator will
    instantiate a LiDAR sensor I/O class (ROS2 or direct) and feed
    LiDAR frames into the perception pipeline.  Defaults to False.
    """

    enable_camera: bool
    """Enable camera ingestion.  When True, the orchestrator will
    instantiate a camera sensor I/O class and feed camera frames into
    the perception pipeline.  Defaults to False.
    """

    use_ros2: bool
    """Use ROS2 for sensor ingestion.  When True, the ROS2 bridge
    (``Ros2SensorIO``) is used to subscribe to LiDAR and camera
    topics.  When False, direct SDK ingestion (``OusterSDKSensorIO``
    and ``UvcCameraSensorIO``) is used.  Defaults to True.
    """

    lidar_topic: str
    """ROS2 topic name for LiDAR point clouds.  Ignored when
    ``use_ros2`` is False.  Defaults to ``'/os_cloud_node/points'``.
    """

    camera_topic: str
    """ROS2 topic name for camera images.  Defaults to
    ``'/camera/color/image_raw'``.  Ignored when ``use_ros2`` is False.
    """

    camera_info_topic: str
    """ROS2 topic name for camera intrinsic calibration data.  Defaults
    to ``'/camera/color/camera_info'``.  Ignored when ``use_ros2`` is
    False.
    """

    lidar_ip: str
    """IP address of the Ouster LiDAR sensor used for direct SDK
    ingestion.  Ignored when ``use_ros2`` is True.  Defaults to
    ``'192.168.1.120'``.
    """

    host_ip: str
    """IP address of the host machine used for direct LiDAR ingestion.
    Ignored when ``use_ros2`` is True.  Defaults to ``'192.168.1.10'``.
    """

    lidar_port: int
    """UDP port for LiDAR packets when using direct ingestion.  Defaults
    to 7502.
    """

    imu_port: int
    """UDP port for IMU packets when using direct ingestion.  Defaults
    to 7503.
    """

    camera_device: int | None
    """Device index for UVC camera capture when using direct ingestion.
    ``None`` instructs the system to attempt using a RealSense camera via
    pyrealsense2.  Defaults to 0.
    """


def load() -> RobotConfig:
    """Load the robot configuration from environment variables.

    Returns a frozen `RobotConfig` instance.  Raises ValueError on
    invalid numeric inputs (e.g. floats outside allowed ranges).
    """
    return RobotConfig(
        robot_environment=os.getenv("ROBOT_ENVIRONMENT", "production"),
        operation_mode=os.getenv("OPERATION_MODE", "operational"),
        simulator_enabled=_b("SIMULATOR_ENABLED", False),
        skip_hardware_check=_b("SKIP_HARDWARE_CHECK", False),
        joint_ids=_csv("JOINT_IDS", "joint1,joint2,joint3,joint4"),
        actuator_types=_csv("ACTUATOR_TYPES", "servo,servo,servo,servo"),
        max_torque_util=_f("MAX_TORQUE_UTIL", 1.0, min=0.0, max=1.0),
        default_torque_mult=_f("DEFAULT_TORQUE_MULT", 0.5, min=0.0),
        max_velocity_per_joint=_floats_csv("MAX_VELOCITY_PER_JOINT", "1.0,1.0,1.0,1.0"),
        max_torque_per_joint=_floats_csv("MAX_TORQUE_PER_JOINT", "1.5,1.5,1.5,1.5"),
        min_safety_margin=_f("MIN_SAFETY_MARGIN", 0.1, min=0.0, max=1.0),
        fault_temperature_buffer=_f("FAULT_TEMPERATURE_BUFFER", 5.0, min=0.0),
        max_heat_accumulation=_f("MAX_HEAT_ACCUMULATION", 0.8, min=0.0, max=1.0),
        max_cyclic_wear=_f("MAX_CYCLIC_WEAR", 0.7, min=0.0, max=1.0),
        emergency_stop_cooldown_s=_f("EMERGENCY_STOP_COOLDOWN_S", 5.0, min=0.0),
        battery_capacity_wh=_f("BATTERY_CAPACITY_WH", 100.0, min=0.0),
        low_battery_threshold=_f("LOW_BATTERY_THRESHOLD", 0.2, min=0.0, max=1.0),
        max_temperature_c=_f("MAX_TEMPERATURE_C", 60.0),
        cooling_hysteresis_c=_f("COOLING_HYSTERESIS_C", 5.0, min=0.0),
        total_memory_bytes=int(os.getenv("TOTAL_MEMORY_BYTES", "1048576")),
        lock_names=_csv("LOCK_NAMES", "hardware,memory,tasks"),
        enable_monitoring_ui=_b("ENABLE_MONITORING_UI", False),
        monitor_port=int(os.getenv("MONITOR_PORT", "8051")),
        robot_log_level=os.getenv("ROBOT_LOG_LEVEL", "INFO"),
        enable_aws_metrics=_b("ENABLE_AWS_METRICS", False),
        aws_region=os.getenv("AWS_REGION", "eu-central-1"),
        robot_control_key=os.getenv("ROBOT_CONTROL_KEY"),
        robot_telemetry_key=os.getenv("ROBOT_TELEMETRY_KEY"),
        cycle_time_s=_f("CYCLE_TIME_S", 0.01, min=1e-4),
        enable_power_saving_mode=_b("ENABLE_POWER_SAVING_MODE", False),
        power_budget_w=_f("POWER_BUDGET_W", 100.0, min=0.0),
        environment_profile=os.getenv("ENVIRONMENT_PROFILE", "GENERAL").upper(),
        environment_hardware_ids=_csv("ENVIRONMENT_HARDWARE_IDS", ""),
        sensor_calibration_factors=_floats_csv("SENSOR_CALIBRATION_FACTORS", ""),
        safety_thresholds=_floats_csv("SAFETY_THRESHOLDS", ""),
        environment_power_budget_w=_f(
            "ENVIRONMENT_POWER_BUDGET_W",
            _f("POWER_BUDGET_W", 100.0, min=0.0),
            min=0.0,
        ),
        environment_thermal_limit_c=_f(
            "ENVIRONMENT_THERMAL_LIMIT_C",
            _f("MAX_TEMPERATURE_C", 60.0),
        ),
        # Additional tunables for advanced features
        path_planning_algorithm=os.getenv("PATH_PLANNING_ALGORITHM", "A_STAR").upper(),
        locomotion_mode=os.getenv("LOCOMOTION_MODE", "WHEELS").upper(),
        sensor_noise_std=_f("SENSOR_NOISE_STD", 0.0, min=0.0),
        environment_sensor_types=_csv("ENVIRONMENT_SENSOR_TYPES", ""),
        max_position_error=_f("MAX_POSITION_ERROR", 5.0, min=0.0),
        max_velocity_error=_f("MAX_VELOCITY_ERROR", 2.0, min=0.0),

        # Sensor integration flags and parameters
        enable_lidar=_b("ENABLE_LIDAR", False),
        enable_camera=_b("ENABLE_CAMERA", False),
        use_ros2=_b("USE_ROS2", True),
        lidar_topic=os.getenv("LIDAR_TOPIC", "/os_cloud_node/points"),
        camera_topic=os.getenv("CAMERA_TOPIC", "/camera/color/image_raw"),
        camera_info_topic=os.getenv("CAMERA_INFO_TOPIC", "/camera/color/camera_info"),
        lidar_ip=os.getenv("LIDAR_IP", "192.168.1.120"),
        host_ip=os.getenv("HOST_IP", "192.168.1.10"),
        lidar_port=int(os.getenv("LIDAR_PORT", "7502")),
        imu_port=int(os.getenv("IMU_PORT", "7503")),
        camera_device=(
            None if (val := os.getenv("CAMERA_DEVICE")) is None or val.strip().lower() == "none" else int(val)
        ),
    )
