"""
Robot Orchestrator
===================

This module defines a `RobotOrchestrator` class that ties together all
the core modules developed for the humanoid robot control system.  It
instantiates each manager (hardware synchronisation, joint
synchronisation, battery management, thermal management, memory
management, consistency verification, concurrency management and task
mapping) and runs a deterministic control loop at a fixed frequency.

The orchestrator is responsible for the following sequence in each
control cycle:

1. **Read sensor data:** Retrieve the latest sensor state from the
   hardware via the `HardwareSynchronizer`.  In this mock
   implementation, the hardware synchroniser also applies the
   commands queued from the previous cycle.
2. **Update managers:** Update the battery and thermal managers with
   the new sensor data and check whether the system remains safe.  If
   safety checks fail, the orchestrator may trigger an emergency stop.
3. **Run scheduled tasks:** Process any tasks in the execution stack.
   Tasks may schedule new high‑level tasks or compute joint commands.
4. **Map high‑level tasks:** Convert any high‑level tasks into joint
   instructions using the task–hardware mapper and kinematics model.
5. **Synchronise joints:** Send the computed joint commands to the
   `JointSynchronizer` which applies velocity/torque limits and
   dispatches them via the `HardwareSynchronizer`.
6. **Verify consistency:** Use the `ConsistencyVerifier` to ensure
   the returned state is consistent (e.g. monotonic timestamps,
   numeric positions).  Log or handle any inconsistencies.
7. **Maintain timing:** Ensure that the loop runs at the configured
   frequency by sleeping for the remainder of the cycle time.

This skeleton implementation demonstrates the interactions between
modules without requiring real hardware or sensors.  A `MockHardware`
interface is used internally.  Future work should extend this
orchestrator to include proper error handling, logging, telemetry
integration and external task submission interfaces.
"""

from __future__ import annotations

import contextlib
import time
from typing import Any

from robot_hw.control.joint_synchronization import JointCommand, JointSynchronizer
from robot_hw.control.locomotion_controller import LocomotionController
from robot_hw.core.communication import CommunicationInterface
from robot_hw.core.concurrency_management import ConcurrencyManager
from robot_hw.core.consistency_verification import ConsistencyVerifier
from robot_hw.core.environment_adapter import EnvironmentAdapter
from robot_hw.core.execution_stack import ExecutionStack
from robot_hw.core.fault_detection import FaultDetector
from robot_hw.core.hardware_synchronization import HardwareSynchronizer
from robot_hw.core.hazard_manager import HazardManager
from robot_hw.core.memory_management import MemoryManager
from robot_hw.perception.sensor_processing import SensorProcessor
from robot_hw.perception.time_sync import TimeSync  # type: ignore
from robot_hw.planning.mission_planner import MissionPlanner
from robot_hw.planning.navigation_manager import NavigationManager
from robot_hw.planning.task_hardware_mapping import (
    JointInstruction,
    KinematicsModel,
    Task,
    TaskHardwareMapper,
)
from robot_hw.power.battery_management import BatteryManager, BatteryState
from robot_hw.power.thermal_management import ThermalManager

# Stage B additions: sensor processing and hazard management
from robot_hw.robot_config import load as load_config
from robot_hw.telemetry.runtime_metrics import (
    build_health_events,
    build_runtime_metrics,
    build_structured_log,
)


class DummyKinematicsModel:
    """A minimal kinematics model used for demonstration.

    The model simply passes through joint commands if provided via
    ``joint_positions`` in the target parameters.  Otherwise it
    returns a fixed mapping for demonstration purposes.  Real
    implementations should provide inverse kinematics solving.
    """

    def inverse_kinematics(self, target: dict[str, Any], current: dict[str, Any]) -> dict[str, Any]:
        if "joint_positions" in target:
            return {jid: {"position": pos} for jid, pos in target["joint_positions"].items()}
        # Fallback: pretend we have two joints with positions derived from target
        # This is purely illustrative; a real IK solver would compute
        # joint angles from Cartesian coordinates.
        x = float(target.get("x", 0.0))
        y = float(target.get("y", 0.0))
        return {"joint1": {"position": x}, "joint2": {"position": y}}


class RobotOrchestrator:
    """Main orchestrator for the humanoid robot control system.

    Parameters
    ----------
    cycle_time : float
        Duration of each control cycle in seconds (e.g. 0.01 for 100 Hz).
    total_memory_bytes : int
        Total memory to manage via the `MemoryManager`.
    battery_capacity_wh : float
        Rated battery capacity in watt‑hours for runtime estimation.
    max_temperature : float
        Maximum allowed temperature (°C) for the thermal manager.
    max_velocity : float
        Default maximum joint velocity.
    max_torque : float
        Default maximum joint torque.
    """

    def __init__(
        self,
        cycle_time: float = 0.01,
        total_memory_bytes: int = 1024 * 1024,
        battery_capacity_wh: float = 100.0,
        max_temperature: float = 60.0,
        max_velocity: float = 1.0,
        max_torque: float = 1.0,
    ) -> None:
        self.cycle_time = cycle_time
        # Instantiate hardware synchroniser with a mock interface
        self.hardware = HardwareSynchronizer(HardwareSynchronizer.MockHardwareInterface())
        self.joint_sync = JointSynchronizer(
            self.hardware, max_velocity=max_velocity, max_torque=max_torque, cycle_time=cycle_time
        )
        self.battery_manager = BatteryManager(capacity_wh=battery_capacity_wh)
        self.thermal_manager = ThermalManager(max_temp=max_temperature)
        self.memory_manager = MemoryManager(total_bytes=total_memory_bytes)
        self.execution_stack = ExecutionStack()
        self.consistency_verifier = ConsistencyVerifier()
        self.concurrency_manager = ConcurrencyManager()
        # Use a dummy kinematics model; can be replaced with a real one
        self.kinematics_model: KinematicsModel = DummyKinematicsModel()  # type: ignore[assignment]
        self.task_mapper = TaskHardwareMapper(self.kinematics_model)
        # Pending high‑level tasks to be converted into joint commands
        self.pending_tasks: list[Task] = []
        # Last sensor state
        self.current_state: dict[str, Any] = {}
        self.runtime_metrics_history: list[dict[str, Any]] = []
        self.structured_logs: list[dict[str, Any]] = []
        self.health_events: list[dict[str, Any]] = []

        # ------------------------------------------------------------------
        # Stage B: instantiate sensor processor and hazard manager
        #
        # Load configuration once to initialise perception and hazard modules.
        # These modules use environment profiles, calibration factors and
        # safety thresholds defined in :mod:`robot_config`.  If
        # environment variables are not set, sensible defaults are used.
        self.config = load_config()
        self.sensor_processor = SensorProcessor(self.config)
        self.hazard_manager = HazardManager(self.config)
        # Stage C: instantiate navigation manager and locomotion controller
        self.navigation_manager = NavigationManager(self.config)
        self.locomotion_controller = LocomotionController(self.config)
        # Stage E: instantiate fault detector to monitor sensors and actuators
        self.fault_detector = FaultDetector(self.config)
        # Keep track of last joint commands sent to actuators for fault correlation
        self._last_joint_commands: dict[str, JointCommand] = {}
        # Stage F: environment adapter
        self.environment_adapter = EnvironmentAdapter(self.config)
        # Apply environment‑specific configuration overrides
        with contextlib.suppress(Exception):
            self.environment_adapter.configure()
        self.environment_overrides = self.environment_adapter.get_current_limits()
        # Adjust thermal manager based on environment thermal limit
        env_thermal = self.environment_overrides.get("thermal_limit_c")
        if env_thermal is not None:
            self.thermal_manager.max_temp = float(env_thermal)
        # Apply sensor noise and enabled sensor types overrides to the sensor processor.
        try:
            noise_override = self.environment_overrides.get("sensor_noise_std")
            if noise_override is not None:
                # SensorProcessor stores noise standard deviation on an internal attribute
                self.sensor_processor._noise_std = float(noise_override)  # type: ignore[attr-defined]
            types_override = self.environment_overrides.get("environment_sensor_types")
            if types_override:
                self.sensor_processor._sensor_types = tuple(types_override)  # type: ignore[attr-defined]
        except Exception:
            # Silently ignore if overrides cannot be applied
            pass
        # Apply locomotion mode override to the locomotion controller
        try:
            loc_override = self.environment_overrides.get("locomotion_mode")
            if loc_override:
                # LocomotionController stores mode in _mode attribute; ensure uppercase
                self.locomotion_controller._mode = str(loc_override).upper()  # type: ignore[attr-defined]
        except Exception:
            pass
        # Record the current environment profile to detect changes during runtime
        self._current_env_profile = self.config.environment_profile
        # Stage G: mission planner and communication interface
        self.mission_planner = MissionPlanner(
            self.config, self.battery_manager, self.thermal_manager, self.hazard_manager
        )
        self.communication = CommunicationInterface(self.config)
        # Establish communication (no‑op in skeleton)
        with contextlib.suppress(Exception):
            self.communication.connect()

        # ------------------------------------------------------------------
        # Stage 1: initialise sensor ingestion and time synchronisation
        #
        # If LiDAR or camera ingestion is enabled in the configuration,
        # instantiate the appropriate sensor I/O classes (ROS2 or direct)
        # and a TimeSync helper to associate frames.  These objects
        # operate asynchronously and provide the latest frames when
        # queried.  They are optional and do not block the main loop.
        self.lidar_io: Any | None = None
        self.camera_io: Any | None = None
        self.sensor_io: Any | None = None
        self.time_sync: TimeSync | None = None
        try:
            if self.config.enable_lidar or self.config.enable_camera:
                # Always instantiate the TimeSync helper; the default
                # max_offset of 0.05 seconds can be overridden via
                # environment variables in the future.
                self.time_sync = TimeSync()
                if self.config.use_ros2:
                    try:
                        from perception.sensor_io_ros2 import Ros2SensorIO  # type: ignore

                        self.sensor_io = Ros2SensorIO(
                            lidar_topic=self.config.lidar_topic,
                            camera_topic=self.config.camera_topic,
                            camera_info_topic=self.config.camera_info_topic,
                        )
                        # Start the ROS2 I/O thread immediately
                        self.sensor_io.start()
                    except Exception:
                        # Failed to start ROS2 ingestion; fallback to no sensor I/O
                        self.sensor_io = None
                else:
                    # Direct SDK ingestion: instantiate separate I/O objects
                    from perception.sensor_io_direct import (  # type: ignore
                        OusterSDKSensorIO,
                        UvcCameraSensorIO,
                    )

                    if self.config.enable_lidar:
                        try:
                            self.lidar_io = OusterSDKSensorIO(
                                host_ip=self.config.host_ip,
                                lidar_ip=self.config.lidar_ip,
                                lidar_port=self.config.lidar_port,
                                imu_port=self.config.imu_port,
                            )
                            self.lidar_io.start()
                        except Exception:
                            self.lidar_io = None
                    if self.config.enable_camera:
                        try:
                            self.camera_io = UvcCameraSensorIO(device=self.config.camera_device)
                            self.camera_io.start()
                        except Exception:
                            self.camera_io = None
        except Exception:
            # Ignore any ingestion initialisation errors
            self.sensor_io = None
            self.lidar_io = None
            self.camera_io = None
            self.time_sync = None

    def _normalise_state(self, sensor_data: dict[str, Any]) -> dict[str, Any]:
        """Convert raw sensor data into a unified state structure.

        The hardware synchroniser returns a dictionary keyed by joint IDs
        with sub‑fields ``position``, ``velocity`` and ``torque``.  It
        also contains a top‑level ``timestamp`` entry.  This helper
        extracts these fields into separate dictionaries for positions,
        velocities and torques, and preserves the timestamp.  Missing
        values are left as ``None``.
        """
        positions: dict[str, Any] = {}
        velocities: dict[str, Any] = {}
        torques: dict[str, Any] = {}
        timestamp = sensor_data.get("timestamp")
        for joint_id, info in sensor_data.items():
            # Skip the timestamp and any non‑joint keys (e.g. hazard keys)
            if joint_id in {"timestamp", "proximity", "hazards", "pedestrian"}:
                continue
            if isinstance(info, dict):
                positions[joint_id] = info.get("position")
                velocities[joint_id] = info.get("velocity")
                torques[joint_id] = info.get("torque")
        state: dict[str, Any] = {"timestamp": timestamp, "positions": positions}
        if velocities:
            state["velocities"] = velocities
        if torques:
            state["torques"] = torques
        return state

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def submit_task(self, task: Task) -> None:
        """Queue a high‑level task for execution in the next control cycle.

        This method is thread‑safe and may be called from external
        components (e.g. a planner or remote operator).  Tasks are
        stored in ``pending_tasks`` and processed at the start of
        the next cycle.
        """
        with self.concurrency_manager.acquire("tasks"):
            self.pending_tasks.append(task)

    def submit_goal(self, goal: dict[str, Any]) -> None:
        """Submit a high‑level mission goal to the mission planner.

        The goal is forwarded to the mission planner and, when it includes
        planar ``x``/``y`` coordinates, also becomes the active navigation
        target for map-based planning and replanning.

        Parameters
        ----------
        goal : dict[str, Any]
            A goal specification, e.g. ``{"x": 1.0, "y": 2.0}``.
        """
        self.mission_planner.add_goal(goal)
        if isinstance(goal, dict) and "x" in goal and "y" in goal:
            with contextlib.suppress(Exception):
                self.navigation_manager.set_goal((float(goal["x"]), float(goal["y"])))

    def run(self, num_cycles: int = 100) -> None:
        """Run the control loop for a fixed number of cycles.

        In each cycle the orchestrator reads sensors, updates managers,
        processes tasks, sends joint commands and verifies state
        consistency.  Timing is maintained using ``time.sleep`` to
        approximate the desired cycle duration.  Note that this
        demonstration does not handle missed deadlines or jitter; it
        simply sleeps for the remaining time.

        Parameters
        ----------
        num_cycles : int
            Number of iterations to run.  In a production system this
            loop would run indefinitely.
        """
        for cycle in range(num_cycles):
            # ------------------------------------------------------------------
            # Detect environment profile changes
            # ------------------------------------------------------------------
            try:
                new_config = load_config()
            except Exception:
                new_config = None
            if (
                new_config is not None
                and new_config.environment_profile != self._current_env_profile
            ):
                # Update configuration and reconfigure environment
                self.config = new_config
                self._current_env_profile = new_config.environment_profile
                # Recreate adapter and apply overrides
                self.environment_adapter = EnvironmentAdapter(self.config)
                with contextlib.suppress(Exception):
                    self.environment_adapter.configure()
                self.environment_overrides = self.environment_adapter.get_current_limits()
                env_thermal = self.environment_overrides.get("thermal_limit_c")
                if env_thermal is not None:
                    self.thermal_manager.max_temp = float(env_thermal)
                # Apply sensor overrides to the newly created sensor processor
                try:
                    noise_override = self.environment_overrides.get("sensor_noise_std")
                    if noise_override is not None:
                        self.sensor_processor._noise_std = float(noise_override)  # type: ignore[attr-defined]
                    types_override = self.environment_overrides.get("environment_sensor_types")
                    if types_override:
                        self.sensor_processor._sensor_types = tuple(types_override)  # type: ignore[attr-defined]
                except Exception:
                    pass
                # Apply locomotion mode override to the locomotion controller
                try:
                    loc_override = self.environment_overrides.get("locomotion_mode")
                    if loc_override:
                        self.locomotion_controller._mode = str(loc_override).upper()  # type: ignore[attr-defined]
                except Exception:
                    pass
                # Modules that depend on configuration should be re-instantiated
                self.sensor_processor = SensorProcessor(self.config)
                self.hazard_manager = HazardManager(self.config)
                self.navigation_manager = NavigationManager(self.config)
                self.locomotion_controller = LocomotionController(self.config)
                # Reconfigure fault detector with updated config
                self.fault_detector = FaultDetector(self.config)
                # Recreate mission planner and communication with updated references
                self.mission_planner = MissionPlanner(
                    self.config, self.battery_manager, self.thermal_manager, self.hazard_manager
                )
                # Communication uses config keys; we reconnect
                self.communication = CommunicationInterface(self.config)
                with contextlib.suppress(Exception):
                    self.communication.connect()
            # ------------------------------------------------------------------
            # Stage G: handle incoming remote commands
            # ------------------------------------------------------------------
            try:
                cmd = self.communication.receive_commands()
            except Exception:
                cmd = {}
            if isinstance(cmd, dict) and cmd:
                # Recognise simple command patterns
                if "add_goal" in cmd:
                    goal = cmd.get("add_goal")
                    if isinstance(goal, dict):
                        self.submit_goal(goal)
                if "task" in cmd:
                    tdef = cmd.get("task")
                    if isinstance(tdef, dict):
                        tid = tdef.get("id")
                        params = tdef.get("parameters", {})
                        if tid and isinstance(params, dict):
                            with contextlib.suppress(Exception):
                                self.submit_task(Task(id=tid, parameters=params))
            start = time.perf_counter()
            # ------------------------------------------------------------------
            # 1. Read sensors (and apply any commands from previous cycle)
            # ------------------------------------------------------------------
            with self.concurrency_manager.acquire("hardware"):
                # Fetch actuator and basic sensor data from hardware
                sensor_data = self.hardware.sync({})
                # Normalise into unified state (positions/velocities/torques)
                self.current_state = self._normalise_state(sensor_data)
                # ------------------------------------------------------------------
                # Stage 1: augment sensor state with LiDAR and camera frames if
                # enabled.  We query the sensor I/O classes for the most
                # recent frames and perform time synchronisation.  The
                # augmented state is passed to the sensor processor for
                # fusion.  If ingestion is not initialised, fall back
                # to the original state.
                augmented_state = dict(self.current_state)
                try:
                    lidar_frame = None  # type: ignore
                    camera_frame = None  # type: ignore
                    if self.time_sync:
                        if self.config.use_ros2 and self.sensor_io is not None:
                            try:
                                lidar_frame = self.sensor_io.get_latest_lidar_frame()
                                camera_frame = self.sensor_io.get_latest_camera_frame()
                            except Exception:
                                lidar_frame = None
                                camera_frame = None
                        else:
                            # Direct ingestion
                            if self.lidar_io is not None:
                                try:
                                    lidar_frame = self.lidar_io.get_latest_lidar_frame()
                                except Exception:
                                    lidar_frame = None
                            if self.camera_io is not None:
                                try:
                                    camera_frame = self.camera_io.get_latest_camera_frame()
                                except Exception:
                                    camera_frame = None
                        # Feed camera frames into the time sync buffer
                        if camera_frame is not None:
                            with contextlib.suppress(Exception):
                                self.time_sync.add_camera_frame(camera_frame)
                        matched_camera = None
                        if lidar_frame is not None:
                            try:
                                res = self.time_sync.match(lidar_frame)
                                matched_camera = res.camera_frame
                            except Exception:
                                matched_camera = None
                        if lidar_frame is not None:
                            augmented_state["lidar_frame"] = lidar_frame
                        if matched_camera is not None:
                            augmented_state["camera_frame"] = matched_camera
                        elif camera_frame is not None:
                            # Use the latest camera frame even if not synchronised
                            augmented_state["camera_frame"] = camera_frame
                    # ------------------------------------------------------------------
                    # Stage B: sensor fusion with augmented state
                    fused = self.sensor_processor.fuse_sensors(augmented_state)
                except Exception:
                    # On failure, fall back to fusing the basic state
                    fused = self.sensor_processor.fuse_sensors(self.current_state)
                # Merge fused keys into the current state
                for k, v in fused.items():
                    if k not in self.current_state:
                        self.current_state[k] = v
                # ------------------------------------------------------------------
                # Stage E: fault detection
                # Evaluate sensor health using the fault detector and record any faults
                with contextlib.suppress(Exception):
                    self.fault_detector.update(self.current_state, self._last_joint_commands)
                fault_list = (
                    self.fault_detector.get_faults() if self.fault_detector.has_fault() else []
                )
                # ------------------------------------------------------------------
                # Stage B: hazard detection (integrate faults)
                signals = {}
                if "proximity" in self.current_state:
                    signals["proximity"] = self.current_state["proximity"]
                hazards_dict = self.current_state.get("hazards")
                if isinstance(hazards_dict, dict):
                    for hk, hv in hazards_dict.items():
                        signals[hk] = hv
                if "pedestrian" in self.current_state:
                    signals["pedestrian"] = self.current_state["pedestrian"]
                if fault_list:
                    signals["faults"] = list(fault_list)
                self.hazard_manager.update(signals)
                self.current_state["hazard_flags"] = self.hazard_manager.current_hazards()

                # Determine overall hazard risk.  High‑risk hazards stop
                # all task execution; moderate risk triggers throttling.
                hazard_risk: str = "none"
                try:
                    hazards = self.hazard_manager.current_hazards()
                    for info in hazards.values():
                        risk = None
                        # Each hazard entry may include a risk_level field
                        if isinstance(info, dict):
                            risk = info.get("risk_level")
                        # Default to high risk if unknown
                        if risk is None or str(risk).lower() == "high":
                            hazard_risk = "high"
                            break
                        elif str(risk).lower() == "moderate" and hazard_risk != "high":
                            hazard_risk = "moderate"
                    # If no hazards, hazard_risk remains 'none'
                except Exception:
                    hazard_risk = "high"
                # Set flags used later in task scheduling
                hazard_stop = hazard_risk == "high"
                hazard_throttle = hazard_risk == "moderate"

                # ------------------------------------------------------------------
                # Stage G: mission planning based on current state
                #
                # Generate high‑level tasks from queued goals if conditions are safe.
                try:
                    planned = self.mission_planner.plan_tasks(self.current_state)
                except Exception:
                    planned = []
                for t in planned:
                    self.submit_task(t)

            # ------------------------------------------------------------------
            # Stage C: navigation and locomotion updates
            #
            # Compute or refine the navigation plan based on the fused state,
            # store planning artefacts on the shared state, and derive an
            # executable locomotion command from the local plan.
            _plan = self.navigation_manager.update(self.current_state)
            navigation_report = self.navigation_manager.get_latest_report()
            if navigation_report:
                self.current_state["navigation"] = navigation_report
                nav2_payload = navigation_report.get("nav2")
                if isinstance(nav2_payload, dict):
                    self.current_state["nav2_costmap"] = nav2_payload
                map_summary = navigation_report.get("map")
                if isinstance(map_summary, dict):
                    self.current_state["map"] = map_summary
                if isinstance(navigation_report.get("global_plan"), list):
                    self.current_state["global_plan"] = navigation_report["global_plan"]
                if isinstance(navigation_report.get("local_plan"), list):
                    self.current_state["local_plan"] = navigation_report["local_plan"]
            self.locomotion_controller.update_state(self.current_state)
            desired_velocity = self.navigation_manager.get_latest_velocity_command()
            locomotion_commands = self.locomotion_controller.compute_commands(desired_velocity)
            if locomotion_commands:
                self.current_state["locomotion_commands"] = locomotion_commands
                self.submit_task(
                    Task(
                        id="follow_navigation_velocity",
                        parameters={"target_velocity": desired_velocity},
                    )
                )
            # ------------------------------------------------------------------
            # 2. Update battery and thermal managers
            # ------------------------------------------------------------------
            # Extract battery and temperature measurements from the
            # sensor state if available.  In this mock, these values
            # are absent; instead we synthesise simple data.
            timestamp = self.current_state.get("timestamp", cycle * self.cycle_time)
            # Synthesis: battery voltage decays slowly, current is constant
            battery_state = BatteryState(
                voltage=50.0 - 0.01 * cycle,
                current=10.0,
                temperature=25.0,
                soc=max(1.0 - 0.001 * cycle, 0.0),
                health=1.0,
                timestamp=timestamp,
            )
            self.battery_manager.update(battery_state)
            # Synthesis: temperatures increase slightly with cycle
            # Synthesis of temperatures: one entry per joint (based on positions)
            temp_data = {
                joint: 30.0 + 0.1 * cycle for joint in self.current_state.get("positions", {})
            }
            self.thermal_manager.update(temp_data)
            # ------------------------------------------------------------------
            # 3. Process scheduled tasks
            # ------------------------------------------------------------------
            self.execution_stack.step()
            # ------------------------------------------------------------------
            # 4. Handle high‑level tasks and map them to joint commands
            # ------------------------------------------------------------------
            desired_joint_commands: dict[str, JointCommand] = {}
            # Determine whether to process tasks or halt based on hazard risk
            # hazard_stop and hazard_throttle are computed earlier in the cycle
            if hazard_stop:
                # High‑risk hazards require immediate halt of task execution
                print(f"[Cycle {cycle}] Emergency stop: high‑risk hazard detected; skipping tasks")
            else:
                with self.concurrency_manager.acquire("tasks"):
                    tasks_to_process = self.pending_tasks
                    self.pending_tasks = []
                for task in tasks_to_process:
                    # Map high‑level tasks into joint‑level instructions
                    instructions: list[JointInstruction]
                    try:
                        instructions = self.task_mapper.map_task(task, self.current_state)
                    except Exception:
                        # Invalid task; skip and log
                        continue
                    # Estimate energy consumption for these instructions
                    try:
                        energy_req = self.battery_manager.estimate_task_energy(
                            instructions, self.current_state
                        )
                    except Exception:
                        energy_req = 0.0
                    # Decide whether to defer based on energy budget
                    if self.battery_manager.should_defer_task(energy_req):
                        print(
                            f"[Cycle {cycle}] Task '{getattr(task, 'id', '')}' deferred due to low energy budget"
                        )
                        continue
                    # Estimate thermal load
                    try:
                        thermal_load = self.thermal_manager.estimate_task_thermal_load(
                            instructions, self.current_state
                        )
                    except Exception:
                        thermal_load = 0.0
                    # Determine if throttling is needed due to thermal or hazard risk
                    throttle_due_to_thermal = self.thermal_manager.should_throttle(thermal_load)
                    throttle = throttle_due_to_thermal or hazard_throttle
                    # Map instructions into joint commands, applying throttling if required
                    for inst in instructions:
                        cmd_dict = inst.command.copy() if isinstance(inst.command, dict) else {}
                        if throttle:
                            # Reduce velocity and torque commands by 50 % to mitigate risk
                            if cmd_dict.get("velocity") is not None:
                                with contextlib.suppress(Exception):
                                    cmd_dict["velocity"] = 0.5 * float(cmd_dict["velocity"])
                            if cmd_dict.get("torque") is not None:
                                with contextlib.suppress(Exception):
                                    cmd_dict["torque"] = 0.5 * float(cmd_dict["torque"])
                        jc = JointCommand(
                            position=cmd_dict.get("position"),
                            velocity=cmd_dict.get("velocity"),
                            torque=cmd_dict.get("torque"),
                        )
                        desired_joint_commands[inst.joint_id] = jc
            # ------------------------------------------------------------------
            # 5. Apply joint commands via joint synchroniser
            # ------------------------------------------------------------------
            if desired_joint_commands:
                # Extract current positions and velocities for the joints we will command
                preserved_state = {
                    key: self.current_state.get(key)
                    for key in (
                        "fusion",
                        "hazards",
                        "proximity",
                        "pedestrian",
                        "hazard_flags",
                        "navigation",
                        "map",
                        "nav2_costmap",
                        "global_plan",
                        "local_plan",
                        "locomotion_commands",
                    )
                    if key in self.current_state
                }
                joint_state_only = {
                    jid: {
                        "position": self.current_state.get("positions", {}).get(jid),
                        "velocity": self.current_state.get("velocities", {}).get(jid),
                    }
                    for jid in desired_joint_commands
                }
                with self.concurrency_manager.acquire("hardware"):
                    sensor_data = self.joint_sync.synchronize(
                        desired_joint_commands, joint_state_only
                    )
                self.current_state = self._normalise_state(sensor_data)
                self.current_state.update(preserved_state)
                # Record the commands sent for fault detection in the next cycle
                self._last_joint_commands = desired_joint_commands.copy()
            # ------------------------------------------------------------------
            # 6. Verify state consistency and check safety
            # ------------------------------------------------------------------
            consistent = self.consistency_verifier.verify(self.current_state)
            if not consistent:
                # In production, log errors and potentially trigger an emergency stop
                # Here we simply print the errors for demonstration purposes
                print(f"[Cycle {cycle}] Consistency errors: {self.consistency_verifier.errors}")
            # Check battery and thermal safety
            if not self.battery_manager.is_ok():
                print(f"[Cycle {cycle}] Warning: battery low or temperature out of range")
            if not self.thermal_manager.is_within_limits():
                print(f"[Cycle {cycle}] Warning: thermal limit exceeded; cooling recommended")
                # Compute cooling duties and notify any registered callbacks
                duties = self.thermal_manager.recommended_cooling()
                # In this demonstration, simply print the duty cycles
                print(f"[Cycle {cycle}] Cooling duties: {duties}")
            # ------------------------------------------------------------------
            # Stage G: send telemetry
            #
            try:
                # Compose telemetry dictionary with key state variables and runtime metrics.
                faults = self.fault_detector.get_faults() if self.fault_detector.has_fault() else []
                hazards = self.hazard_manager.current_hazards()
                metrics = build_runtime_metrics(
                    cycle=cycle,
                    loop_duration_s=time.perf_counter() - start,
                    state=self.current_state,
                    telemetry_count=len(getattr(self.communication, "_telemetry_log", [])),
                    fault_count=len(faults),
                    hazard_count=len(hazards),
                )
                events = build_health_events(
                    cycle=cycle,
                    timestamp=self.current_state.get("timestamp"),
                    battery_ok=self.battery_manager.is_ok(),
                    thermal_ok=self.thermal_manager.is_within_limits(),
                    consistency_ok=consistent,
                    faults=faults,
                    hazards=hazards,
                )
                telemetry = {
                    "timestamp": self.current_state.get("timestamp"),
                    "battery_soc": (
                        getattr(self.battery_manager.state, "soc", None)
                        if self.battery_manager.state
                        else None
                    ),
                    "battery_voltage": (
                        getattr(self.battery_manager.state, "voltage", None)
                        if self.battery_manager.state
                        else None
                    ),
                    "battery_current": (
                        getattr(self.battery_manager.state, "current", None)
                        if self.battery_manager.state
                        else None
                    ),
                    "thermal_max": (
                        max(self.thermal_manager.current_temps.values())
                        if self.thermal_manager.current_temps
                        else None
                    ),
                    "hazards": hazards,
                    "faults": faults,
                    "runtime_metrics": metrics,
                    "health_events": [event.as_dict() for event in events],
                }
                self.communication.send_telemetry(telemetry)
                self.runtime_metrics_history.append(metrics)
                self.health_events.extend(event.as_dict() for event in events)
                self.structured_logs.append(
                    build_structured_log(
                        cycle=cycle,
                        state=self.current_state,
                        metrics=metrics,
                        events=events,
                    )
                )
            except Exception:
                pass
            # ------------------------------------------------------------------
            # 7. Sleep to maintain loop timing
            # ------------------------------------------------------------------
            elapsed = time.perf_counter() - start
            remaining = self.cycle_time - elapsed
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    # Example usage: run the orchestrator for a few cycles and submit a simple task
    orchestrator = RobotOrchestrator(
        cycle_time=0.01,
        total_memory_bytes=1024 * 1024,
        battery_capacity_wh=100.0,
        max_temperature=60.0,
        max_velocity=2.0,
        max_torque=1.5,
    )
    # Submit a high‑level task to move joints to specific positions
    orchestrator.submit_task(
        Task(id="move_joints", parameters={"joint_positions": {"joint1": 0.5, "joint2": -0.3}})
    )
    orchestrator.run(num_cycles=20)
