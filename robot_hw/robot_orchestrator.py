"""Robot orchestrator for the integrated hardware, planning, and telemetry stack.

This module defines :class:`RobotOrchestrator`, the top-level coordinator for
simulation and hardware-adjacent execution. The orchestrator is responsible for
reading the latest joint state, fusing optional perception inputs, updating
fault and hazard models, planning high-level work, mapping tasks to joint
commands, dispatching those commands to the joint synchronizer, and publishing
telemetry.

The implementation is intentionally deterministic and conservative. It keeps the
control loop structure simple, preserves the existing repository interfaces, and
provides stable behavior for both repeated smoke tests and multi-call
simulation entry points.
"""

from __future__ import annotations

import contextlib
import logging
import time
from typing import Any, Final

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
from robot_hw.robot_config import load as load_config
from robot_hw.telemetry.runtime_metrics import (
    build_health_events,
    build_runtime_metrics,
    build_structured_log,
)


_PRESERVED_STATE_KEYS: Final[tuple[str, ...]] = (
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


class DummyKinematicsModel:
    """Minimal kinematics model used when no robot-specific solver is available."""

    def inverse_kinematics(
        self,
        target: dict[str, Any],
        current: dict[str, Any],
    ) -> dict[str, Any]:
        del current
        if "joint_positions" in target:
            joint_positions = target["joint_positions"]
            if isinstance(joint_positions, dict):
                return {
                    str(joint_id): {"position": position}
                    for joint_id, position in joint_positions.items()
                }
        x = float(target.get("x", 0.0))
        y = float(target.get("y", 0.0))
        return {"joint1": {"position": x}, "joint2": {"position": y}}


class RobotOrchestrator:
    """Coordinate sensing, planning, actuation, and telemetry for the robot stack."""

    def __init__(
        self,
        cycle_time: float = 0.01,
        total_memory_bytes: int = 1024 * 1024,
        battery_capacity_wh: float = 100.0,
        max_temperature: float = 60.0,
        max_velocity: float = 1.0,
        max_torque: float = 1.0,
    ) -> None:
        self._logger = logging.getLogger(__name__)
        self.cycle_time = cycle_time
        self._cycle_index = 0

        self.hardware = HardwareSynchronizer(HardwareSynchronizer.MockHardwareInterface())
        self.joint_sync = JointSynchronizer(
            self.hardware,
            max_velocity=max_velocity,
            max_torque=max_torque,
            cycle_time=cycle_time,
        )
        self.battery_manager = BatteryManager(capacity_wh=battery_capacity_wh)
        self.thermal_manager = ThermalManager(max_temp=max_temperature)
        self.memory_manager = MemoryManager(total_bytes=total_memory_bytes)
        self.execution_stack = ExecutionStack()
        self.consistency_verifier = ConsistencyVerifier()
        self.concurrency_manager = ConcurrencyManager()
        self.kinematics_model: KinematicsModel = DummyKinematicsModel()  # type: ignore[assignment]
        self.task_mapper = TaskHardwareMapper(self.kinematics_model)

        self.pending_tasks: list[Task] = []
        self.current_state: dict[str, Any] = {}
        self.runtime_metrics_history: list[dict[str, Any]] = []
        self.structured_logs: list[dict[str, Any]] = []
        self.health_events: list[dict[str, Any]] = []
        self._last_joint_commands: dict[str, JointCommand] = {}

        self.config = load_config()
        self.sensor_processor = SensorProcessor(self.config)
        self.hazard_manager = HazardManager(self.config)
        self.navigation_manager = NavigationManager(self.config)
        self.locomotion_controller = LocomotionController(self.config)
        self.fault_detector = FaultDetector(self.config)
        self.environment_adapter = EnvironmentAdapter(self.config)
        self.environment_overrides: dict[str, Any] = {}
        self._current_env_profile = self.config.environment_profile

        self.communication = CommunicationInterface(self.config)
        with contextlib.suppress(Exception):
            self.communication.connect()

        self.lidar_io: Any | None = None
        self.camera_io: Any | None = None
        self.sensor_io: Any | None = None
        self.time_sync: TimeSync | None = None

        self._configure_environment()
        self._initialise_sensor_interfaces()
        self.mission_planner = MissionPlanner(
            self.config,
            self.battery_manager,
            self.thermal_manager,
            self.hazard_manager,
        )

    def _configure_environment(self) -> None:
        """Apply environment-derived overrides to runtime components."""
        with contextlib.suppress(Exception):
            self.environment_adapter.configure()
        self.environment_overrides = self.environment_adapter.get_current_limits()

        thermal_limit = self.environment_overrides.get("thermal_limit_c")
        if thermal_limit is not None:
            self.thermal_manager.max_temp = float(thermal_limit)

        self._apply_sensor_processor_overrides(self.environment_overrides)
        self._apply_locomotion_override(self.environment_overrides)

    def _initialise_sensor_interfaces(self) -> None:
        """Initialise optional LiDAR, camera, and time-sync interfaces."""
        self.lidar_io = None
        self.camera_io = None
        self.sensor_io = None
        self.time_sync = None

        try:
            if not (self.config.enable_lidar or self.config.enable_camera):
                return

            self.time_sync = TimeSync()
            if self.config.use_ros2:
                try:
                    from perception.sensor_io_ros2 import Ros2SensorIO  # type: ignore

                    self.sensor_io = Ros2SensorIO(
                        lidar_topic=self.config.lidar_topic,
                        camera_topic=self.config.camera_topic,
                        camera_info_topic=self.config.camera_info_topic,
                    )
                    self.sensor_io.start()
                except Exception as exc:
                    self._logger.debug("Failed to initialise ROS2 sensor I/O: %s", exc)
                    self.sensor_io = None
                return

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
                except Exception as exc:
                    self._logger.debug("Failed to initialise LiDAR sensor I/O: %s", exc)
                    self.lidar_io = None

            if self.config.enable_camera:
                try:
                    self.camera_io = UvcCameraSensorIO(device=self.config.camera_device)
                    self.camera_io.start()
                except Exception as exc:
                    self._logger.debug("Failed to initialise camera sensor I/O: %s", exc)
                    self.camera_io = None
        except Exception as exc:
            self._logger.debug("Sensor ingestion initialisation failed: %s", exc)
            self.lidar_io = None
            self.camera_io = None
            self.sensor_io = None
            self.time_sync = None

    def _reconfigure_if_needed(self) -> None:
        """Reload configuration when runtime settings have changed."""
        try:
            new_config = load_config()
        except Exception:
            return

        if new_config == self.config:
            return

        self.config = new_config
        self._current_env_profile = new_config.environment_profile
        self.sensor_processor = SensorProcessor(self.config)
        self.hazard_manager = HazardManager(self.config)
        self.navigation_manager = NavigationManager(self.config)
        self.locomotion_controller = LocomotionController(self.config)
        self.fault_detector = FaultDetector(self.config)
        self.environment_adapter = EnvironmentAdapter(self.config)
        self.communication = CommunicationInterface(self.config)
        with contextlib.suppress(Exception):
            self.communication.connect()

        self._configure_environment()
        self._initialise_sensor_interfaces()
        self.mission_planner = MissionPlanner(
            self.config,
            self.battery_manager,
            self.thermal_manager,
            self.hazard_manager,
        )

    def _apply_sensor_processor_overrides(self, overrides: dict[str, Any]) -> None:
        """Apply environment overrides to the sensor processor."""
        try:
            noise_override = overrides.get("sensor_noise_std")
            if noise_override is not None:
                self.sensor_processor._noise_std = float(noise_override)  # type: ignore[attr-defined]
            sensor_types = overrides.get("environment_sensor_types")
            if sensor_types:
                self.sensor_processor._sensor_types = tuple(sensor_types)  # type: ignore[attr-defined]
        except (TypeError, ValueError) as exc:
            self._logger.debug("Failed to apply sensor processor overrides: %s", exc)

    def _apply_locomotion_override(self, overrides: dict[str, Any]) -> None:
        """Apply environment overrides to the locomotion controller."""
        try:
            locomotion_mode = overrides.get("locomotion_mode")
            if locomotion_mode:
                self.locomotion_controller._mode = str(locomotion_mode).upper()  # type: ignore[attr-defined]
        except (TypeError, ValueError) as exc:
            self._logger.debug("Failed to apply locomotion override: %s", exc)

    def _normalise_state(self, sensor_data: dict[str, Any]) -> dict[str, Any]:
        """Convert raw hardware payloads into the orchestrator state shape."""
        positions: dict[str, Any] = {}
        velocities: dict[str, Any] = {}
        torques: dict[str, Any] = {}
        timestamp = sensor_data.get("timestamp")

        for key, value in sensor_data.items():
            if key in {"timestamp", "proximity", "hazards", "pedestrian"}:
                continue
            if isinstance(value, dict):
                positions[key] = value.get("position")
                velocities[key] = value.get("velocity")
                torques[key] = value.get("torque")

        state: dict[str, Any] = {"timestamp": timestamp, "positions": positions}
        if velocities:
            state["velocities"] = velocities
        if torques:
            state["torques"] = torques
        return state

    def _read_augmented_state(self) -> None:
        """Synchronise hardware, fuse optional sensor inputs, and update state."""
        with self.concurrency_manager.acquire("hardware"):
            sensor_data = self.hardware.sync({})

        self.current_state = self._normalise_state(sensor_data)
        augmented_state = dict(self.current_state)

        try:
            lidar_frame: Any | None = None
            camera_frame: Any | None = None

            if self.time_sync is not None:
                if self.config.use_ros2 and self.sensor_io is not None:
                    with contextlib.suppress(Exception):
                        lidar_frame = self.sensor_io.get_latest_lidar_frame()
                    with contextlib.suppress(Exception):
                        camera_frame = self.sensor_io.get_latest_camera_frame()
                else:
                    if self.lidar_io is not None:
                        with contextlib.suppress(Exception):
                            lidar_frame = self.lidar_io.get_latest_lidar_frame()
                    if self.camera_io is not None:
                        with contextlib.suppress(Exception):
                            camera_frame = self.camera_io.get_latest_camera_frame()

                if camera_frame is not None:
                    with contextlib.suppress(Exception):
                        self.time_sync.add_camera_frame(camera_frame)

                matched_camera = None
                if lidar_frame is not None:
                    with contextlib.suppress(Exception):
                        match = self.time_sync.match(lidar_frame)
                        matched_camera = match.camera_frame

                if lidar_frame is not None:
                    augmented_state["lidar_frame"] = lidar_frame
                if matched_camera is not None:
                    augmented_state["camera_frame"] = matched_camera
                elif camera_frame is not None:
                    augmented_state["camera_frame"] = camera_frame

            fused = self.sensor_processor.fuse_sensors(augmented_state)
        except Exception:
            fused = self.sensor_processor.fuse_sensors(self.current_state)

        for key, value in fused.items():
            if key not in {"timestamp", "positions", "velocities", "torques"}:
                self.current_state[key] = value

    def _handle_incoming_commands(self) -> None:
        """Consume one queued communication payload and translate it into work."""
        try:
            command = self.communication.receive_commands()
        except Exception:
            command = {}

        if not isinstance(command, dict) or not command:
            return

        goal = command.get("add_goal")
        if isinstance(goal, dict):
            self.submit_goal(goal)

        task_def = command.get("task")
        if isinstance(task_def, dict):
            task_id = task_def.get("id")
            parameters = task_def.get("parameters", {})
            if task_id and isinstance(parameters, dict):
                with contextlib.suppress(Exception):
                    self.submit_task(Task(id=str(task_id), parameters=parameters))

    def _update_faults_and_hazards(self) -> tuple[bool, bool, list[str], dict[str, Any]]:
        """Update safety monitors and return stop/throttle decisions."""
        with contextlib.suppress(Exception):
            self.fault_detector.update(self.current_state, self._last_joint_commands)

        faults = self.fault_detector.get_faults() if self.fault_detector.has_fault() else []

        signals: dict[str, Any] = {}
        if "proximity" in self.current_state:
            signals["proximity"] = self.current_state["proximity"]
        if "pedestrian" in self.current_state:
            signals["pedestrian"] = self.current_state["pedestrian"]

        hazards_in_state = self.current_state.get("hazards")
        if isinstance(hazards_in_state, dict):
            for hazard_key, hazard_value in hazards_in_state.items():
                signals[str(hazard_key)] = hazard_value

        if faults:
            signals["faults"] = list(faults)

        self.hazard_manager.update(signals)
        hazards = self.hazard_manager.current_hazards()
        self.current_state["hazard_flags"] = hazards

        hazard_risk = "none"
        for info in hazards.values():
            if not isinstance(info, dict):
                hazard_risk = "high"
                break
            risk_level = str(info.get("risk_level", "high")).lower()
            if risk_level == "high":
                hazard_risk = "high"
                break
            if risk_level == "moderate" and hazard_risk != "high":
                hazard_risk = "moderate"

        return hazard_risk == "high", hazard_risk == "moderate", list(faults), hazards

    def _update_planning(self) -> None:
        """Advance mission planning, navigation, and locomotion state."""
        try:
            planned_tasks = self.mission_planner.plan_tasks(self.current_state)
        except Exception:
            planned_tasks = []
        for task in planned_tasks:
            self.submit_task(task)

        self.navigation_manager.update(self.current_state)
        navigation_report = self.navigation_manager.get_latest_report()
        if isinstance(navigation_report, dict) and navigation_report:
            self.current_state["navigation"] = navigation_report

            nav2_payload = navigation_report.get("nav2")
            if isinstance(nav2_payload, dict):
                self.current_state["nav2_costmap"] = nav2_payload

            map_summary = navigation_report.get("map")
            if isinstance(map_summary, dict):
                self.current_state["map"] = map_summary

            global_plan = navigation_report.get("global_plan")
            if isinstance(global_plan, list):
                self.current_state["global_plan"] = global_plan

            local_plan = navigation_report.get("local_plan")
            if isinstance(local_plan, list):
                self.current_state["local_plan"] = local_plan

        self.locomotion_controller.update_state(self.current_state)
        desired_velocity = self.navigation_manager.get_latest_velocity_command()
        locomotion_commands = self.locomotion_controller.compute_commands(desired_velocity)
        if locomotion_commands:
            self.current_state["locomotion_commands"] = locomotion_commands
            if self._has_nonzero_velocity_command(desired_velocity):
                self.submit_task(
                    Task(
                        id="follow_navigation_velocity",
                        parameters={"target_velocity": desired_velocity},
                    )
                )

    def _update_power_and_thermal(self, cycle: int) -> None:
        """Refresh battery and temperature models for the current cycle."""
        timestamp = self.current_state.get("timestamp", cycle * self.cycle_time)
        battery_state = BatteryState(
            voltage=50.0 - 0.01 * cycle,
            current=10.0,
            temperature=25.0,
            soc=max(1.0 - 0.001 * cycle, 0.0),
            health=1.0,
            timestamp=timestamp,
        )
        self.battery_manager.update(battery_state)

        temps = {
            joint_id: 30.0 + 0.1 * cycle
            for joint_id in self.current_state.get("positions", {})
        }
        self.thermal_manager.update(temps)

    def _collect_joint_commands(
        self,
        cycle: int,
        hazard_stop: bool,
        hazard_throttle: bool,
    ) -> dict[str, JointCommand]:
        """Map queued tasks into bounded joint commands for this cycle."""
        desired_joint_commands: dict[str, JointCommand] = {}
        if hazard_stop:
            print(f"[Cycle {cycle}] Emergency stop: high-risk hazard detected; skipping tasks")
            return desired_joint_commands

        with self.concurrency_manager.acquire("tasks"):
            tasks_to_process = self.pending_tasks
            self.pending_tasks = []

        for task in tasks_to_process:
            try:
                instructions = self.task_mapper.map_task(task, self.current_state)
            except (KeyError, TypeError, ValueError) as exc:
                self._logger.debug(
                    "Skipping task %r because mapping failed: %s",
                    getattr(task, "id", ""),
                    exc,
                )
                continue

            energy_required = 0.0
            with contextlib.suppress(Exception):
                energy_required = self.battery_manager.estimate_task_energy(
                    instructions,
                    self.current_state,
                )
            if self.battery_manager.should_defer_task(energy_required):
                print(
                    f"[Cycle {cycle}] Task {getattr(task, 'id', '')!r} deferred due to low energy budget"
                )
                continue

            thermal_load = 0.0
            with contextlib.suppress(Exception):
                thermal_load = self.thermal_manager.estimate_task_thermal_load(
                    instructions,
                    self.current_state,
                )
            throttle = self.thermal_manager.should_throttle(thermal_load) or hazard_throttle

            for instruction in instructions:
                desired_joint_commands[instruction.joint_id] = self._instruction_to_joint_command(
                    instruction,
                    throttle=throttle,
                )

        return desired_joint_commands

    def _instruction_to_joint_command(
        self,
        instruction: JointInstruction,
        *,
        throttle: bool,
    ) -> JointCommand:
        """Convert a task-mapping instruction into a bounded joint command."""
        command_dict = instruction.command.copy() if isinstance(instruction.command, dict) else {}
        if throttle:
            velocity = command_dict.get("velocity")
            if velocity is not None:
                with contextlib.suppress(Exception):
                    command_dict["velocity"] = 0.5 * float(velocity)
            torque = command_dict.get("torque")
            if torque is not None:
                with contextlib.suppress(Exception):
                    command_dict["torque"] = 0.5 * float(torque)

        return JointCommand(
            position=command_dict.get("position"),
            velocity=command_dict.get("velocity"),
            torque=command_dict.get("torque"),
        )

    def _apply_joint_commands(self, commands: dict[str, JointCommand]) -> None:
        """Synchronise desired joint commands and refresh the cached robot state."""
        if not commands:
            self._last_joint_commands = {}
            return

        preserved_state = {
            key: self.current_state.get(key)
            for key in _PRESERVED_STATE_KEYS
            if key in self.current_state
        }
        joint_state_only = {
            joint_id: {
                "position": self.current_state.get("positions", {}).get(joint_id),
                "velocity": self.current_state.get("velocities", {}).get(joint_id),
            }
            for joint_id in commands
        }

        with self.concurrency_manager.acquire("hardware"):
            sensor_data = self.joint_sync.synchronize(commands, joint_state_only)

        self.current_state = self._normalise_state(sensor_data)
        self.current_state.update(preserved_state)
        self._last_joint_commands = commands.copy()

    def _publish_runtime_outputs(
        self,
        cycle: int,
        start_time: float,
        consistent: bool,
        faults: list[str],
        hazards: dict[str, Any],
    ) -> None:
        """Emit runtime metrics, health events, and telemetry for the cycle."""
        try:
            metrics = build_runtime_metrics(
                cycle=cycle,
                loop_duration_s=time.perf_counter() - start_time,
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
        except (AttributeError, OSError, RuntimeError, TypeError, ValueError) as exc:
            self._logger.debug("Telemetry dispatch failed for cycle %s: %s", cycle, exc)

    def submit_task(self, task: Task) -> None:
        """Queue a high-level task for execution in the next control cycle."""
        with self.concurrency_manager.acquire("tasks"):
            self.pending_tasks.append(task)

    def submit_goal(self, goal: dict[str, Any]) -> None:
        """Submit a high-level mission goal and, when possible, a navigation target."""
        self.mission_planner.add_goal(goal)
        if isinstance(goal, dict) and "x" in goal and "y" in goal:
            with contextlib.suppress(Exception):
                self.navigation_manager.set_goal((float(goal["x"]), float(goal["y"])))

    def run(self, num_cycles: int = 100) -> None:
        """Run the deterministic control loop for ``num_cycles`` iterations."""
        for _ in range(num_cycles):
            cycle = self._cycle_index
            self._cycle_index += 1

            self._reconfigure_if_needed()
            self._handle_incoming_commands()
            start = time.perf_counter()

            self._read_augmented_state()
            hazard_stop, hazard_throttle, faults, hazards = self._update_faults_and_hazards()
            self._update_planning()
            self._update_power_and_thermal(cycle)
            self.execution_stack.step()

            desired_joint_commands = self._collect_joint_commands(
                cycle,
                hazard_stop,
                hazard_throttle,
            )
            self._apply_joint_commands(desired_joint_commands)

            consistent = self.consistency_verifier.verify(self.current_state)
            if not consistent:
                print(f"[Cycle {cycle}] Consistency errors: {self.consistency_verifier.errors}")
            if not self.battery_manager.is_ok():
                print(f"[Cycle {cycle}] Warning: battery low or temperature out of range")
            if not self.thermal_manager.is_within_limits():
                print(f"[Cycle {cycle}] Warning: thermal limit exceeded; cooling recommended")
                print(f"[Cycle {cycle}] Cooling duties: {self.thermal_manager.recommended_cooling()}")

            self._publish_runtime_outputs(cycle, start, consistent, faults, hazards)

            elapsed = time.perf_counter() - start
            remaining = self.cycle_time - elapsed
            if remaining > 0:
                time.sleep(remaining)

    @staticmethod
    def _has_nonzero_velocity_command(command: Any) -> bool:
        """Return ``True`` when a navigation velocity payload requests motion."""
        if not isinstance(command, (tuple, list)):
            return False
        for value in command:
            with contextlib.suppress(TypeError, ValueError):
                if abs(float(value)) > 1e-9:
                    return True
        return False


if __name__ == "__main__":
    orchestrator = RobotOrchestrator(
        cycle_time=0.01,
        total_memory_bytes=1024 * 1024,
        battery_capacity_wh=100.0,
        max_temperature=60.0,
        max_velocity=2.0,
        max_torque=1.5,
    )
    orchestrator.submit_task(
        Task(
            id="move_joints",
            parameters={"joint_positions": {"joint1": 0.5, "joint2": -0.3}},
        )
    )
    orchestrator.run(num_cycles=20)
