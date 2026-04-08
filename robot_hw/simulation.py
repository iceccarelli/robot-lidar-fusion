"""
Stress Test Simulation for the Humanoid Robot Control System
===========================================================

This script runs a verbose and stochastic simulation of the robot
control system.  It overrides the orchestrator’s run loop to emit
detailed information about each stage (sensor reading, manager
updates, task processing, command dispatch, consistency checking) and
injects randomness into battery consumption, temperature changes,
joint commands and memory usage.  The goal is to stress the entire
stack and observe interactions between modules under load.

Key Features
------------
* **Verbose logging:** Each control cycle prints which functions are
  invoked and reports sensor data, manager states, tasks processed and
  any warnings or errors.
* **Random task generation:** Multiple threads submit random joint
  position commands, including occasional velocity and torque
  requests, to test the joint synchroniser’s clamping and fallback
  behaviour.
* **Battery & thermal noise:** Battery voltage, current and SOC are
  perturbed each cycle to simulate realistic fluctuations; temperatures
  rise randomly and trigger cooling suggestions.
* **Memory & concurrency stress:** Random memory allocations and
  multi‑lock acquisitions are performed to exercise the memory and
  concurrency managers.

Run this script with ``python stress_simulation.py`` from the root of
the repository.  Adjust `NUM_CYCLES`, `NUM_TASK_THREADS` or other
constants to change the intensity and duration of the test.
"""

from __future__ import annotations

import contextlib
import random
import threading
import time
from typing import Any

from robot_orchestrator import RobotOrchestrator

from robot_hw.planning.task_hardware_mapping import Task
from robot_hw.robot_config import load as load_config


class VerboseRobotOrchestrator(RobotOrchestrator):
    """Extends RobotOrchestrator to provide verbose output and random noise."""

    def run(self, num_cycles: int = 100) -> None:  # type: ignore[override]
        for cycle in range(num_cycles):
            cycle_start = time.perf_counter()
            print(f"\n===== Cycle {cycle} =====")
            # 1. Read sensors (always refresh state)
            with self.concurrency_manager.acquire("hardware"):
                print("[Sensor] Reading sensors via hardware synchroniser…")
                sensor_data = self.hardware.sync({})
                self.current_state = self._normalise_state(sensor_data)
                # Inject a random proximity reading (0.1–3 m) to test hazard detection
                self.current_state["proximity"] = random.uniform(0.1, 3.0)
                # Randomly inject environmental hazards.  Each hazard has a
                # small probability of being present on each cycle.  If
                # present, the value represents the distance in metres to the
                # hazard.  Empty dictionary means no hazards detected.
                hazards: dict[str, Any] = {}
                if random.random() < 0.05:
                    hazards["high_voltage"] = random.uniform(0.5, 5.0)
                if random.random() < 0.03:
                    hazards["train"] = random.uniform(5.0, 20.0)
                if random.random() < 0.05:
                    hazards["car"] = random.uniform(1.0, 10.0)
                if random.random() < 0.04:
                    hazards["human"] = random.uniform(0.5, 3.0)
                self.current_state["hazards"] = hazards
                # Randomly flag the presence of a pedestrian crossing the path
                self.current_state["pedestrian"] = random.random() < 0.02
                # Stage B: sensor fusion and hazard detection within verbose simulation
                # Fuse additional state (orientation, linear velocity, acceleration)
                fused = self.sensor_processor.fuse_sensors(self.current_state)
                for k, v in fused.items():
                    if k not in self.current_state:
                        self.current_state[k] = v
                # Build signals dictionary for hazard manager
                signals: dict[str, Any] = {}
                if "proximity" in self.current_state:
                    signals["proximity"] = self.current_state["proximity"]
                hazards_dict = self.current_state.get("hazards")
                if isinstance(hazards_dict, dict):
                    for hk, hv in hazards_dict.items():
                        signals[hk] = hv
                if "pedestrian" in self.current_state:
                    signals["pedestrian"] = self.current_state["pedestrian"]
                # Update hazard manager and record flags
                self.hazard_manager.update(signals)
                self.current_state["hazard_flags"] = self.hazard_manager.current_hazards()
                # Stage C: navigation and locomotion updates within verbose simulation
                _plan = self.navigation_manager.update(self.current_state)
                _ = self.locomotion_controller.compute_commands((0.0, 0.0, 0.0))
                print(f"[Sensor] State: {self.current_state}")
            # 2. Update battery and thermal managers with noise
            # Battery: random decay and noise on voltage/current
            ts = self.current_state.get("timestamp", cycle * self.cycle_time)
            voltage = max(0.0, 48.0 + random.uniform(-0.5, 0.5) - 0.02 * cycle)
            current = max(0.0, 8.0 + random.uniform(-2.0, 2.0))
            # Simulate SOC decay (0.001 per cycle) plus noise
            soc = max(0.0, 1.0 - 0.001 * cycle + random.uniform(-0.01, 0.01))
            temperature = 25.0 + 0.2 * cycle + random.uniform(-1.0, 1.0)
            from power.battery_management import BatteryState

            battery_state = BatteryState(
                voltage=voltage,
                current=current,
                temperature=temperature,
                soc=soc,
                health=1.0,
                timestamp=ts,
            )
            self.battery_manager.update(battery_state)
            # Thermal: per-joint temperatures with noise
            temp_data: dict[str, float] = {}
            for jid in self.current_state.get("positions", {}):
                # Temperature rises with cycle count and random noise
                temp_data[jid] = 30.0 + 0.3 * cycle + random.uniform(-2.0, 2.0)
            self.thermal_manager.update(temp_data)
            print(f"[Managers] Battery state: {battery_state}")
            print(f"[Managers] Temps: {temp_data}")
            # 3. Run any scheduled tasks
            print("[ExecutionStack] Executing scheduled tasks…")
            self.execution_stack.step()
            # 4. Process pending high-level tasks, prioritise and map them
            desired_joint_commands: dict[str, Any] = {}
            with self.concurrency_manager.acquire("tasks"):
                tasks_to_process = self.pending_tasks
                self.pending_tasks = []
            if tasks_to_process:
                # First order tasks by distance to minimise travel
                try:
                    ordered = self.task_mapper.assign_task_sequence(
                        tasks_to_process, self.current_state
                    )
                except Exception:
                    ordered = list(tasks_to_process)
                # Then prioritise based on estimated energy and thermal load
                try:
                    prioritized = self.task_mapper.prioritize_tasks(
                        ordered,
                        battery_manager=self.battery_manager,
                        thermal_manager=self.thermal_manager,
                        current_state=self.current_state,
                    )
                except Exception:
                    prioritized = list(ordered)
                print(f"[Tasks] Mapping {len(prioritized)} tasks…")
                for task in prioritized:
                    try:
                        instructions = self.task_mapper.map_task(task, self.current_state)
                    except Exception as e:
                        print(f"[Tasks] Error mapping task {task.id}: {e}")
                        continue
                    # Estimate resource costs
                    energy = self.battery_manager.estimate_task_energy(
                        instructions, self.current_state
                    )
                    thermal_load = self.thermal_manager.estimate_task_thermal_load(
                        instructions, self.current_state
                    )
                    print(
                        f"[Tasks] Estimated energy: {energy:.3f} Wh, thermal load: {thermal_load:.3f} °C"
                    )
                    # Check battery and thermal limits
                    if self.battery_manager.should_defer_task(energy):
                        print(f"[Battery] Deferring task {task.id} (energy {energy:.2f} Wh)")
                        continue
                    throttled = False
                    if self.thermal_manager.should_throttle(thermal_load):
                        throttled = True
                        print(f"[Thermal] Throttling task {task.id} (load {thermal_load:.2f} °C)")
                    for inst in instructions:
                        cmd_dict = inst.command.copy()
                        if throttled and cmd_dict.get("velocity") is not None:
                            with contextlib.suppress(Exception):
                                cmd_dict["velocity"] = float(cmd_dict["velocity"]) * 0.5
                        desired_joint_commands[inst.joint_id] = cmd_dict
                        print(f"[Tasks] Instruction → joint {inst.joint_id}: {cmd_dict}")
            # 5. Apply joint commands if any were created
            if desired_joint_commands:
                print(f"[JointSync] Applying commands: {desired_joint_commands}")
                # Build JointCommand objects on the fly
                from control.joint_synchronization import JointCommand

                jc_map = {
                    jid: JointCommand(
                        position=cmd.get("position"),
                        velocity=cmd.get("velocity"),
                        torque=cmd.get("torque"),
                    )
                    for jid, cmd in desired_joint_commands.items()
                }
                # Build the current joint state for the synchroniser.  This
                # includes only the position and velocity for joints in
                # ``desired_joint_commands``.  Environment hazard keys are
                # appended to allow the synchroniser to merge them into
                # sensor feedback for safety checks.
                joint_state_only = {
                    jid: {
                        "position": self.current_state.get("positions", {}).get(jid),
                        "velocity": self.current_state.get("velocities", {}).get(jid),
                    }
                    for jid in desired_joint_commands
                }
                # Propagate hazard fields under an environment key to keep types consistent
                env = {
                    "proximity": self.current_state.get("proximity"),
                    "hazards": self.current_state.get("hazards"),
                    "pedestrian": self.current_state.get("pedestrian"),
                }
                joint_state_only["_env"] = env
                with self.concurrency_manager.acquire("hardware"):
                    sensor_data = self.joint_sync.synchronize(jc_map, joint_state_only)
                self.current_state = self._normalise_state(sensor_data)
                print(f"[JointSync] New state: {self.current_state}")
            # 6. Consistency verification and safety checks
            consistent = self.consistency_verifier.verify(self.current_state)
            if not consistent:
                print(f"[Consistency] Errors: {self.consistency_verifier.errors}")
            else:
                print("[Consistency] OK")
            if not self.battery_manager.is_ok():
                print("[Safety] Battery warning: low SOC or temperature out of range")
            if not self.thermal_manager.is_within_limits():
                print("[Safety] Thermal warning: temperature exceeds limit")
            # 7. Random memory allocation and concurrency stress
            try:
                if random.random() < 0.3:
                    size = random.randint(256, 2048)
                    handle = self.memory_manager.allocate(size)
                    print(f"[Memory] Allocated {size} bytes")
                    # Immediately release half of allocations
                    if random.random() < 0.5:
                        self.memory_manager.release(handle)
                        print(f"[Memory] Released {size} bytes")
            except MemoryError:
                print("[Memory] Allocation failed: insufficient memory")
            # Acquire multiple locks in random order to test concurrency manager
            if random.random() < 0.2:
                locks = ["hardware", "memory", "tasks"]
                random.shuffle(locks)
                with self.concurrency_manager.acquire_many(locks):
                    print(f"[Concurrency] Acquired locks {locks} safely")
            # 8. Report battery runtime and memory health occasionally
            if cycle % 10 == 0:
                remaining_runtime = self.battery_manager.predict_runtime()
                mem_ok = self.memory_manager.check_health()
                print(
                    f"[Report] Predicted runtime: {remaining_runtime:.2f} s"
                    if remaining_runtime
                    else "[Report] Runtime prediction unavailable"
                )
                print(f"[Report] Memory health: {'OK' if mem_ok else 'Low'}")
            # Sleep to maintain timing
            elapsed = time.perf_counter() - cycle_start
            remaining = self.cycle_time - elapsed
            if remaining > 0:
                time.sleep(remaining)


def random_task_submitter(
    orchestrator: VerboseRobotOrchestrator, interval: float, run_event: threading.Event
) -> None:
    """Submit random tasks with random joint commands at a fixed interval."""
    cfg = load_config()
    joint_ids = list(cfg.joint_ids)
    while run_event.is_set():
        targets: dict[str, float] = {}
        velocities: dict[str, float] = {}
        torques: dict[str, float] = {}
        for jid in joint_ids:
            targets[jid] = random.uniform(-1.5, 1.5)
            # Occasionally send velocity or torque commands
            if random.random() < 0.3:
                velocities[jid] = random.uniform(-2.0, 2.0)
            if random.random() < 0.2:  # nosec B311
                torques[jid] = random.uniform(-1.0, 1.0)  # nosec B311
        # Build a parameter dict that includes only present fields
        params: dict[str, Any] = {"joint_positions": targets}
        if velocities:
            params["joint_velocities"] = velocities
        if torques:
            params["joint_torques"] = torques
        task = Task(id=f"random_task_{time.time():.3f}", parameters=params)
        orchestrator.submit_task(task)
        time.sleep(interval)


def main() -> None:
    cfg = load_config()
    orchestrator = VerboseRobotOrchestrator(
        cycle_time=cfg.cycle_time_s,
        total_memory_bytes=cfg.total_memory_bytes,
        battery_capacity_wh=cfg.battery_capacity_wh,
        max_temperature=cfg.max_temperature_c,
        max_velocity=max(cfg.max_velocity_per_joint) if cfg.max_velocity_per_joint else 1.0,
        max_torque=max(cfg.max_torque_per_joint) if cfg.max_torque_per_joint else 1.0,
    )
    NUM_CYCLES = 100
    TASK_SUBMIT_INTERVAL = 0.05
    # Run orchestrator in background
    done_event = threading.Event()
    done_event.set()
    orch_thread = threading.Thread(target=orchestrator.run, args=(NUM_CYCLES,), daemon=True)
    orch_thread.start()
    # Run random task submitter
    run_event = threading.Event()
    run_event.set()
    submitter_thread = threading.Thread(
        target=random_task_submitter,
        args=(orchestrator, TASK_SUBMIT_INTERVAL, run_event),
        daemon=True,
    )
    submitter_thread.start()
    # Wait for orchestrator to finish
    orch_thread.join()
    run_event.clear()
    submitter_thread.join()
    print("\nStress simulation complete.")


if __name__ == "__main__":
    main()
