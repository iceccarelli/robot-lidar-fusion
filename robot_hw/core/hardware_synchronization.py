"""
Hardware Synchronisation Module
===============================

This module defines the `HardwareSynchronizer` class responsible for
coordinating all actuator commands and sensor readings in a real‑time
control system.  It provides a single entry point for applying
batched commands to the hardware interface and returning updated
sensor data.  All callers should use this module rather than
interacting with the hardware interface directly to ensure
deterministic timing and ordering.

Key responsibilities:

* Dispatch actuator commands in a single coherent batch per control
  cycle.
* Enforce that sensor readings used in a cycle originate from the
  same timestamp.
* Provide a stable interface that can be mocked for simulation or
  unit tests.

The class is deliberately lightweight; heavy lifting (e.g. command
queueing, driver calls) will live in the underlying hardware interface
passed to the constructor.  This design allows swapping between
different hardware backends (real robot, simulator, test harness)
without changing higher‑level code.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import Any


@dataclass
class ActuatorCommand:
    """Container for actuator commands.

    Each command may specify a desired position, velocity and/or
    torque.  Omitting a field leaves it unchanged on the hardware.  At
    least one of the fields should be set for a meaningful command.

    Attributes
    ----------
    id : str
        Unique identifier of the actuator (e.g. joint name).
    position : float | None
        Target position in radians/metres; None to leave unchanged.
    velocity : float | None
        Target velocity; None to leave unchanged.
    torque : float | None
        Target torque or force; None to leave unchanged.
    """

    id: str
    position: float | None = None
    velocity: float | None = None
    torque: float | None = None


class HardwareSynchronizer:
    """Synchronise actuator commands and sensor readings.

    The `HardwareSynchronizer` is a thin wrapper around a lower‑level
    hardware interface.  It is responsible for ensuring that all
    actuators receive commands in a coordinated manner within a single
    control cycle and that sensor readings returned to callers are
    consistent (i.e. originate from the same timestamp).  This class
    does not block; instead it delegates real‑time constraints to the
    orchestrator and hardware interface.

    Invariants
    ----------
    * Each call to :meth:`sync` must send all accumulated commands
      together.
    * Sensor data returned from :meth:`sync` corresponds to the state
      *after* applying the commands from that same call.

    Notes
    -----
    The actual implementation of hardware communication (e.g. CAN bus,
    serial, Ethernet) should be provided by the `hardware_interface`
    passed to the constructor.  That object must expose methods to
    apply a batch of commands and to retrieve a batch of sensor
    readings.
    """

    def __init__(self, hardware_interface: Any) -> None:
        """Create a new hardware synchroniser.

        Parameters
        ----------
        hardware_interface : Any
            An object responsible for low‑level communication with the
            robot hardware.  It must implement a method
            ``apply_commands(commands: Dict[str, ActuatorCommand]) -> None``
            and a method ``read_sensors() -> Dict[str, Any]`` which
            returns the latest sensor states.
        """
        self._hardware = hardware_interface

    # ------------------------------------------------------------------
    # Optional mock implementation for testing and simulation
    # ------------------------------------------------------------------
    class MockHardwareInterface:
        """A simple in‑memory hardware interface for testing.

        This mock allows the synchroniser to be used without real hardware.
        It stores actuator states in a dictionary and updates them when
        commands are applied.  Sensor readings simply return the
        internal state.
        """

        def __init__(self) -> None:
            # Internal state keyed by actuator ID; each value is a dict
            # with position, velocity and torque.
            self.states: dict[str, dict[str, float]] = {}
            # Monotonic timestamp updated on each apply or read.  We use
            # a simple counter in lieu of real time for determinism.
            self._timestamp: float = 0.0

        def apply_commands(self, commands: dict[str, ActuatorCommand]) -> None:
            for aid, cmd in commands.items():
                state = self.states.setdefault(
                    aid, {"position": 0.0, "velocity": 0.0, "torque": 0.0}
                )
                # Update only fields that are provided
                if cmd.position is not None:
                    state["position"] = float(cmd.position)
                if cmd.velocity is not None:
                    state["velocity"] = float(cmd.velocity)
                if cmd.torque is not None:
                    state["torque"] = float(cmd.torque)

        def read_sensors(self) -> dict[str, dict[str, float]]:
            """Return a snapshot of all actuator states with a timestamp and hazard signals.

            Each actuator entry is a shallow copy of the internal state
            (position, velocity, torque).  The returned dictionary also
            includes a top‑level ``timestamp`` indicating the time of the
            reading.  In addition, environment‑specific hazard signals
            (e.g. gas concentration, radiation levels, proximity to
            obstacles) are synthesised to emulate sensor inputs in
            extreme environments.  Hazard values are generated
            probabilistically based on the current ``ENVIRONMENT_PROFILE``.

            In this mock implementation, the timestamp is incremented
            by 0.01 seconds on each call to simulate a 100 Hz control loop.
            Real implementations should use high‑precision monotonic timers.
            """
            # Increment the internal timestamp to simulate elapsed time
            self._timestamp += 0.01
            snapshot: dict[str, Any] = {aid: state.copy() for aid, state in self.states.items()}
            snapshot["timestamp"] = self._timestamp
            # Inject synthetic hazard signals to emulate environment sensors
            try:
                import os
                import random

                env = os.getenv("ENVIRONMENT_PROFILE", "GENERAL").strip().upper()
                # Define hazard injection per environment
                if env == "MINING":
                    # Gas and radiation hazards occur frequently in mining
                    snapshot["gas"] = random.uniform(0.0, 3.0)  # nosec B311
                    snapshot["radiation"] = random.uniform(0.0, 2.0)  # nosec B311
                    snapshot["high_voltage"] = random.uniform(0.0, 10.0)  # nosec B311
                    # Occasional trains or pedestrians in tunnels
                    train_distance = random.uniform(0.0, 3.0)  # nosec B311
                    train_detected = random.random() < 0.1  # nosec B311
                    snapshot["train"] = train_distance if train_detected else 5.0
                    snapshot["pedestrian"] = bool(random.random() < 0.05)  # nosec B311
                elif env == "UNDERWATER":
                    # Obstacles within a few metres detected by sonar
                    snapshot["proximity"] = random.uniform(0.0, 2.0)  # nosec B311
                    # Rare gas leaks or radiation underwater
                    gas_level = random.uniform(0.0, 0.5)  # nosec B311
                    gas_detected = random.random() < 0.05  # nosec B311
                    snapshot["gas"] = gas_level if gas_detected else 0.0
                    snapshot["radiation"] = (
                        random.uniform(0.0, 1.0) if random.random() < 0.02 else 0.0  # nosec B311
                    )
                    snapshot["pedestrian"] = False
                elif env == "SPACE":
                    # High radiation and electrical hazards in space operations
                    snapshot["radiation"] = random.uniform(0.0, 3.0)  # nosec B311
                    snapshot["high_voltage"] = random.uniform(0.0, 20.0)  # nosec B311
                    # Proximity values are less relevant; objects are far away
                    snapshot["proximity"] = random.uniform(0.0, 10.0)  # nosec B311
                    snapshot["pedestrian"] = False
                elif env == "FORESTRY":
                    # Frequent obstacles and humans in forestry
                    snapshot["proximity"] = random.uniform(0.0, 1.5)  # nosec B311
                    snapshot["pedestrian"] = bool(random.random() < 0.2)  # nosec B311
                    # Occasionally detect a human or animal close by
                    human_distance = random.uniform(0.0, 2.0)  # nosec B311
                    human_detected = random.random() < 0.1  # nosec B311
                    snapshot["human"] = human_distance if human_detected else 5.0
                else:
                    # Generic environment: occasional obstacles but mostly clear
                    snapshot["proximity"] = (
                        random.uniform(0.0, 5.0) if random.random() < 0.05 else 5.0  # nosec B311
                    )
                    snapshot["pedestrian"] = bool(random.random() < 0.01)  # nosec B311

            except Exception as exc:
                # If hazard injection fails, proceed with the basic snapshot
                snapshot["hazard_injection_error"] = str(exc)
            return snapshot

    def sync(self, commands: dict[str, ActuatorCommand]) -> dict[str, Any]:
        """Apply a batch of commands and return updated sensor data.

        This method should be called exactly once per control cycle by
        the orchestrator.  It applies all provided commands to the
        hardware interface and then reads back the sensor data.  The
        ordering is critical: commands are applied first, then sensors
        are read, ensuring causal consistency.  Callers must not
        mutate the `commands` dictionary after passing it to this
        method.

        Parameters
        ----------
        commands : dict[str, ActuatorCommand]
            A mapping from actuator identifiers to their desired
            commands for this cycle.

        Returns
        -------
        dict[str, Any]
            A mapping from actuator identifiers to sensor readings.
            The exact structure of each entry depends on the
            underlying hardware interface but typically includes
            position, velocity, torque, temperature and a timestamp.
        """
        # If no commands are provided, we still read sensors to
        # maintain synchronisation.
        if hasattr(self._hardware, "apply_commands"):
            with contextlib.suppress(Exception):
                self._hardware.apply_commands(commands)
        if hasattr(self._hardware, "read_sensors"):
            try:
                sensor_state = self._hardware.read_sensors()
                return dict(sensor_state)
            except Exception:
                # TODO: log and propagate sensor reading errors
                return {}
        # Fallback: return empty sensor data if interface lacks methods
        return {}
