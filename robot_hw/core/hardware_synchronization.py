"""
Hardware Synchronisation Module
===============================

This module defines the :class:`HardwareSynchronizer`, responsible for
coordinating actuator commands and sensor readings in a deterministic
control loop. It provides a single entry point for applying a coherent
batch of actuator commands and then retrieving a consistent sensor
snapshot that reflects those commands.

The wrapper is intentionally lightweight so it can sit above many
possible hardware backends. A small in-memory mock backend is included
for simulation and tests. That mock is designed to integrate cleanly
with the repository's joint synchroniser, fault detector and
orchestrator.

Key responsibilities:

* Dispatch actuator commands in a single coherent batch per control
  cycle.
* Return sensor readings that correspond to the post-command state.
* Provide a stable mock backend for simulation and unit tests.
* Preserve deterministic timing semantics for higher-level controllers.
"""

from __future__ import annotations

import contextlib
import os
from dataclasses import dataclass
from secrets import SystemRandom
from typing import Any


@dataclass
class ActuatorCommand:
    """Container for actuator commands.

    Parameters
    ----------
    id : str
        Unique actuator identifier, such as a joint name.
    position : float | None
        Target position in radians or metres. ``None`` leaves the
        existing target unchanged.
    velocity : float | None
        Target velocity in units per second. ``None`` leaves the
        existing velocity unchanged unless a new position target implies
        a derived velocity.
    torque : float | None
        Target torque or force. ``None`` leaves the existing torque
        unchanged.
    """

    id: str
    position: float | None = None
    velocity: float | None = None
    torque: float | None = None


class HardwareSynchronizer:
    """Synchronise actuator commands and sensor readings.

    The synchroniser delegates low-level I/O to the supplied hardware
    interface. Callers interact with one stable method, :meth:`sync`,
    which applies a command batch and then reads a sensor snapshot.
    This preserves causal ordering and keeps higher-level modules
    independent from backend-specific details.
    """

    def __init__(self, hardware_interface: Any) -> None:
        """Create a new hardware synchroniser.

        Parameters
        ----------
        hardware_interface : Any
            Backend object expected to expose
            ``apply_commands(commands: dict[str, ActuatorCommand])`` and
            ``read_sensors() -> dict[str, Any]`` methods.
        """
        self._hardware = hardware_interface

    class MockHardwareInterface:
        """Simple in-memory backend for tests and simulation.

        The mock updates joint velocities and evolves joint position
        consistently for velocity-driven commands so the resulting sensor
        snapshots remain coherent for the repository's simulation,
        navigation, and fault-detection flows.
        """

        def __init__(self, cycle_time_s: float = 0.01) -> None:
            self.states: dict[str, dict[str, float]] = {}
            self._timestamp: float = 0.0
            self._cycle_time_s: float = float(cycle_time_s) if cycle_time_s > 0.0 else 0.01
            self._rng = SystemRandom()

        @staticmethod
        def _as_float(value: Any, default: float = 0.0) -> float:
            """Coerce ``value`` to ``float`` with a safe fallback."""
            try:
                return float(value)
            except Exception:
                return float(default)

        def _state_for(self, actuator_id: str) -> dict[str, float]:
            """Return the mutable state dictionary for one actuator."""
            return self.states.setdefault(
                actuator_id,
                {"position": 0.0, "velocity": 0.0, "torque": 0.0},
            )

        def apply_commands(self, commands: dict[str, ActuatorCommand]) -> None:
            """Apply one control batch to the simulated actuator state.

            Position-only commands update the joint position directly and
            derive a velocity estimate from the displacement over the
            mock cycle time. Velocity-only commands advance the stored
            position by ``velocity * cycle_time`` so the simulated sensor
            state evolves in a way that the fault detector recognises as
            healthy motion.
            """
            if not isinstance(commands, dict):
                return

            dt = self._cycle_time_s
            for actuator_id, command in commands.items():
                if not isinstance(command, ActuatorCommand):
                    continue

                state = self._state_for(actuator_id)
                previous_position = self._as_float(state.get("position"), 0.0)
                previous_velocity = self._as_float(state.get("velocity"), 0.0)
                previous_torque = self._as_float(state.get("torque"), 0.0)

                next_velocity = previous_velocity
                if command.velocity is not None:
                    next_velocity = self._as_float(command.velocity, previous_velocity)

                if command.position is not None:
                    next_position = self._as_float(command.position, previous_position)
                    if command.velocity is None and dt > 0.0:
                        next_velocity = (next_position - previous_position) / dt
                else:
                    next_position = previous_position + (next_velocity * dt)

                next_torque = previous_torque
                if command.torque is not None:
                    next_torque = self._as_float(command.torque, previous_torque)

                state["position"] = next_position
                state["velocity"] = next_velocity
                state["torque"] = next_torque

        def _inject_environment_signals(self, snapshot: dict[str, Any]) -> None:
            """Add simple synthetic environment signals to the snapshot.

            These signals intentionally remain lightweight and best-effort.
            They support the repository's simulation paths without
            imposing requirements on production backends.
            """
            env = os.getenv("ENVIRONMENT_PROFILE", "GENERAL").strip().upper()
            rng = self._rng

            if env == "MINING":
                snapshot["gas"] = rng.uniform(0.0, 3.0)
                snapshot["radiation"] = rng.uniform(0.0, 2.0)
                snapshot["high_voltage"] = rng.uniform(0.0, 10.0)
                snapshot["train"] = rng.uniform(0.0, 3.0) if rng.random() < 0.1 else 5.0
                snapshot["pedestrian"] = rng.random() < 0.05
            elif env == "UNDERWATER":
                snapshot["proximity"] = rng.uniform(0.0, 2.0)
                snapshot["gas"] = rng.uniform(0.0, 0.5) if rng.random() < 0.05 else 0.0
                snapshot["radiation"] = rng.uniform(0.0, 1.0) if rng.random() < 0.02 else 0.0
                snapshot["pedestrian"] = False
            elif env == "SPACE":
                snapshot["radiation"] = rng.uniform(0.0, 3.0)
                snapshot["high_voltage"] = rng.uniform(0.0, 20.0)
                snapshot["proximity"] = rng.uniform(0.0, 10.0)
                snapshot["pedestrian"] = False
            elif env == "FORESTRY":
                snapshot["proximity"] = rng.uniform(0.0, 1.5)
                snapshot["pedestrian"] = rng.random() < 0.2
                snapshot["human"] = rng.uniform(0.0, 2.0) if rng.random() < 0.1 else 5.0
            else:
                snapshot["proximity"] = rng.uniform(0.0, 5.0) if rng.random() < 0.05 else 5.0
                snapshot["pedestrian"] = rng.random() < 0.01

        def read_sensors(self) -> dict[str, Any]:
            """Return a consistent sensor snapshot for the current state.

            The timestamp advances once per sensor read using the fixed
            mock cycle time. Each actuator entry is returned as a fresh
            shallow copy containing ``position``, ``velocity`` and
            ``torque``.
            """
            self._timestamp += self._cycle_time_s
            snapshot: dict[str, Any] = {
                actuator_id: {
                    "position": self._as_float(state.get("position"), 0.0),
                    "velocity": self._as_float(state.get("velocity"), 0.0),
                    "torque": self._as_float(state.get("torque"), 0.0),
                }
                for actuator_id, state in self.states.items()
            }
            snapshot["timestamp"] = self._timestamp

            with contextlib.suppress(Exception):
                self._inject_environment_signals(snapshot)
            return snapshot

    def sync(self, commands: dict[str, ActuatorCommand]) -> dict[str, Any]:
        """Apply a batch of actuator commands and return sensor data.

        Parameters
        ----------
        commands : dict[str, ActuatorCommand]
            Mapping of actuator identifiers to commands for the current
            control step.

        Returns
        -------
        dict[str, Any]
            Raw sensor snapshot produced by the backend. The snapshot is
            returned as a plain dictionary so upstream callers can safely
            normalise or copy it.
        """
        if hasattr(self._hardware, "apply_commands"):
            with contextlib.suppress(Exception):
                self._hardware.apply_commands(commands)

        if hasattr(self._hardware, "read_sensors"):
            try:
                sensor_state = self._hardware.read_sensors()
                return dict(sensor_state) if isinstance(sensor_state, dict) else {}
            except Exception:
                return {}

        return {}
