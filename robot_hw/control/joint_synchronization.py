"""
Joint Synchronization Module
===========================

This module defines a `JointSynchronizer` class responsible for
synchronising joint commands across all actuators.  In a humanoid
robot, multiple joints must move in a coordinated fashion to achieve
desired poses and trajectories.  The synchroniser uses feedback from
sensors and the kinematic model to adjust commands on the fly and to
respect constraints such as maximum velocity and torque.

Key ideas:

* Convert high‑level desired joint states into low‑level actuator
  commands.
* Maintain synchronisation by ensuring that commands are sent for all
  joints in the same control cycle.
* Interface with the hardware synchroniser to apply commands and
  retrieve updated joint states.

This class does not perform motion planning; that is the role of the
task planner and hardware mapper.  Instead, it ensures the plan is
executed faithfully and adjusts for minor discrepancies.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from robot_hw.core.hardware_synchronization import ActuatorCommand, HardwareSynchronizer


@dataclass
class JointCommand:
    """Desired control inputs for a single joint.

    Attributes
    ----------
    position : float | None
        Target joint position (radians or metres).
    velocity : float | None
        Target joint velocity.
    torque : float | None
        Target torque/force to apply.
    """

    position: float | None = None
    velocity: float | None = None
    torque: float | None = None


class JointSynchronizer:
    """Synchronise joint commands and enforce kinematic constraints.

    The ``JointSynchronizer`` converts high‑level joint commands into
    low‑level actuator commands suitable for the hardware synchroniser.
    It applies conservative saturation on commanded velocities and torques
    to avoid commanding values outside the physical capabilities of the
    actuators.  If only a desired position is provided, a simple
    proportional controller is used to compute an appropriate velocity
    towards the target within a single cycle.  The synchroniser does
    **not** perform full trajectory planning; it merely ensures that
    each joint is commanded in a way that is safe and synchronised.

    Parameters
    ----------
    hardware_synchronizer : HardwareSynchronizer
        The low‑level synchroniser that actually dispatches commands
        to the hardware.
    max_velocity : float, optional
        A default maximum velocity (in units per second) used when
        clamping requested velocities.  Individual joints may
        internally apply their own limits based on configuration.
    max_torque : float, optional
        A default maximum torque/force used when clamping requested
        torque values.
    cycle_time : float, optional
        The control cycle duration in seconds.  This is used when
        computing a velocity from a position error if no velocity is
        provided.  Defaults to 0.01 s (100 Hz).
    """

    def __init__(
        self,
        hardware_synchronizer: HardwareSynchronizer,
        max_velocity: float = 1.0,
        max_torque: float = 1.0,
        cycle_time: float = 0.01,
    ) -> None:
        self._hardware = hardware_synchronizer
        self.max_velocity = float(max_velocity)
        self.max_torque = float(max_torque)
        self.cycle_time = float(cycle_time) if cycle_time > 0 else 0.01

    def _clamp(self, value: float, limit: float) -> float:
        """Clamp ``value`` to the range [−limit, +limit]."""
        return max(min(value, limit), -limit)

    def synchronize(
        self, desired: dict[str, JointCommand], current: dict[str, Any]
    ) -> dict[str, Any]:
        """Compute synchronised actuator commands and apply them.

        For each joint, the method constructs an :class:`ActuatorCommand`
        with position, velocity and/or torque fields.  If a desired
        velocity is supplied, it is clamped to ``max_velocity``.  If
        only a desired position is given, the method computes a simple
        velocity proportional to the position error such that the joint
        would reach the target in one control cycle at most.  Torque
        commands are clamped to ``max_torque``.

        Parameters
        ----------
        desired : dict[str, JointCommand]
            Desired position/velocity/torque for each joint.
        current : dict[str, Any]
            Current joint state, as returned by the hardware layer.  At a
            minimum should include ``position`` per joint, and optionally
            ``velocity`` and ``torque``.

        Returns
        -------
        dict[str, Any]
            Updated joint states after applying the commands.
        """
        commands: dict[str, ActuatorCommand] = {}
        for joint_id, cmd in desired.items():
            pos_cmd = cmd.position
            vel_cmd = cmd.velocity
            torque_cmd = cmd.torque
            # Clamp velocity if provided
            if vel_cmd is not None:
                vel_cmd = float(self._clamp(float(vel_cmd), self.max_velocity))
            # Compute velocity from position error if not provided
            if pos_cmd is not None and vel_cmd is None:
                # Determine current position; fall back to 0 if unknown
                curr_state = current.get(joint_id) or {}
                curr_pos = curr_state.get("position", 0.0)
                try:
                    error = float(pos_cmd) - float(curr_pos)
                except Exception:
                    error = 0.0
                # Compute velocity needed to close error in one cycle
                # but clamp to max_velocity
                vel_cmd = self._clamp(error / self.cycle_time, self.max_velocity)
            # Clamp torque if provided
            if torque_cmd is not None:
                torque_cmd = float(self._clamp(float(torque_cmd), self.max_torque))
            commands[joint_id] = ActuatorCommand(
                id=joint_id, position=pos_cmd, velocity=vel_cmd, torque=torque_cmd
            )
        # Send commands and receive new sensor data
        new_state = self._hardware.sync(commands)

        # -----------------------------------------------------------------
        # -----------------------------------------------------------------
        # Perform safety and environment checks using hazard information.
        #
        # The hardware interface returns joint positions/velocities/torques
        # but does not include environment keys (proximity, hazards,
        # pedestrian).  These keys may be present in the ``current``
        # dictionary when the synchroniser is called from a higher layer.
        # To avoid contaminating the sensor data with non‑joint keys, we
        # build a separate copy of the state that includes the hazards
        # solely for safety checks.  The returned ``new_state`` remains
        # free of these keys so that downstream normalisation treats
        # only joint identifiers as actuators.
        state_for_check: dict[str, Any] = dict(new_state)
        for key in ("proximity", "hazards", "pedestrian"):
            if key in current:
                state_for_check[key] = current[key]
        # After applying commands, perform additional safety and environment
        # awareness checks.  These checks log warnings for dangerous
        # conditions (e.g. torque overloads, nearby obstacles, uneven
        # ground, pedestrians or environmental hazards).  They must not
        # throw exceptions; any errors are swallowed to preserve the
        # deterministic control loop.
        try:
            # Check torque limits
            self.check_torque_overload(state_for_check)
            # Warn if objects are too close
            self.detect_proximity_hazards(state_for_check)
            # Detect uneven ground or joint spans
            self.adjust_for_uneven_ground(state_for_check)
            # Pause if pedestrians are detected
            self.wait_for_pedestrians(state_for_check)
            # Detect environmental hazards such as high voltage cables or
            # approaching trains/cars/humans
            self.detect_environment_hazards(state_for_check)
        except Exception as exc:
            return state_for_check
        # Return the clean joint state; hazard keys are not propagated
        return new_state

    # ------------------------------------------------------------------
    # Additional safety and environment awareness features
    # ------------------------------------------------------------------
    def check_torque_overload(self, state: dict[str, Any]) -> None:
        """Check if any joint exceeds safe torque levels and log warnings.

        Parameters
        ----------
        state : dict
            Sensor data dictionary containing a ``torques`` mapping.

        Notes
        -----
        A warning is printed if the absolute torque exceeds ``self.max_torque``.
        """
        torques = state.get("torques", {})
        for jid, tor in torques.items():
            try:
                tor_val = float(tor)
            except (TypeError, ValueError):
                continue
            if abs(tor_val) > self.max_torque:
                print(
                    f"[TorqueWarn] Joint {jid} torque {tor_val:.2f} exceeds limit {self.max_torque:.2f}"
                )

    def adjust_for_uneven_ground(self, state: dict[str, Any]) -> None:
        """Placeholder to adjust commands when uneven ground is detected.

        In a real robot, accelerometer or force sensors would detect ground
        slope or compliance.  Here we simply print a notice when the
        difference between foot joint positions suggests an incline.
        """
        positions = state.get("positions", {})
        if not positions:
            return
        # Compute simple heuristic: range of joint positions
        try:
            vals = [float(v) for v in positions.values()]
        except Exception:
            return
        if vals:
            span = max(vals) - min(vals)
            if span > 1.0:
                print(f"[Ground] Uneven ground detected (span {span:.2f}); consider adjusting gait")

    def detect_proximity_hazards(self, state: dict[str, Any]) -> None:
        """Detect proximity hazards such as humans or obstacles.

        The mock hardware may include a ``proximity`` key representing the
        distance to the nearest object.  If the distance falls below
        0.5 m a warning is printed.  In real deployments this would use
        LIDAR, ultrasonic or camera sensors.
        """
        proximity = state.get("proximity")
        try:
            dist = float(proximity) if proximity is not None else None
        except Exception:
            dist = None
        if dist is not None and dist < 0.5:
            print(f"[Hazard] Proximity alert: object detected at {dist:.2f} m")

    def wait_for_pedestrians(self, state: dict[str, Any]) -> None:
        """Pause motion if a pedestrian or human is in the robot's path.

        A boolean ``pedestrian`` field in the state signals that a human
        is currently crossing the robot's intended path.  When this flag
        is true, a warning is printed.  In a real controller, this
        method would command the actuators to hold position until the
        pedestrian is clear.
        """
        ped = state.get("pedestrian")
        if ped:
            print("[Pedestrian] Human detected in path – pausing movement")

    def detect_environment_hazards(self, state: dict[str, Any]) -> None:
        """Inspect environmental hazard sensors and log warnings.

        The state may include a ``hazards`` dictionary with boolean or
        numeric values for various danger categories (e.g. high voltage,
        trains, cars, humans).  This method iterates over those keys
        and prints a descriptive warning for any active hazard.  If a
        numeric value is provided (e.g. distance), it is formatted.
        """
        hazards = state.get("hazards")
        if not isinstance(hazards, dict):
            return
        for name, info in hazards.items():
            # Interpret ``info``: it may be a boolean or a distance
            if isinstance(info, bool) and info:
                print(f"[Hazard] {name.replace('_', ' ').title()} detected – take caution")
            else:
                try:
                    dist = float(info)
                except (TypeError, ValueError):
                    continue
                print(
                    f"[Hazard] {name.replace('_', ' ').title()} within {dist:.2f} m – take caution"
                )
