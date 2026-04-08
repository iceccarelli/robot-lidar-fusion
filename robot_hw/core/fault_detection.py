"""Fault Detection Module
=========================

This module defines the :class:`FaultDetector`, responsible for
monitoring the health of sensors and actuators.  It detects stuck
readings, timeouts, outliers and other anomalies that may indicate a
hardware or software failure.  Upon detecting a fault, it can
transition the robot into a safe mode and initiate recovery
procedures.

Key features envisioned for this module include:

* **Sensor health monitoring:** Validate that sensors update at
  expected rates and within expected ranges.
* **Actuator feedback monitoring:** Ensure commanded actions are
  reflected in sensed states; detect discrepancies that may indicate
  mechanical failures.
* **Safe‑mode management:** Expose an interface for entering and
  exiting safe modes, including gradual shutdown or reduced
  capability modes.

In stage A this class is a skeleton; detailed fault detection logic
will be implemented in later stages.
"""

from __future__ import annotations

import logging
from typing import Any

from robot_hw.robot_config import RobotConfig


class FaultDetector:
    """Monitor and detect hardware or software faults.

    Parameters
    ----------
    config : RobotConfig
        System configuration including expected update rates and
        thresholds for fault detection.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        # Internal storage for detected faults
        self._faults: list[str] = []
        # Keep track of previous sensor data to detect stale or stuck values
        self._last_sensor_data: dict[str, Any] | None = None
        self._last_timestamp: float | None = None
        # Per‑joint counters for stuck detection
        self._stuck_counters: dict[str, int] = {}
        # Thresholds derived from configuration
        # Maximum allowed latency between sensor updates (seconds)
        self.max_sensor_latency = getattr(config, "cycle_time_s", 0.01) * 5.0
        # Maximum allowed positional jump per cycle (units) to detect outliers
        self.max_position_jump = 10.0
        # Number of consecutive identical readings before declaring a stuck sensor
        self.max_stuck_cycles = 3
        # Thresholds for comparing commanded and measured values
        # Pull from configuration with fallbacks
        try:
            self.max_position_error = float(getattr(config, "max_position_error", 5.0))
        except Exception:
            self.max_position_error = 5.0
        try:
            self.max_velocity_error = float(getattr(config, "max_velocity_error", 2.0))
        except Exception:
            self.max_velocity_error = 2.0
        # Logger for diagnostics
        self._logger = logging.getLogger(self.__class__.__name__)

    def update(self, sensor_data: dict[str, Any], actuator_commands: dict[str, Any]) -> None:
        """Update the fault detector with new sensor and actuator data.

        This method compares the latest sensor readings against the
        previous readings to detect anomalies such as stale timestamps,
        sensors that are stuck at a constant value and outliers where
        values jump unexpectedly.  Detected faults are stored in
        ``self._faults``.  In a real system this method should run
        quickly and deterministically in each cycle.

        Parameters
        ----------
        sensor_data : dict[str, Any]
            Latest sensor readings from the hardware synchroniser.  It
            should contain a ``timestamp`` and dictionaries of joint
            positions and velocities under keys like ``positions``.
        actuator_commands : dict[str, Any]
            Commands recently sent to actuators, not used in this
            simplified implementation but included for completeness.
        """
        # Reset faults each update
        self._faults.clear()
        if not isinstance(sensor_data, dict):
            return
        timestamp = sensor_data.get("timestamp")
        positions = sensor_data.get("positions", {})
        # Detect missing timestamp or stale update
        if timestamp is None:
            self._faults.append("missing_timestamp")
        else:
            try:
                ts = float(timestamp)
            except Exception:
                self._faults.append("invalid_timestamp")
                ts = None
            if ts is not None and self._last_timestamp is not None:
                dt = ts - self._last_timestamp
                # Negative or excessively long intervals indicate a timeout
                if dt < 0 or dt > self.max_sensor_latency:
                    self._faults.append("sensor_timeout")
        # Detect stuck or outlier positions
        if isinstance(positions, dict):
            for jid, pos in positions.items():
                # Normalise numeric value
                try:
                    p = float(pos)
                except Exception:
                    continue
                # Retrieve the last position for this joint
                last_pos = None
                if self._last_sensor_data and isinstance(
                    self._last_sensor_data.get("positions"), dict
                ):
                    last_pos = self._last_sensor_data["positions"].get(jid)
                    try:
                        last_pos = float(last_pos) if last_pos is not None else None
                    except Exception:
                        last_pos = None
                # Determine whether a motion command is currently active for this joint.
                # If the commanded velocity is near zero (robot intentionally stopped),
                # we temporarily disable stuck detection to avoid false latching.
                commanded_vel = None
                if isinstance(actuator_commands, dict):
                    cmd = actuator_commands.get(jid)
                    if hasattr(cmd, "velocity"):
                        commanded_vel = (
                            cmd.velocity if cmd is not None and cmd.velocity is not None else 0.0
                        )
                    elif isinstance(cmd, dict):
                        commanded_vel = cmd.get("velocity")
                    try:
                        commanded_vel = float(commanded_vel) if commanded_vel is not None else None
                    except Exception:
                        commanded_vel = None
                # Update stuck counter: only increment if last_pos equals current and robot is commanded to move
                moving = False
                if commanded_vel is not None:
                    # Consider velocities above a small epsilon as movement
                    moving = abs(commanded_vel) > 1e-3
                if last_pos is not None and p == last_pos and moving:
                    self._stuck_counters[jid] = self._stuck_counters.get(jid, 0) + 1
                else:
                    # Reset counter if joint is not moving or value changed
                    self._stuck_counters[jid] = 0
                # If stuck count exceeds threshold while moving, declare a fault
                if self._stuck_counters[jid] > self.max_stuck_cycles and moving:
                    self._faults.append(f"sensor_stuck:{jid}")
                # Outlier detection: large jump from previous position
                if last_pos is not None and abs(p - last_pos) > self.max_position_jump:
                    self._faults.append(f"position_outlier:{jid}")
        # ------------------------------------------------------------------
        # Compare commanded vs measured positions and velocities to detect
        # actuator faults such as stalls or misalignment.  Commands are
        # provided via ``actuator_commands`` mapping joint IDs to
        # JointCommand objects or dictionaries with ``position`` and
        # ``velocity`` fields.  Large deviations trigger faults.
        # ------------------------------------------------------------------
        try:
            if isinstance(actuator_commands, dict) and isinstance(
                sensor_data.get("positions"), dict
            ):
                for jid, cmd in actuator_commands.items():
                    # Extract commanded position and velocity if available
                    c_pos = None
                    c_vel = None
                    if hasattr(cmd, "position"):
                        c_pos = cmd.position
                        c_vel = cmd.velocity
                    elif isinstance(cmd, dict):
                        c_pos = cmd.get("position")
                        c_vel = cmd.get("velocity")
                    # Measured values
                    m_pos = sensor_data["positions"].get(jid)
                    m_vel = (
                        sensor_data.get("velocities", {}).get(jid)
                        if isinstance(sensor_data.get("velocities"), dict)
                        else None
                    )
                    # Compare positions
                    if c_pos is not None and m_pos is not None:
                        try:
                            cval = float(c_pos)
                            mval = float(m_pos)
                            if abs(cval - mval) > self.max_position_error:
                                self._faults.append(f"position_mismatch:{jid}")
                        except Exception:
                            pass
                    # Compare velocities
                    if c_vel is not None and m_vel is not None:
                        try:
                            cval = float(c_vel)
                            mval = float(m_vel)
                            if abs(cval - mval) > self.max_velocity_error:
                                self._faults.append(f"velocity_mismatch:{jid}")
                        except Exception:
                            pass
        except Exception as exc:
            self._logger.debug("Error detecting mismatch faults: %s", exc)
        # Store current as last for next update
        self._last_sensor_data = sensor_data.copy()
        if timestamp is not None:
            try:
                self._last_timestamp = float(timestamp)
            except Exception:
                self._last_timestamp = None

    def has_fault(self) -> bool:
        """Return True if any fault has been detected.

        Returns
        -------
        bool
            True if there are one or more active faults; False otherwise.
        """
        return bool(self._faults)

    def get_faults(self) -> list[str]:
        """Return a list of detected faults.

        Returns
        -------
        list[str]
            Human‑readable descriptions of detected faults.
        """
        return list(self._faults)

    def clear_faults(self) -> None:
        """Clear the list of detected faults.

        This method should be called after the system has taken
        appropriate recovery actions.  In more advanced
        implementations, clearing faults may involve notifying other
        modules or resetting state.
        """
        self._faults.clear()
