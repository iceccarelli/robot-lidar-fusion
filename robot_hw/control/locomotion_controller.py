"""Locomotion Controller Module
=============================

This module defines the :class:`LocomotionController`, responsible for
generating actuator commands that achieve desired movements while
maintaining balance and stability.  It abstracts over different
locomotion modes (wheeled, legged, tracked, thruster‑based) and
provides a unified interface for the rest of the control stack.

Core responsibilities include:

* **Gait generation:** Compute joint trajectories for walking, running
  or swimming using inverse kinematics and dynamic models.
* **Balance control:** Adjust commands based on sensor feedback to
  maintain stability over uneven terrain or under external forces.
* **Slip detection and recovery:** Monitor contact sensors to detect
  slipping and initiate corrective actions.

In stage A this class is a skeleton; detailed control algorithms will
be implemented in later stages.
"""

from __future__ import annotations

import contextlib
import logging
import math
from typing import Any

from robot_hw.robot_config import RobotConfig


class LocomotionController:
    """Compute low‑level locomotion commands for the robot.

    Parameters
    ----------
    config : RobotConfig
        System configuration providing joint limits, actuator types and
        environment profiles.  See :mod:`robot_config` for details.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        # Store any state required for gait cycles or balance control
        self._internal_state: dict[str, Any] = {}
        # Determine locomotion mode from config; fallback to environment adapter overrides if available
        try:
            self._mode = getattr(config, "locomotion_mode", "WHEELS").upper()
        except Exception:
            self._mode = "WHEELS"
        # Last sensor state for slip detection
        self._last_sensor_state: dict[str, Any] | None = None
        self._slip_counters: dict[str, int] = {}
        # Slip detection threshold: difference between commanded and measured velocities
        self._slip_threshold = 0.5
        # Logger for diagnostics
        self._logger = logging.getLogger(self.__class__.__name__)

    def compute_commands(self, target_velocity: tuple[float, float, float]) -> dict[str, float]:
        """Compute actuator commands to achieve a target velocity.

        Given a desired linear and angular velocity vector in the robot's
        body frame, compute the necessary joint velocities for each
        actuator.  This simple implementation maps the x‑component of
        the target velocity equally to all joints, clamped by per‑joint
        maximum velocities defined in :class:`RobotConfig`.  More
        sophisticated implementations should consider the locomotion
        mode (wheeled, legged, thrusters) and perform inverse
        kinematics or gait generation.

        Parameters
        ----------
        target_velocity : tuple[float, float, float]
            Desired velocities along the x, y and angular axes.  Only
            the first component (forward speed) is used in this
            implementation.

        Returns
        -------
        dict[str, float]
            A mapping from joint identifiers to velocity commands.
        """
        # Extract desired linear velocities and angular velocity
        try:
            vx = float(target_velocity[0])
        except Exception:
            vx = 0.0
        try:
            vy = float(target_velocity[1])
        except Exception:
            vy = 0.0
        try:
            w = float(target_velocity[2])
        except Exception:
            w = 0.0
        commands: dict[str, float] = {}
        # Determine locomotion mode and compute commands accordingly
        joint_ids = self._config.joint_ids if hasattr(self._config, "joint_ids") else ()
        max_vels = (
            self._config.max_velocity_per_joint
            if hasattr(self._config, "max_velocity_per_joint")
            else ()
        )
        mode = self._mode
        if mode == "WHEELS" or mode == "TRACKS":
            # Map forward (vx) and angular (w) velocity to differential wheels/tracks
            for idx, jid in enumerate(joint_ids):
                # Determine sign: assume even indices are left, odd are right for tracks
                side = 1.0
                if mode == "TRACKS" and idx % 2 == 0:
                    side = -1.0
                # Compute commanded velocity: forward velocity plus angular component
                cmd = vx + side * w
                max_v = 1.0
                with contextlib.suppress(Exception):
                    max_v = float(max_vels[idx])
                commands[jid] = max(-max_v, min(cmd, max_v))
        elif mode == "THRUSTERS":
            # Use forward and sideways velocities to assign thruster outputs; map each joint to vx, vy combination
            for idx, jid in enumerate(joint_ids):
                max_v = 1.0
                with contextlib.suppress(Exception):
                    max_v = float(max_vels[idx])
                # Alternate thrusters control x and y axes
                cmd = vx if idx % 2 == 0 else vy
                commands[jid] = max(-max_v, min(cmd, max_v))
        elif mode == "LEGS":
            # Generate a simple gait pattern using sinusoidal oscillations for walking
            t = self._internal_state.get("time", 0.0)
            period = 1.0  # seconds per gait cycle
            amplitude = 0.5 * vx  # scale stride length with desired velocity
            # Update time based on a nominal cycle (assuming compute_commands called at 100 Hz)
            dt = 0.01
            self._internal_state["time"] = t + dt
            for idx, jid in enumerate(joint_ids):
                phase = (idx / len(joint_ids)) * math.pi
                _pos = amplitude * math.sin(2 * math.pi * (t / period) + phase)
                # Use velocity command equal to derivative of position (approx)
                vel_cmd = (
                    (2 * math.pi / period)
                    * amplitude
                    * math.cos(2 * math.pi * (t / period) + phase)
                )
                max_v = 1.0
                with contextlib.suppress(Exception):
                    max_v = float(max_vels[idx])
                commands[jid] = max(-max_v, min(vel_cmd, max_v))
        else:
            # Default behaviour: distribute forward velocity equally
            for idx, jid in enumerate(joint_ids):
                max_v = 1.0
                with contextlib.suppress(Exception):
                    max_v = float(max_vels[idx])
                commands[jid] = max(-max_v, min(vx, max_v))
        # Slip detection: compare commanded velocity with measured velocity when available
        if self._last_sensor_state and isinstance(self._last_sensor_state.get("velocities"), dict):
            measured = self._last_sensor_state["velocities"]
            for jid, cmd_v in commands.items():
                m_v = measured.get(jid)
                try:
                    mv = float(m_v) if m_v is not None else 0.0
                except Exception:
                    mv = 0.0
                if abs(cmd_v - mv) > self._slip_threshold:
                    self._slip_counters[jid] = self._slip_counters.get(jid, 0) + 1
                    # If slip persists, reduce command magnitude
                    if self._slip_counters[jid] > 2:
                        commands[jid] = 0.5 * cmd_v
                else:
                    self._slip_counters[jid] = 0
        return commands

    def update_state(self, sensor_state: dict[str, Any]) -> None:
        """Update internal state with the latest sensor readings.

        The locomotion controller uses sensor state to detect slip and
        adjust commands accordingly.  This method should be called each
        control cycle before computing commands.

        Parameters
        ----------
        sensor_state : dict[str, Any]
            The most recent fused sensor state containing at least
            ``positions`` and optionally ``velocities``.
        """
        if isinstance(sensor_state, dict):
            self._last_sensor_state = sensor_state

    def adjust_for_terrain(self, terrain_type: str) -> None:
        """Adjust controller parameters based on terrain.

        For example, switch to a crawling gait on loose rubble or
        increase thruster output underwater.  In this skeleton
        implementation, the method performs no action.

        Parameters
        ----------
        terrain_type : str
            A categorical descriptor of the terrain (e.g. ``"rocky"``,
            ``"sandy"``, ``"underwater"``, ``"microgravity"``).
        """
        # TODO: implement terrain‑aware parameter adjustments
        return
