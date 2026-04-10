"""
Thermal Management Module
=========================

This module defines a `ThermalManager` class to monitor and control
the thermal state of the robot's components. Robots generate heat
during operation, especially at motors, controllers and power
electronics. Excessive heat can damage components or reduce
performance. The thermal manager tracks temperatures and suggests
cooling actions before critical thresholds are reached.

The class exposes simple APIs to update temperatures, check whether
all components are within safe limits and compute recommended
cooling duties. More advanced models (e.g. thermal prediction
models or PID controllers for fans) can be integrated later.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any


class ThermalManager:
    """Monitor temperatures, predict thermal trends and suggest cooling actions.

    The :class:`ThermalManager` encapsulates the thermal state of the robot and
    provides helpers to estimate the thermal impact of tasks, predict how
    temperatures will evolve in the near future and recommend cooling
    strategies. It supports environment-specific thermal limits and
    optional callbacks to activate cooling hardware.

    Parameters
    ----------
    max_temp : float
        Maximum safe temperature (in Celsius) for any component. This can be
        environment specific (e.g. lower in space due to lack of convection).
    cooling_hysteresis : float, optional
        Temperature margin below the maximum at which cooling is relaxed.
        Defaults to 5 °C. A larger hysteresis reduces cooling cycling.
    """

    def __init__(self, max_temp: float, cooling_hysteresis: float = 5.0) -> None:
        self.max_temp = max_temp
        self.cooling_hysteresis = cooling_hysteresis
        self.current_temps: dict[str, float] = {}
        self._history: list[dict[str, float]] = []
        self._cooling_callbacks: list[Callable[[dict[str, float]], None]] = []
        self._logger = logging.getLogger(self.__class__.__name__)

    @staticmethod
    def _coerce_float(value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def update(self, temps: dict[str, float]) -> None:
        """Update the current temperatures and record them for trend analysis.

        The provided temperature mapping replaces the previously stored
        temperatures. A copy of the mapping is appended to the internal
        history for simple trend prediction. To avoid unbounded growth,
        callers may clear the history periodically or pass an optional
        ``retain`` parameter in future extensions.

        Parameters
        ----------
        temps : dict[str, float]
            Mapping from component identifier to temperature in
            degrees Celsius.
        """
        self.current_temps = dict(temps)
        self._history.append(self.current_temps.copy())
        max_history = 20
        if len(self._history) > max_history:
            self._history = self._history[-max_history:]

    def is_within_limits(self) -> bool:
        """Return True if all components are below the maximum temperature.

        Components that report invalid numeric values or are missing are
        considered unsafe.
        """
        for temp in self.current_temps.values():
            parsed = self._coerce_float(temp)
            if parsed is None:
                return False
            if parsed > self.max_temp:
                return False
        return True

    def recommended_cooling(self) -> dict[str, float]:
        """Suggest cooling duty cycles and trigger cooling callbacks.

        This method computes a duty cycle for each component based on
        its temperature relative to the allowable range. It uses a
        linear rule: no cooling if below ``max_temp - hysteresis``, full
        cooling if at or above ``max_temp`` and proportional in
        between. After computing the mapping, registered cooling
        callbacks are invoked. The mapping is returned to the caller.

        Returns
        -------
        dict[str, float]
            Duty cycles for each component (0.0 – 1.0).
        """
        duties: dict[str, float] = {}
        threshold = self.max_temp - self.cooling_hysteresis
        for comp, temp in self.current_temps.items():
            parsed = self._coerce_float(temp)
            if parsed is None:
                duties[comp] = 1.0
                continue
            if parsed <= threshold:
                duty = 0.0
            elif parsed >= self.max_temp:
                duty = 1.0
            else:
                duty = (parsed - threshold) / self.cooling_hysteresis
            duties[comp] = min(max(duty, 0.0), 1.0)

        for callback in list(self._cooling_callbacks):
            try:
                callback(duties)
            except Exception as exc:
                self._logger.debug("Cooling callback failed: %s", exc)
        return duties

    def estimate_task_thermal_load(
        self, instructions: list[Any], current_state: dict[str, Any] | None = None
    ) -> float:
        """Estimate the thermal load of executing a set of joint instructions.

        A simplified thermal model is used: the predicted temperature
        increase is proportional to the magnitude of position change
        (1 °C per rad/metre) plus contributions from commanded
        velocities (0.5 °C per unit) and torques (0.3 °C per unit). In
        practice, thermal models should be calibrated against
        measurements for each actuator.

        Parameters
        ----------
        instructions : list
            Sequence of joint instructions (e.g. :class:`JointInstruction`
            objects or dicts). Each command may specify ``position``,
            ``velocity`` and ``torque``.
        current_state : dict, optional
            Current positions of joints; if absent, starting positions are
            assumed to be zero.

        Returns
        -------
        float
            Estimated maximum temperature rise (°C) across all joints
            resulting from the task.
        """
        max_rise = 0.0
        curr_pos: dict[str, float] = {}
        if current_state and isinstance(current_state.get("positions"), dict):
            curr_pos = current_state.get("positions", {})
        for inst in instructions:
            if inst is None:
                continue
            if hasattr(inst, "joint_id") and hasattr(inst, "command"):
                jid = inst.joint_id
                cmd = inst.command
            elif isinstance(inst, dict):
                jid = inst.get("joint_id")
                cmd = inst.get("command")
            else:
                continue
            if not isinstance(jid, str) or not isinstance(cmd, dict):
                continue

            target = self._coerce_float(cmd.get("position"))
            if target is None:
                target = 0.0
            start = self._coerce_float(curr_pos.get(jid, 0.0))
            if start is None:
                start = 0.0
            rise = abs(target - start)

            velocity = self._coerce_float(cmd.get("velocity"))
            if velocity is not None:
                rise += 0.5 * abs(velocity)

            torque = self._coerce_float(cmd.get("torque"))
            if torque is not None:
                rise += 0.3 * abs(torque)

            if rise > max_rise:
                max_rise = rise
        return max_rise

    def should_throttle(self, thermal_load: float) -> bool:
        """Determine whether a task should be throttled based on thermal load.

        This method checks if adding ``thermal_load`` to any current
        temperature would exceed ``self.max_temp - self.cooling_hysteresis``.
        If so, the task should be executed at reduced speed or deferred
        until components have cooled. Requires that temperatures have
        been updated via ``update()``.

        Parameters
        ----------
        thermal_load : float
            Estimated temperature increase in °C from executing the task.

        Returns
        -------
        bool
            True if the task should be throttled, False otherwise.
        """
        if not self.current_temps:
            return False
        threshold = self.max_temp - self.cooling_hysteresis
        for temp in self.current_temps.values():
            parsed = self._coerce_float(temp)
            if parsed is None:
                continue
            if parsed + thermal_load > threshold:
                return True
        return False

    def predict_future_temps(self, thermal_load: float) -> dict[str, float]:
        """Predict future temperatures after applying a thermal load.

        This helper estimates the temperatures of components after
        executing a task that is expected to raise temperatures by
        ``thermal_load`` degrees Celsius. The simplest model adds the
        load to current temperatures. More sophisticated models could
        consider thermal dynamics and cooling effects by analysing
        ``self._history``.

        Parameters
        ----------
        thermal_load : float
            Estimated temperature increase (°C) from a task across all
            components.

        Returns
        -------
        dict[str, float]
            Predicted temperatures for each component.
        """
        predictions: dict[str, float] = {}
        for comp, temp in self.current_temps.items():
            parsed = self._coerce_float(temp)
            if parsed is None:
                predictions[comp] = self.max_temp + thermal_load
                continue
            predictions[comp] = parsed + thermal_load
        return predictions

    def register_cooling_callback(self, callback: Callable[[dict[str, float]], None]) -> None:
        """Register a callback to be invoked when cooling duties are computed.

        Callbacks should be non-blocking; they will be invoked with the
        mapping returned by :meth:`recommended_cooling`.

        Parameters
        ----------
        callback : callable
            Function that accepts a dict of duty cycles keyed by component.
        """
        if callable(callback):
            self._cooling_callbacks.append(callback)
