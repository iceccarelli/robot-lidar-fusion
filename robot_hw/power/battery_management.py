"""
Battery Management Module
=========================

This module defines classes to track and manage the robot's battery
status. Robots typically rely on rechargeable batteries whose
voltage, current draw, temperature and state of charge determine how
long the robot can operate safely. The `BatteryManager` provides
APIs to update the current state, report whether the battery is
within safe operating limits and predict remaining runtime based on
observed current draw and the total capacity.

All values are expressed in SI units where applicable. For example,
voltage in volts, current in amperes, temperature in degrees Celsius
and time in seconds. The state of charge (SOC) and state of health
(SOH) are represented as ratios in the range 0..1.

The module intentionally avoids any direct I/O operations such as
reading from ADCs or communicating with battery management systems.
Those responsibilities belong to the hardware interface. Instead,
callers should construct a :class:`BatteryState` with measurements
obtained from the hardware and pass it to the `BatteryManager`.
"""

from __future__ import annotations

import contextlib
import logging
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any


@dataclass
class BatteryState:
    """Snapshot of battery measurements.

    Attributes
    ----------
    voltage : float
        The terminal voltage of the battery in volts.
    current : float
        The instantaneous current draw from the battery in amperes
        (positive when discharging, negative when charging).
    temperature : float
        The temperature of the battery pack in degrees Celsius.
    soc : float
        State of charge (0..1), where 1.0 indicates a full battery.
    health : float
        State of health (0..1), representing the remaining capacity
        relative to a new battery.
    timestamp : float
        Time at which the measurement was taken, expressed in
        seconds (e.g., epoch time or monotonic time).
    """

    voltage: float
    current: float
    temperature: float
    soc: float
    health: float
    timestamp: float


class BatteryManager:
    """Monitor and predict the battery status.

    The `BatteryManager` encapsulates logic for determining whether the
    robot can continue operating safely. It should be updated at least
    once per control cycle with the latest measurements. Based on the
    state of charge and discharge rate, it can provide an estimate of
    remaining runtime.

    Parameters
    ----------
    capacity_wh : float
        Rated capacity of the battery in watt-hours. Used for
        runtime prediction.
    low_soc_threshold : float, optional
        Minimum allowable state of charge (0..1) before the manager
        flags a low-battery condition. Defaults to 0.2 (20 %).
    """

    def __init__(self, capacity_wh: float, low_soc_threshold: float = 0.2) -> None:
        self.capacity_wh = max(capacity_wh, 0.0)
        self.low_soc_threshold = max(min(low_soc_threshold, 1.0), 0.0)
        self.state: BatteryState | None = None
        self.pack_states: list[BatteryState] = []
        self._last_state: BatteryState | None = None
        self._regen_callbacks: list[Callable[[float], None]] = []
        self._logger = logging.getLogger(self.__class__.__name__)

    def update(self, state: BatteryState) -> None:
        """Update the battery manager with a new state.

        When called, the manager stores the previous state and records the
        new state. The difference between successive states is used for
        estimating the discharge rate and remaining runtime.

        Parameters
        ----------
        state : BatteryState
            The latest measurement of battery voltage, current, SOC and
            temperature.
        """
        self._last_state = self.state
        self.state = state
        if not self.pack_states:
            self.pack_states = [state]

    def update_packs(self, states: list[BatteryState]) -> None:
        """Update the manager with a list of battery pack states.

        When multiple battery packs are used, this method should be
        called once per cycle with the state of each pack. The
        aggregated ``state`` attribute is updated to reflect the
        average of the packs' measurements (SOC, voltage, current,
        temperature and health).

        Parameters
        ----------
        states : list of BatteryState
            Individual pack measurements.
        """
        if not states:
            return
        self.pack_states = list(states)
        pack_count = len(states)
        try:
            avg_voltage = sum(state.voltage for state in states) / pack_count
            avg_current = sum(state.current for state in states) / pack_count
            avg_temperature = sum(state.temperature for state in states) / pack_count
            avg_soc = sum(state.soc for state in states) / pack_count
            avg_health = sum(state.health for state in states) / pack_count
            latest_ts = max(state.timestamp for state in states)
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self._logger.debug("Failed to aggregate battery pack states: %s", exc)
            return
        aggregated = BatteryState(
            voltage=avg_voltage,
            current=avg_current,
            temperature=avg_temperature,
            soc=avg_soc,
            health=avg_health,
            timestamp=latest_ts,
        )
        self._last_state = self.state
        self.state = aggregated

    def is_ok(self) -> bool:
        """Return True if the battery is within safe operating limits.

        The battery is considered OK if the state has been set, the SOC
        is above the low threshold and the temperature remains within
        reasonable bounds (e.g., below 60 °C). Temperature checks can
        be refined based on manufacturer specifications.
        """
        if self.state is None:
            return False
        packs = self.pack_states if self.pack_states else [self.state]
        for pack in packs:
            if pack.soc < self.low_soc_threshold:
                return False
            if not (-10.0 <= pack.temperature <= 60.0):
                return False
        return True

    def predict_runtime(self) -> float | None:
        """Estimate remaining runtime in seconds.

        Two estimation strategies are used:

        * **SOC trend:** Using the change in state of charge between
          successive measurements and the elapsed time, compute how
          quickly the battery is discharging. Predict how long until
          the state of charge reaches the low-SOC threshold.

        * **Instantaneous power:** Compute the remaining energy
          (capacity × SOC × SOH) and divide by the instantaneous
          discharge power (``voltage × current``) converted to
          watt-hours per second (i.e. watts). This approach may be
          more accurate if the discharge rate is stable but SOC
          measurement noise is significant.

        If both predictions are available, the smaller (more
        conservative) estimate is returned. If insufficient data is
        available, returns ``None``.
        """
        if self.state is None:
            return None
        predictions: list[float] = []
        if self._last_state is not None:
            dt = self.state.timestamp - self._last_state.timestamp
            if dt > 0:
                delta_soc = self.state.soc - self._last_state.soc
                if delta_soc < 0:
                    discharge_rate = -delta_soc / dt
                    if discharge_rate > 0:
                        remaining_soc = max(self.state.soc - self.low_soc_threshold, 0.0)
                        predictions.append(remaining_soc / discharge_rate)
        try:
            remaining_energy_wh = self.capacity_wh * self.state.soc * self.state.health
            power_w = self.state.voltage * max(self.state.current, 0.0)
            if power_w > 0:
                predictions.append((remaining_energy_wh / power_w) * 3600)
        except (TypeError, ValueError) as exc:
            self._logger.debug("Failed to compute instantaneous runtime prediction: %s", exc)
        if not predictions:
            return None
        return min(predictions)

    def estimate_soh(self) -> float | None:
        """Estimate the average state of health (SOH) of the battery packs.

        Returns ``None`` if no state is available.
        """
        if self.state is None:
            return None
        if self.pack_states:
            try:
                return sum(pack.health for pack in self.pack_states) / len(self.pack_states)
            except (TypeError, ValueError, ZeroDivisionError) as exc:
                self._logger.debug("Failed to estimate pack SOH: %s", exc)
        return self.state.health

    def apply_regenerative_braking(self, energy_recovered: float) -> None:
        """Apply energy recovered through regenerative braking.

        This method increases the state of charge of the battery by
        distributing the recovered energy across the packs. A simple
        proportional model is used: each pack's SOC is increased by
        ``energy_recovered / (capacity_wh * health * n_packs)``. The
        SOC is capped at 1.0. After updating, registered callbacks
        are invoked with the amount of energy recovered.

        Parameters
        ----------
        energy_recovered : float
            Energy recovered in watt-hours.
        """
        if energy_recovered <= 0.0 or self.state is None:
            return
        packs = self.pack_states if self.pack_states else [self.state]
        pack_count = len(packs)
        for index, pack in enumerate(packs):
            try:
                increment = energy_recovered / (
                    self.capacity_wh * max(pack.health, 1e-3) * pack_count
                )
                pack.soc = min(pack.soc + increment, 1.0)
            except (TypeError, ValueError, ZeroDivisionError) as exc:
                self._logger.debug(
                    "Skipping regenerative update for pack %s due to invalid data: %s",
                    index,
                    exc,
                )
                continue
        if self.pack_states:
            self.update_packs(self.pack_states)
        else:
            self.state.soc = min(
                self.state.soc
                + energy_recovered / (self.capacity_wh * max(self.state.health, 1e-3)),
                1.0,
            )
        for callback in list(self._regen_callbacks):
            with contextlib.suppress(Exception):
                callback(energy_recovered)

    def register_regenerative_brake_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback to be invoked when regenerative energy is applied.

        Parameters
        ----------
        callback : callable
            Function that accepts a single float (energy recovered in Wh).
        """
        if callable(callback):
            self._regen_callbacks.append(callback)

    def estimate_task_energy(
        self, instructions: list[Any], current_state: dict[str, Any] | None = None
    ) -> float:
        """Estimate the energy required to execute a set of joint instructions.

        This helper computes a rough energy cost for moving the joints
        from their current positions to the commanded positions and
        applying velocities/torques. The estimate assumes a linear
        energy model: moving one radian (or metre) consumes ``0.1`` Wh,
        commanding a velocity consumes ``0.05`` Wh per unit and
        commanding torque consumes ``0.02`` Wh per unit. If
        ``current_state`` is provided, the current positions are
        extracted; otherwise a cost proportional to the magnitude of
        the target positions is used.

        Parameters
        ----------
        instructions : list
            Sequence of joint instructions (e.g. a list of
            :class:`JointInstruction` objects or dictionaries with
            ``joint_id`` and ``command`` keys). Each command must
            specify at least a ``position``.
        current_state : dict, optional
            Mapping of joint IDs to their current positions. If
            absent, positions are assumed to be zero.

        Returns
        -------
        float
            Estimated energy in watt-hours required to execute the
            instructions.
        """
        energy = 0.0
        current_positions: dict[str, float] = {}
        if current_state and isinstance(current_state.get("positions"), dict):
            current_positions = current_state.get("positions", {})
        for instruction in instructions:
            if instruction is None:
                continue
            if hasattr(instruction, "joint_id") and hasattr(instruction, "command"):
                joint_id = instruction.joint_id
                command = instruction.command
            elif isinstance(instruction, dict):
                joint_id = instruction.get("joint_id")
                command = instruction.get("command")
            else:
                continue
            if not isinstance(command, dict):
                continue

            position = command.get("position")
            try:
                target = float(position) if position is not None else 0.0
            except (TypeError, ValueError):
                target = 0.0

            start = 0.0
            if isinstance(joint_id, str):
                try:
                    start = float(current_positions.get(joint_id, 0.0))
                except (TypeError, ValueError):
                    start = 0.0
            distance = abs(target - start)
            energy += 0.1 * distance

            velocity = command.get("velocity")
            if velocity is not None:
                with contextlib.suppress(Exception):
                    energy += 0.05 * abs(float(velocity))

            torque = command.get("torque")
            if torque is not None:
                with contextlib.suppress(Exception):
                    energy += 0.02 * abs(float(torque))
        return energy

    def should_defer_task(self, energy_required: float) -> bool:
        """Decide whether to defer a task based on its estimated energy.

        The available energy budget is computed as the difference
        between the current state of charge and the low-SOC threshold,
        multiplied by the battery capacity (Wh). If the task energy
        exceeds this budget, ``True`` is returned to indicate that the
        task should be deferred. This method requires that a
        ``BatteryState`` has been set via ``update()``; otherwise it
        returns ``False``.
        """
        if self.state is None:
            return False
        available_soc = max(0.0, self.state.soc - self.low_soc_threshold)
        available_energy = available_soc * self.capacity_wh
        return energy_required > available_energy
