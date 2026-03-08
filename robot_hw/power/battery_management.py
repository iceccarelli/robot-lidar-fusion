"""
Battery Management Module
=========================

This module defines classes to track and manage the robot's battery
status.  Robots typically rely on rechargeable batteries whose
voltage, current draw, temperature and state of charge determine how
long the robot can operate safely.  The `BatteryManager` provides
APIs to update the current state, report whether the battery is
within safe operating limits and predict remaining runtime based on
observed current draw and the total capacity.

All values are expressed in SI units where applicable.  For example,
voltage in volts, current in amperes, temperature in degrees Celsius
and time in seconds.  The state of charge (SOC) and state of health
(SOH) are represented as ratios in the range 0..1.

The module intentionally avoids any direct I/O operations such as
reading from ADCs or communicating with battery management systems.
Those responsibilities belong to the hardware interface.  Instead,
callers should construct a :class:`BatteryState` with measurements
obtained from the hardware and pass it to the `BatteryManager`.
"""

from __future__ import annotations

import contextlib
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
    robot can continue operating safely.  It should be updated at least
    once per control cycle with the latest measurements.  Based on the
    state of charge and discharge rate, it can provide an estimate of
    remaining runtime.

    Parameters
    ----------
    capacity_wh : float
        Rated capacity of the battery in watt‑hours.  Used for
        runtime prediction.
    low_soc_threshold : float, optional
        Minimum allowable state of charge (0..1) before the manager
        flags a low‑battery condition.  Defaults to 0.2 (20 %).
    """

    def __init__(self, capacity_wh: float, low_soc_threshold: float = 0.2) -> None:
        self.capacity_wh = max(capacity_wh, 0.0)
        self.low_soc_threshold = max(min(low_soc_threshold, 1.0), 0.0)
        # Support a single aggregated battery state as before
        self.state: BatteryState | None = None
        # Maintain states of multiple packs; if non‑empty, the aggregate
        # ``state`` reflects their combined status (averaged)
        self.pack_states: list[BatteryState] = []
        # Internal history to compute discharge rate for prediction
        self._last_state: BatteryState | None = None
        # Regenerative braking callbacks (called with recovered energy in Wh)
        self._regen_callbacks: list[Callable[[float], None]] = []

    def update(self, state: BatteryState) -> None:
        """Update the battery manager with a new state.

        When called, the manager stores the previous state and records the
        new state.  The difference between successive states is used for
        estimating the discharge rate and remaining runtime.

        Parameters
        ----------
        state : BatteryState
            The latest measurement of battery voltage, current, SOC and
            temperature.
        """
        self._last_state = self.state
        self.state = state
        # When updating a single state, also refresh pack_states if empty
        if not self.pack_states:
            self.pack_states = [state]

    # ------------------------------------------------------------------
    # Multiple battery pack support
    # ------------------------------------------------------------------
    def update_packs(self, states: list[BatteryState]) -> None:
        """Update the manager with a list of battery pack states.

        When multiple battery packs are used, this method should be
        called once per cycle with the state of each pack.  The
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
        # Compute aggregated state by averaging values
        n = len(states)
        try:
            avg_voltage = sum(s.voltage for s in states) / n
            avg_current = sum(s.current for s in states) / n
            avg_temperature = sum(s.temperature for s in states) / n
            avg_soc = sum(s.soc for s in states) / n
            avg_health = sum(s.health for s in states) / n
            # Use the latest timestamp among packs
            latest_ts = max(s.timestamp for s in states)
        except Exception:
            return
        aggregated = BatteryState(voltage=avg_voltage,
                                  current=avg_current,
                                  temperature=avg_temperature,
                                  soc=avg_soc,
                                  health=avg_health,
                                  timestamp=latest_ts)
        self._last_state = self.state
        self.state = aggregated

    def is_ok(self) -> bool:
        """Return True if the battery is within safe operating limits.

        The battery is considered OK if the state has been set, the SOC
        is above the low threshold and the temperature remains within
        reasonable bounds (e.g., below 60 °C).  Temperature checks can
        be refined based on manufacturer specifications.
        """
        if self.state is None:
            return False
        # Evaluate each pack individually if present
        packs = self.pack_states if self.pack_states else [self.state]
        for pack in packs:
            if pack.soc < self.low_soc_threshold:
                return False
            # Temperature bounds (assumed same across packs)
            if not (-10.0 <= pack.temperature <= 60.0):
                return False
        return True

    def predict_runtime(self) -> float | None:
        """Estimate remaining runtime in seconds.

        Two estimation strategies are used:

        * **SOC trend:** Using the change in state of charge between
          successive measurements and the elapsed time, compute how
          quickly the battery is discharging.  Predict how long until
          the state of charge reaches the low‑SOC threshold.

        * **Instantaneous power:** Compute the remaining energy
          (capacity × SOC × SOH) and divide by the instantaneous
          discharge power (``voltage × current``) converted to
          watt‑hours per second (i.e. watts).  This approach may be
          more accurate if the discharge rate is stable but SOC
          measurement noise is significant.

        If both predictions are available, the smaller (more
        conservative) estimate is returned.  If insufficient data is
        available, returns ``None``.
        """
        if self.state is None:
            return None
        predictions: list[float] = []
        # SOC trend based prediction
        if self._last_state is not None:
            dt = self.state.timestamp - self._last_state.timestamp
            if dt > 0:
                delta_soc = self.state.soc - self._last_state.soc
                if delta_soc < 0:
                    discharge_rate = -delta_soc / dt  # SOC per second
                    if discharge_rate > 0:
                        remaining_soc = max(self.state.soc - self.low_soc_threshold, 0.0)
                        predictions.append(remaining_soc / discharge_rate)
        # Instantaneous power based prediction
        try:
            # Compute remaining energy in watt‑hours
            remaining_energy_wh = self.capacity_wh * self.state.soc * self.state.health
            # Compute current power consumption (W); positive current means discharge
            power_w = self.state.voltage * max(self.state.current, 0.0)
            if power_w > 0:
                # Convert watt‑hours to joules (Wh → J) then divide by watts (J/s) yields seconds
                # Simplify: remaining_time (h) = remaining_energy_wh / power_w, then multiply by 3600 to get seconds
                predictions.append((remaining_energy_wh / power_w) * 3600)
        except Exception:
            pass
        if not predictions:
            return None
        # Return the most conservative (minimum) estimate
        return min(predictions)

    # ------------------------------------------------------------------
    # State‑of‑health estimation and regenerative braking
    # ------------------------------------------------------------------
    def estimate_soh(self) -> float | None:
        """Estimate the average state of health (SOH) of the battery packs.

        Returns ``None`` if no state is available.
        """
        if self.state is None:
            return None
        if self.pack_states:
            try:
                return sum(p.health for p in self.pack_states) / len(self.pack_states)
            except Exception:
                pass
        return self.state.health

    def apply_regenerative_braking(self, energy_recovered: float) -> None:
        """Apply energy recovered through regenerative braking.

        This method increases the state of charge of the battery by
        distributing the recovered energy across the packs.  A simple
        proportional model is used: each pack's SOC is increased by
        ``energy_recovered / (capacity_wh * health * n_packs)``.  The
        SOC is capped at 1.0.  After updating, registered callbacks
        are invoked with the amount of energy recovered.

        Parameters
        ----------
        energy_recovered : float
            Energy recovered in watt‑hours.
        """
        if energy_recovered <= 0.0 or self.state is None:
            return
        packs = self.pack_states if self.pack_states else [self.state]
        n = len(packs)
        for _i, pack in enumerate(packs):
            try:
                # Compute SOC increment proportionally to pack health
                inc = energy_recovered / (self.capacity_wh * max(pack.health, 1e-3) * n)
                new_soc = min(pack.soc + inc, 1.0)
                # Update dataclass field directly (mutable dataclass)
                pack.soc = new_soc
            except Exception:
                continue
        # Recompute aggregate state
        if self.pack_states:
            self.update_packs(self.pack_states)
        else:
            self.state.soc = min(self.state.soc + energy_recovered / (self.capacity_wh * max(self.state.health, 1e-3)), 1.0)
        # Invoke callbacks
        for cb in list(self._regen_callbacks):
            with contextlib.suppress(Exception):
                cb(energy_recovered)

    def register_regenerative_brake_callback(self, callback: Callable[[float], None]) -> None:
        """Register a callback to be invoked when regenerative energy is applied.

        Parameters
        ----------
        callback : callable
            Function that accepts a single float (energy recovered in Wh).
        """
        if callable(callback):
            self._regen_callbacks.append(callback)

    # ------------------------------------------------------------------
    # Advanced energy estimation and scheduling helpers
    # ------------------------------------------------------------------
    def estimate_task_energy(self, instructions: list[Any], current_state: dict[str, Any] | None = None) -> float:
        """Estimate the energy required to execute a set of joint instructions.

        This helper computes a rough energy cost for moving the joints
        from their current positions to the commanded positions and
        applying velocities/torques.  The estimate assumes a linear
        energy model: moving one radian (or metre) consumes ``0.1`` Wh,
        commanding a velocity consumes ``0.05`` Wh per unit and
        commanding torque consumes ``0.02`` Wh per unit.  If
        ``current_state`` is provided, the current positions are
        extracted; otherwise a cost proportional to the magnitude of
        the target positions is used.

        Parameters
        ----------
        instructions : list
            Sequence of joint instructions (e.g. a list of
            :class:`JointInstruction` objects or dictionaries with
            ``joint_id`` and ``command`` keys).  Each command must
            specify at least a ``position``.
        current_state : dict, optional
            Mapping of joint IDs to their current positions.  If
            absent, positions are assumed to be zero.

        Returns
        -------
        float
            Estimated energy in watt‑hours required to execute the
            instructions.
        """
        energy = 0.0
        curr_pos: dict[str, float] = {}
        if current_state and isinstance(current_state.get("positions"), dict):
            curr_pos = current_state.get("positions", {})
        for inst in instructions:
            if inst is None:
                continue
            # Support both dataclass and dict styles
            if hasattr(inst, "joint_id") and hasattr(inst, "command"):
                jid = inst.joint_id
                cmd = inst.command
            elif isinstance(inst, dict):
                jid = inst.get("joint_id")
                cmd = inst.get("command")
            else:
                continue
            if not isinstance(cmd, dict):
                continue
            # Position contribution
            pos = cmd.get("position")
            try:
                target = float(pos) if pos is not None else 0.0
            except Exception:
                target = 0.0
            start = 0.0
            # Ensure joint_id is a valid string key before accessing curr_pos
            if isinstance(jid, str):
                try:
                    start = float(curr_pos.get(jid, 0.0))
                except Exception:
                    start = 0.0
            distance = abs(target - start)
            energy += 0.1 * distance
            # Velocity contribution
            vel = cmd.get("velocity")
            if vel is not None:
                with contextlib.suppress(Exception):
                    energy += 0.05 * abs(float(vel))
            # Torque contribution
            torque = cmd.get("torque")
            if torque is not None:
                with contextlib.suppress(Exception):
                    energy += 0.02 * abs(float(torque))
        return energy

    def should_defer_task(self, energy_required: float) -> bool:
        """Decide whether to defer a task based on its estimated energy.

        The available energy budget is computed as the difference
        between the current state of charge and the low‑SOC threshold,
        multiplied by the battery capacity (Wh).  If the task energy
        exceeds this budget, ``True`` is returned to indicate that the
        task should be deferred.  This method requires that a
        ``BatteryState`` has been set via ``update()``; otherwise it
        returns ``False``.

        Parameters
        ----------
        energy_required : float
            Estimated energy consumption in Wh.

        Returns
        -------
        bool
            True if the task should be deferred due to low available
            energy, False otherwise.
        """
        if self.state is None:
            return False
        available_soc = max(0.0, self.state.soc - self.low_soc_threshold)
        available_energy = available_soc * self.capacity_wh
        return energy_required > available_energy
