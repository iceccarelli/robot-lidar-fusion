"""
Consistency Verification Module
===============================

This module defines the `ConsistencyVerifier` class, which performs
cross‑module consistency checks on the robot's state.  In a complex
control system, discrepancies can arise between commanded and measured
values (e.g. due to sensor noise, lost packets or bugs).  The
consistency verifier inspects such discrepancies and reports
violations so that corrective action can be taken.

The verifier is generic: it accepts a state dictionary and applies
configured rules to detect anomalies.  Examples include:

* Missing mandatory fields (position, velocity, timestamp).
* Timestamps out of order or jitter beyond a threshold.
* Commanded values exceeding safety limits.
* Sensor readings outside physically plausible ranges.

An instance of this class should be reused across cycles; it stores
errors detected in the last check in the `errors` list.
"""

from __future__ import annotations

from typing import Any


class ConsistencyVerifier:
    """Check state for inconsistencies and violations.

    Attributes
    ----------
    errors : list[str]
        A list of human‑readable error messages describing the
        inconsistencies detected in the most recent verification.
    """

    def __init__(self) -> None:
        # A list of human-readable error messages from the last check.
        self.errors: list[str] = []
        # Track the last timestamp to verify monotonicity across cycles.
        self._last_timestamp: float | None = None

    def verify(self, state: dict[str, Any]) -> bool:
        """Verify consistency of the provided state.

        Parameters
        ----------
        state : dict[str, Any]
            Arbitrary state dictionary containing sensor readings,
            commanded values and timestamps.  The expected keys and
            value ranges are system dependent.

        Returns
        -------
        bool
            True if the state is considered consistent; False if any
            inconsistencies are detected.  Detected issues are
            recorded in the `errors` attribute.

        Notes
        -----
        This default implementation performs only minimal checks:
        verifies presence of a monotonic timestamp and non‑null
        position fields.  More sophisticated rules should be
        implemented according to system requirements.
        """
        self.errors.clear()
        # Check timestamp
        ts = state.get("timestamp")
        if ts is None:
            self.errors.append("Missing timestamp in state")
        else:
            try:
                ts_val = float(ts)
            except Exception:
                self.errors.append(f"Invalid timestamp: {ts}")
            else:
                # Verify monotonicity if we have a previous timestamp
                if self._last_timestamp is not None and ts_val <= self._last_timestamp:
                    self.errors.append(
                        f"Timestamp {ts_val} not greater than previous {self._last_timestamp}"
                    )
                self._last_timestamp = ts_val
        # Check positions
        positions = state.get("positions")
        if positions is None:
            self.errors.append("Missing positions in state")
        else:
            for joint_id, pos in positions.items():
                try:
                    _ = float(pos)
                except Exception:
                    self.errors.append(f"Invalid position for joint {joint_id}: {pos}")
        # Check velocities if present
        velocities = state.get("velocities")
        if velocities is not None:
            for joint_id, vel in velocities.items():
                try:
                    _ = float(vel)
                except Exception:
                    self.errors.append(f"Invalid velocity for joint {joint_id}: {vel}")
        # Check torques if present
        torques = state.get("torques")
        if torques is not None:
            for joint_id, tor in torques.items():
                try:
                    _ = float(tor)
                except Exception:
                    self.errors.append(f"Invalid torque for joint {joint_id}: {tor}")

        # Optional checks on environmental hazard fields
        # ``proximity`` should be numeric if present
        prox = state.get("proximity")
        if prox is not None:
            try:
                _ = float(prox)
            except Exception:
                self.errors.append(f"Invalid proximity value: {prox}")
        # ``pedestrian`` should be boolean if present
        ped = state.get("pedestrian")
        if ped is not None and not isinstance(ped, bool):
            self.errors.append(f"Invalid pedestrian flag (expected bool): {ped}")
        # ``hazards`` should be a dictionary if present
        hazards = state.get("hazards")
        if hazards is not None and not isinstance(hazards, dict):
            self.errors.append(f"Invalid hazards field (expected dict): {hazards}")

        # Orientation should be a list/tuple of 3 or 4 numeric values
        orientation = state.get("orientation")
        if orientation is not None:
            if not isinstance(orientation, (list, tuple)):
                self.errors.append(f"Invalid orientation type (expected list/tuple): {orientation}")
            else:
                if len(orientation) not in (3, 4):
                    self.errors.append(f"Orientation must have 3 or 4 elements: {orientation}")
                else:
                    for i, val in enumerate(orientation):
                        try:
                            _ = float(val)
                        except Exception:
                            self.errors.append(f"Invalid orientation component {i}: {val}")

        # Linear velocity should be a list/tuple of 3 numeric values
        lin_vel = state.get("linear_velocity")
        if lin_vel is not None:
            if not isinstance(lin_vel, (list, tuple)) or len(lin_vel) != 3:
                self.errors.append(f"Invalid linear_velocity (expected 3‑tuple): {lin_vel}")
            else:
                for i, val in enumerate(lin_vel):
                    try:
                        _ = float(val)
                    except Exception:
                        self.errors.append(f"Invalid linear_velocity component {i}: {val}")

        # Acceleration should be a list/tuple of 3 numeric values
        acc = state.get("acceleration")
        if acc is not None:
            if not isinstance(acc, (list, tuple)) or len(acc) != 3:
                self.errors.append(f"Invalid acceleration (expected 3‑tuple): {acc}")
            else:
                for i, val in enumerate(acc):
                    try:
                        _ = float(val)
                    except Exception:
                        self.errors.append(f"Invalid acceleration component {i}: {val}")

        # Hazard flags should be a dictionary if present
        hazard_flags = state.get("hazard_flags")
        if hazard_flags is not None and not isinstance(hazard_flags, dict):
            self.errors.append(f"Invalid hazard_flags field (expected dict): {hazard_flags}")

        # Stage 5 navigation and mapping outputs
        map_summary = state.get("map")
        if map_summary is not None:
            if not isinstance(map_summary, dict):
                self.errors.append(f"Invalid map field (expected dict): {map_summary}")
            else:
                for key in ("occupied_cells", "lethal_cells", "inflated_cells"):
                    value = map_summary.get(key)
                    if value is not None:
                        try:
                            _ = int(value)
                        except Exception:
                            self.errors.append(f"Invalid map metric '{key}': {value}")

        nav2_costmap = state.get("nav2_costmap")
        if nav2_costmap is not None:
            if not isinstance(nav2_costmap, dict):
                self.errors.append(f"Invalid nav2_costmap field (expected dict): {nav2_costmap}")
            else:
                metadata = nav2_costmap.get("metadata")
                data = nav2_costmap.get("data")
                if not isinstance(metadata, dict):
                    self.errors.append(f"Invalid nav2_costmap metadata: {metadata}")
                if not isinstance(data, list):
                    self.errors.append(f"Invalid nav2_costmap data payload: {data}")

        for field_name in ("global_plan", "local_plan"):
            plan = state.get(field_name)
            if plan is not None:
                if not isinstance(plan, list):
                    self.errors.append(f"Invalid {field_name} field (expected list): {plan}")
                else:
                    for idx, waypoint in enumerate(plan):
                        if not isinstance(waypoint, (list, tuple)) or len(waypoint) < 2:
                            self.errors.append(
                                f"Invalid waypoint {idx} in {field_name}: {waypoint}"
                            )
                            continue
                        for coord_idx, value in enumerate(waypoint[:2]):
                            try:
                                _ = float(value)
                            except Exception:
                                self.errors.append(
                                    f"Invalid {field_name} waypoint component {idx}:{coord_idx}: {value}"
                                )

        navigation = state.get("navigation")
        if navigation is not None and not isinstance(navigation, dict):
            self.errors.append(f"Invalid navigation field (expected dict): {navigation}")

        locomotion_commands = state.get("locomotion_commands")
        if locomotion_commands is not None:
            if not isinstance(locomotion_commands, dict):
                self.errors.append(
                    f"Invalid locomotion_commands field (expected dict): {locomotion_commands}"
                )
            else:
                for joint_id, value in locomotion_commands.items():
                    try:
                        _ = float(value)
                    except Exception:
                        self.errors.append(
                            f"Invalid locomotion command for joint {joint_id}: {value}"
                        )

        return not self.errors
