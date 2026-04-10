"""Hazard Manager Module
======================

This module defines the :class:`HazardManager`, which monitors
environmental signals and internal states to detect potentially
dangerous conditions. It provides a centralised API for querying
whether the robot is operating safely and for obtaining structured
alerts about detected hazards.

Typical hazards include proximity to humans or obstacles, exposure to
gas or radiation, high voltage sources, extreme pressures or
temperatures and moving machinery. The hazard manager should aggregate
data from sensors, configuration thresholds and external systems to
make informed decisions.

In stage A this class is a skeleton; real sensor integration and
threshold checks will be implemented in later stages.
"""

from __future__ import annotations

import logging
from typing import Any

from robot_hw.robot_config import RobotConfig


class HazardManager:
    """Monitor environmental and internal hazards.

    Parameters
    ----------
    config : RobotConfig
        System configuration including safety thresholds and
        environment profile.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        self._hazards: dict[str, dict[str, Any]] = {}
        self._exposures: dict[str, int] = {}
        self._logger = logging.getLogger(self.__class__.__name__)

    def _get_threshold(self, idx: int, default: float = 1.0) -> float:
        """Return a configured numeric threshold with a safe fallback."""
        try:
            return float(self._config.safety_thresholds[idx])
        except (IndexError, TypeError, ValueError) as exc:
            self._logger.debug("Using default threshold for index %s: %s", idx, exc)
            return default

    @staticmethod
    def _classify_distance_risk(distance: float, threshold: float) -> tuple[float, str]:
        """Classify a distance-based hazard using distance/threshold severity."""
        safe_threshold = max(threshold, 1e-3)
        severity = distance / safe_threshold
        if severity < 0.5:
            return severity, "high"
        if severity < 1.0:
            return severity, "moderate"
        return severity, "low"

    @staticmethod
    def _classify_level_risk(level: float, threshold: float) -> tuple[float, str]:
        """Classify a level-based hazard using level/threshold severity."""
        safe_threshold = max(threshold, 1e-3)
        severity = level / safe_threshold
        if severity >= 1.5:
            return severity, "high"
        if severity >= 1.0:
            return severity, "moderate"
        return severity, "low"

    @staticmethod
    def _normalise_faults(faults: Any) -> list[str]:
        """Normalise fault inputs into a list of string identifiers."""
        if isinstance(faults, str):
            return [faults]
        if isinstance(faults, (list, tuple, set)):
            return [str(fault) for fault in faults]
        return []

    def _record_hazard(
        self,
        name: str,
        value: Any,
        threshold: Any,
        severity: float,
        risk_level: str,
        message: str,
    ) -> None:
        """Store a hazard entry in the current cycle dictionary."""
        self._hazards[name] = {
            "value": value,
            "threshold": threshold,
            "severity": severity,
            "risk_level": risk_level,
            "message": message,
        }

    def _apply_exposure_tracking(self) -> None:
        """Update exposure counters for hazards active in the current cycle."""
        current_keys = set(self._hazards.keys())
        for key in current_keys:
            self._exposures[key] = self._exposures.get(key, 0) + 1
            self._hazards[key]["exposure_cycles"] = self._exposures[key]
        for key in list(self._exposures.keys()):
            if key not in current_keys:
                del self._exposures[key]

    def _legacy_update(self, signals: dict[str, Any]) -> None:
        """Update the hazard state based on new signals.

        This method should be called once per control cycle. It
        evaluates incoming sensor signals against configured safety
        thresholds and populates the internal hazard dictionary with
        entries for any conditions deemed unsafe. Hazard detection
        logic implemented here supports the following inputs:

        * ``proximity``: distance to the nearest obstacle (metres).
          A hazard is flagged if the value is less than
          ``config.min_safety_margin``.
        * ``high_voltage``, ``gas``, ``radiation``: scalar values
          representing hazard levels. These are compared against
          indices 0, 1 and 2 of ``config.safety_thresholds``
          respectively. If no threshold is provided for a given
          hazard, a default threshold of 1.0 is used.
        * ``train``, ``car``, ``human``: distances to dynamic
          obstacles. Hazards are flagged if these distances are less
          than ``config.min_safety_margin``.
        * ``pedestrian``: boolean indicating whether a person is
          present in the immediate path. If True, a hazard is
          recorded.

        Parameters
        ----------
        signals : dict[str, Any]
            Raw or pre-processed signals relevant to hazard detection.
        """
        self._hazards.clear()
        if not isinstance(signals, dict):
            return

        prox = signals.get("proximity")
        if prox is not None:
            try:
                distance = float(prox)
                if distance < self._config.min_safety_margin:
                    self._hazards["proximity"] = {
                        "value": distance,
                        "threshold": self._config.min_safety_margin,
                        "message": (
                            f"Object within {distance:.2f} m "
                            f"(min {self._config.min_safety_margin} m)"
                        ),
                    }
            except (TypeError, ValueError) as exc:
                self._logger.debug("Invalid proximity value in legacy update: %s", exc)

        for idx, name in enumerate(["high_voltage", "gas", "radiation"]):
            value = signals.get(name)
            if value is None:
                continue
            try:
                level = float(value)
            except (TypeError, ValueError) as exc:
                self._logger.debug("Invalid %s level in legacy update: %s", name, exc)
                continue
            threshold = self._get_threshold(idx)
            if level > threshold:
                self._hazards[name] = {
                    "value": level,
                    "threshold": threshold,
                    "message": (
                        f"{name.replace('_', ' ').title()} level {level:.2f} "
                        f"exceeds {threshold:.2f}"
                    ),
                }

        dynamic_distances = {
            "train": signals.get("train"),
            "car": signals.get("car"),
            "human": signals.get("human"),
        }
        for name, value in dynamic_distances.items():
            if value is None:
                continue
            try:
                distance = float(value)
            except (TypeError, ValueError) as exc:
                self._logger.debug("Invalid %s distance in legacy update: %s", name, exc)
                continue
            if distance < self._config.min_safety_margin:
                self._hazards[name] = {
                    "value": distance,
                    "threshold": self._config.min_safety_margin,
                    "message": f"{name.title()} within {distance:.2f} m",
                }

        if signals.get("pedestrian"):
            self._hazards["pedestrian"] = {
                "value": True,
                "threshold": True,
                "message": "Pedestrian detected in path",
            }

        for fault_name in self._normalise_faults(signals.get("faults")):
            self._hazards[f"fault:{fault_name}"] = {
                "value": True,
                "threshold": True,
                "message": f"Fault detected: {fault_name}",
            }

    def is_safe(self) -> bool:
        """Return True if the robot can continue operating normally.

        Only hazards classified with a ``risk_level`` of ``"high"`` are
        considered unsafe. Hazards with ``"moderate"`` or ``"low"``
        risk indicate caution but do not mandate an emergency stop. If
        no hazards are present, the method returns ``True``. If any
        high-risk hazard exists or a hazard entry lacks a ``risk_level``
        classification, the method returns ``False``.
        """
        if not self._hazards:
            return True
        for info in self._hazards.values():
            if not isinstance(info, dict):
                self._logger.debug("Malformed hazard entry encountered: %r", info)
                return False
            risk = info.get("risk_level")
            if risk is None or str(risk).lower() == "high":
                return False
        return True

    def current_hazards(self) -> dict[str, Any]:
        """Return a dictionary of the currently detected hazards."""
        return dict(self._hazards)

    def alert_messages(self) -> list[str]:
        """Return human-readable alert messages for current hazards."""
        messages: list[str] = []
        for name, info in self._hazards.items():
            message = info.get("message") if isinstance(info, dict) else None
            if message:
                messages.append(str(message))
            else:
                messages.append(f"Hazard detected: {name}")
        return messages

    def update(self, signals: dict[str, Any]) -> None:
        """Update the hazard state based on new signals with additional metadata.

        This method supersedes the legacy update implementation by computing
        severity ratios, classifying risk levels and tracking exposure
        durations for each detected hazard.
        """
        self._hazards = {}
        if not isinstance(signals, dict):
            return

        proximity = signals.get("proximity")
        if proximity is not None:
            try:
                distance = float(proximity)
                if distance < self._config.min_safety_margin:
                    severity, risk = self._classify_distance_risk(
                        distance,
                        self._config.min_safety_margin,
                    )
                    self._record_hazard(
                        "proximity",
                        value=distance,
                        threshold=self._config.min_safety_margin,
                        severity=severity,
                        risk_level=risk,
                        message=(
                            f"Object within {distance:.2f} m "
                            f"(min {self._config.min_safety_margin} m)"
                        ),
                    )
            except (TypeError, ValueError) as exc:
                self._logger.debug("Error processing proximity hazard: %s", exc)

        for idx, hazard_name in enumerate(["high_voltage", "gas", "radiation"]):
            value = signals.get(hazard_name)
            if value is None:
                continue
            try:
                level = float(value)
            except (TypeError, ValueError) as exc:
                self._logger.debug("Invalid %s level: %s", hazard_name, exc)
                continue
            threshold = self._get_threshold(idx)
            if level > threshold:
                severity, risk = self._classify_level_risk(level, threshold)
                self._record_hazard(
                    hazard_name,
                    value=level,
                    threshold=threshold,
                    severity=severity,
                    risk_level=risk,
                    message=(
                        f"{hazard_name.replace('_', ' ').title()} level {level:.2f} "
                        f"exceeds {threshold:.2f} (severity {severity:.2f})"
                    ),
                )

        for obstacle_name in ["train", "car", "human"]:
            value = signals.get(obstacle_name)
            if value is None:
                continue
            try:
                distance = float(value)
            except (TypeError, ValueError) as exc:
                self._logger.debug("Invalid %s distance: %s", obstacle_name, exc)
                continue
            if distance < self._config.min_safety_margin:
                severity, risk = self._classify_distance_risk(
                    distance,
                    self._config.min_safety_margin,
                )
                self._record_hazard(
                    obstacle_name,
                    value=distance,
                    threshold=self._config.min_safety_margin,
                    severity=severity,
                    risk_level=risk,
                    message=f"{obstacle_name.title()} within {distance:.2f} m",
                )

        if signals.get("pedestrian"):
            self._record_hazard(
                "pedestrian",
                value=True,
                threshold=True,
                severity=2.0,
                risk_level="high",
                message="Pedestrian detected in path",
            )

        for fault_name in self._normalise_faults(signals.get("faults")):
            self._record_hazard(
                f"fault:{fault_name}",
                value=True,
                threshold=True,
                severity=2.0,
                risk_level="high",
                message=f"Fault detected: {fault_name}",
            )

        self._apply_exposure_tracking()
