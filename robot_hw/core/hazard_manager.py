"""Hazard Manager Module
======================

This module defines the :class:`HazardManager`, which monitors
environmental signals and internal states to detect potentially
dangerous conditions.  It provides a centralised API for querying
whether the robot is operating safely and for obtaining structured
alerts about detected hazards.

Typical hazards include proximity to humans or obstacles, exposure to
gas or radiation, high voltage sources, extreme pressures or
temperatures and moving machinery.  The hazard manager should
aggregate data from sensors, configuration thresholds and external
systems to make informed decisions.

In stage A this class is a skeleton; real sensor integration and
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
        # Internal storage for the latest hazards detected
        self._hazards: dict[str, Any] = {}
        # Track how many consecutive cycles each hazard has been active.
        # When a hazard disappears the count is reset.  Exposure
        # information can be used by planners or safety systems to
        # prioritise actions based on how long a hazard has persisted.
        self._exposures: dict[str, int] = {}
        # Logger for debugging hazard evaluation.  Logging exceptions here
        # should never interrupt real‑time processing; messages are
        # emitted at the DEBUG level.
        self._logger = logging.getLogger(self.__class__.__name__)

    def _legacy_update(self, signals: dict[str, Any]) -> None:
        """Update the hazard state based on new signals.

        This method should be called once per control cycle.  It
        evaluates incoming sensor signals against configured safety
        thresholds and populates the internal hazard dictionary with
        entries for any conditions deemed unsafe.  Hazard detection
        logic implemented here supports the following inputs:

        * ``proximity``: distance to the nearest obstacle (metres).
          A hazard is flagged if the value is less than
          ``config.min_safety_margin``.
        * ``high_voltage``, ``gas``, ``radiation``: scalar values
          representing hazard levels.  These are compared against
          indices 0, 1 and 2 of ``config.safety_thresholds``
          respectively.  If no threshold is provided for a given
          hazard, a default threshold of 1.0 is used.
        * ``train``, ``car``, ``human``: distances to dynamic
          obstacles.  Hazards are flagged if these distances are less
          than ``config.min_safety_margin``.
        * ``pedestrian``: boolean indicating whether a person is
          present in the immediate path.  If True, a hazard is
          recorded.

        Parameters
        ----------
        signals : dict[str, Any]
            Raw or pre‑processed signals relevant to hazard detection.
        """
        # Reset hazards each cycle
        self._hazards.clear()
        if not isinstance(signals, dict):
            return

        # Helper to retrieve threshold by index with fallback
        def _get_threshold(idx: int, default: float = 1.0) -> float:
            try:
                return float(self._config.safety_thresholds[idx])
            except Exception:
                return default

        # Proximity hazard
        prox = signals.get("proximity")
        if prox is not None:
            try:
                distance = float(prox)
                if distance < self._config.min_safety_margin:
                    self._hazards["proximity"] = {
                        "value": distance,
                        "threshold": self._config.min_safety_margin,
                        "message": f"Object within {distance:.2f} m (min {self._config.min_safety_margin} m)",
                    }
            except Exception:
                pass
        # Numeric hazards with explicit thresholds
        numeric_hazards = ["high_voltage", "gas", "radiation"]
        for idx, name in enumerate(numeric_hazards):
            val = signals.get(name)
            if val is None:
                continue
            try:
                level = float(val)
            except Exception:
                continue
            threshold = _get_threshold(idx)
            # For hazards measured as level (e.g. volts, ppm, mSv), flag if exceeding threshold
            if level > threshold:
                self._hazards[name] = {
                    "value": level,
                    "threshold": threshold,
                    "message": f"{name.replace('_', ' ').title()} level {level:.2f} exceeds {threshold:.2f}",
                }
        # Dynamic obstacle hazards (distances)
        dynamic = {
            "train": signals.get("train"),
            "car": signals.get("car"),
            "human": signals.get("human"),
        }
        for name, val in dynamic.items():
            if val is None:
                continue
            try:
                dist = float(val)
            except Exception:
                continue
            if dist < self._config.min_safety_margin:
                self._hazards[name] = {
                    "value": dist,
                    "threshold": self._config.min_safety_margin,
                    "message": f"{name.title()} within {dist:.2f} m",
                }
        # Pedestrian flag hazard
        ped = signals.get("pedestrian")
        if ped:
            self._hazards["pedestrian"] = {
                "value": True,
                "threshold": True,
                "message": "Pedestrian detected in path",
            }

        # Integrate fault flags as hazards.  The ``faults`` key may contain
        # a list of fault identifiers or a string.  Each fault is mapped to
        # a hazard entry to unify emergency stop logic.
        faults = signals.get("faults")
        if faults:
            # Accept a single string or iterable of strings
            if isinstance(faults, str):
                faults_iter = [faults]
            elif isinstance(faults, (list, tuple, set)):
                faults_iter = [str(f) for f in faults]
            else:
                faults_iter = []
            for fault_name in faults_iter:
                self._hazards[f"fault:{fault_name}"] = {
                    "value": True,
                    "threshold": True,
                    "message": f"Fault detected: {fault_name}",
                }

    def is_safe(self) -> bool:
        """Return True if the robot can continue operating normally.

        Only hazards classified with a ``risk_level`` of ``"high"`` are
        considered unsafe.  Hazards with ``"moderate"`` or ``"low"``
        risk indicate caution but do not mandate an emergency stop.  If
        no hazards are present, the method returns ``True``.  If any
        high‑risk hazard exists or a hazard entry lacks a ``risk_level``
        classification, the method returns ``False``.

        Returns
        -------
        bool
            ``True`` if the robot may continue operating; ``False`` if an
            immediate stop is required.
        """
        if not self._hazards:
            return True
        for info in self._hazards.values():
            # Each hazard entry should include a risk_level string.
            risk = None
            try:
                risk = info.get("risk_level")
            except Exception:
                # Malformed entry; treat as high risk
                return False
            # Treat unspecified risk as high to err on the side of safety
            if risk is None or str(risk).lower() == "high":
                return False
        return True

    def current_hazards(self) -> dict[str, Any]:
        """Return a dictionary of the currently detected hazards.

        Returns
        -------
        dict[str, Any]
            A mapping of hazard identifiers to descriptive data such as
            severity level, timestamp and sensor origin.
        """
        return dict(self._hazards)

    def alert_messages(self) -> list[str]:
        """Return human‑readable alert messages for current hazards.

        Each entry in the returned list is derived from the hazard
        dictionary populated by :meth:`update`.  The messages include
        the hazard name and a brief description of the condition.

        Returns
        -------
        list[str]
            A list of alert strings suitable for logging or operator
            notification.
        """
        messages: list[str] = []
        for name, info in self._hazards.items():
            msg = info.get("message") if isinstance(info, dict) else None
            if msg:
                messages.append(msg)
            else:
                messages.append(f"Hazard detected: {name}")
        return messages

    # ------------------------------------------------------------------
    # Enhanced hazard update with severity, risk and exposure tracking
    # ------------------------------------------------------------------
    def update(self, signals: dict[str, Any]) -> None:
        """Update the hazard state based on new signals with additional metadata.

        This method supersedes the legacy update implementation by computing
        severity ratios, classifying risk levels and tracking exposure
        durations (number of consecutive cycles) for each detected hazard.
        The method examines the following signals:

        * ``proximity`` (float): distance to the nearest object.  A hazard
          is recorded when the distance is below ``config.min_safety_margin``.
        * ``high_voltage``, ``gas``, ``radiation`` (float): measured hazard
          levels compared against thresholds in ``config.safety_thresholds``.
        * ``train``, ``car``, ``human`` (float): distances to dynamic
          obstacles compared against ``config.min_safety_margin``.
        * ``pedestrian`` (bool): indicates a person directly in the path.
        * ``faults`` (str or iterable[str]): identifiers for faults detected
          by the fault detector.  Faults are treated as hazards with high
          severity.

        Each hazard entry contains:
        ``value``, ``threshold``, ``severity`` (ratio value/threshold),
        ``risk_level`` (low/moderate/high), ``message`` and
        ``exposure_cycles`` (number of consecutive cycles the hazard has
        persisted).

        Parameters
        ----------
        signals : dict[str, Any]
            A mapping of signal names to their measured values.
        """
        # Initialise new hazard dictionary
        self._hazards = {}
        if not isinstance(signals, dict):
            return

        # Helper to get numeric thresholds from config
        def _get_threshold(idx: int, default: float = 1.0) -> float:
            try:
                return float(self._config.safety_thresholds[idx])
            except Exception:
                return default

        # Proximity
        prox = signals.get("proximity")
        if prox is not None:
            try:
                dist = float(prox)
                if dist < self._config.min_safety_margin:
                    thr = max(self._config.min_safety_margin, 1e-3)
                    sev = dist / thr
                    if sev < 0.5:
                        risk = "high"
                    elif sev < 1.0:
                        risk = "moderate"
                    else:
                        risk = "low"
                    self._hazards["proximity"] = {
                        "value": dist,
                        "threshold": self._config.min_safety_margin,
                        "severity": sev,
                        "risk_level": risk,
                        "message": f"Object within {dist:.2f} m (min {self._config.min_safety_margin} m)",
                    }
            except Exception as exc:
                self._logger.debug("Error processing proximity hazard: %s", exc)
        # Numeric hazards
        for idx, hname in enumerate(["high_voltage", "gas", "radiation"]):
            val = signals.get(hname)
            if val is None:
                continue
            try:
                level = float(val)
            except Exception:
                continue
            thr = _get_threshold(idx)
            if level > thr:
                denom = max(thr, 1e-3)
                sev = level / denom
                if sev >= 1.5:
                    risk = "high"
                elif sev >= 1.0:
                    risk = "moderate"
                else:
                    risk = "low"
                self._hazards[hname] = {
                    "value": level,
                    "threshold": thr,
                    "severity": sev,
                    "risk_level": risk,
                    "message": f"{hname.replace('_', ' ').title()} level {level:.2f} exceeds {thr:.2f} (severity {sev:.2f})",
                }
        # Dynamic obstacles
        for oname in ["train", "car", "human"]:
            val = signals.get(oname)
            if val is None:
                continue
            try:
                dist = float(val)
            except Exception:
                continue
            if dist < self._config.min_safety_margin:
                thr = max(self._config.min_safety_margin, 1e-3)
                sev = dist / thr
                if sev < 0.5:
                    risk = "high"
                elif sev < 1.0:
                    risk = "moderate"
                else:
                    risk = "low"
                self._hazards[oname] = {
                    "value": dist,
                    "threshold": self._config.min_safety_margin,
                    "severity": sev,
                    "risk_level": risk,
                    "message": f"{oname.title()} within {dist:.2f} m",
                }
        # Pedestrian
        if signals.get("pedestrian"):
            self._hazards["pedestrian"] = {
                "value": True,
                "threshold": True,
                "severity": 2.0,
                "risk_level": "high",
                "message": "Pedestrian detected in path",
            }
        # Faults
        faults = signals.get("faults")
        if faults:
            if isinstance(faults, str):
                flist = [faults]
            elif isinstance(faults, (list, tuple, set)):
                flist = [str(f) for f in faults]
            else:
                flist = []
            for fname in flist:
                self._hazards[f"fault:{fname}"] = {
                    "value": True,
                    "threshold": True,
                    "severity": 2.0,
                    "risk_level": "high",
                    "message": f"Fault detected: {fname}",
                }
        # Update exposure counts and annotate hazards
        current_keys = set(self._hazards.keys())
        for key in current_keys:
            self._exposures[key] = self._exposures.get(key, 0) + 1
            self._hazards[key]["exposure_cycles"] = self._exposures[key]
        # Remove stale exposures
        for key in list(self._exposures.keys()):
            if key not in current_keys:
                del self._exposures[key]
