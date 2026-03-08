"""Environment Adapter Module
============================

This module defines the :class:`EnvironmentAdapter`, which adapts
system parameters and module behaviour based on the selected
environment profile.  Different environments such as mines,
underwater sites, space habitats and forests impose distinct
constraints (power limits, thermal caps, available sensors) that must
be respected for safe and efficient operation.

The adapter reads the environment profile and related settings from
the configuration and applies them by adjusting power budgets,
thermal limits, locomotion modes and hazard thresholds.  It may also
enable or disable specific hardware devices based on the
``environment_hardware_ids`` list.

In stage A this class is a skeleton; future stages will implement
actual configuration and integration logic.
"""

from __future__ import annotations

from typing import Any

from robot_hw.robot_config import RobotConfig


class EnvironmentAdapter:
    """Adapt system parameters for the current environment.

    Parameters
    ----------
    config : RobotConfig
        System configuration including environment profile and
        environment‑specific budgets and thresholds.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        # Store the active configuration overrides
        self._overrides: dict[str, Any] = {}

    def configure(self) -> None:
        """Apply environment‑specific configuration overrides.

        This method should be called once during system startup or
        whenever the environment profile changes.  It updates
        internal settings such as power budgets and thermal limits
        based on the fields in :class:`RobotConfig`.  In this
        skeleton implementation no action is performed.
        """
        # Determine profile (case‑insensitive)
        profile = (self._config.environment_profile or "GENERAL").strip().upper()
        overrides: dict[str, Any] = {}
        # Apply power budget override: if provided for the environment use it,
        # otherwise fall back to the global power budget.  Note that the
        # configuration stores the budget in watts, whereas the battery
        # manager works in watt‑hours.  The adapter does not convert units;
        # it simply records the limit for the orchestrator to enforce.
        env_power = self._config.environment_power_budget_w
        global_power = self._config.power_budget_w
        if env_power and env_power > 0:
            overrides["power_budget_w"] = env_power
        else:
            overrides["power_budget_w"] = global_power
        # Apply thermal limit override: use environment‑specific thermal limit
        # if provided, otherwise use the global maximum temperature.
        env_thermal = self._config.environment_thermal_limit_c
        global_thermal = self._config.max_temperature_c
        if env_thermal and env_thermal > 0:
            overrides["thermal_limit_c"] = env_thermal
        else:
            overrides["thermal_limit_c"] = global_thermal
        # Determine locomotion mode based on environment profile.  Real
        # implementations would query hardware capabilities; here we map
        # profiles to generic modes.  The values are upper‑cased to
        # align with the locomotion controller's accepted modes.  Space
        # environments use reaction wheels for attitude control, but
        # locomotion in this context falls back to thrusters.
        locomotion_map = {
            "UNDERWATER": "THRUSTERS",
            "SPACE": "THRUSTERS",
            "MINING": "TRACKS",
            "FORESTRY": "LEGS",
            "GENERAL": "WHEELS",
        }
        overrides["locomotion_mode"] = locomotion_map.get(profile, "WHEELS")
        # Enabled hardware and sensors: propagate environment‑specific hardware
        overrides["enabled_hardware"] = tuple(self._config.environment_hardware_ids)
        # Hazard thresholds: preserve generic safety thresholds; these may be
        # interpreted differently depending on the environment.  The hazard
        # manager reads these directly from the configuration.
        overrides["safety_thresholds"] = tuple(self._config.safety_thresholds)

        # Configure optional sensor types and synthetic noise based on
        # environment profile.  Environments impose different sensing
        # requirements; for example, underwater vehicles rely on sonar and
        # pressure sensors, while mining robots need gas and radiation
        # detectors.  The predictive sensor processor uses these lists to
        # include additional raw sensor values in the fused state.  Noise
        # levels approximate sensor accuracy in each environment and are
        # expressed as the standard deviation of Gaussian noise applied
        # by the sensor processor.
        sensor_profiles = {
            "UNDERWATER": ("IMU", "SONAR", "PRESSURE"),
            "MINING": ("IMU", "GAS", "RADIATION", "LIDAR"),
            "SPACE": ("IMU", "RADIATION", "STAR_TRACKER", "GYRO"),
            "FORESTRY": ("IMU", "LIDAR", "THERMAL_IMAGER"),
            "GENERAL": (),
        }
        noise_profiles = {
            "UNDERWATER": 0.02,
            "MINING": 0.01,
            "SPACE": 0.005,
            "FORESTRY": 0.02,
            "GENERAL": 0.0,
        }
        overrides["environment_sensor_types"] = sensor_profiles.get(profile, ())
        overrides["sensor_noise_std"] = noise_profiles.get(profile, 0.0)
        # Store the computed overrides
        self._overrides = overrides

    def get_current_limits(self) -> dict[str, Any]:
        """Return the currently active environment limits.

        Returns
        -------
        dict[str, Any]
            A dictionary containing values such as power budget and
            thermal limit.  Returns an empty dictionary in this
            skeleton implementation.
        """
        # Return a shallow copy of the currently active overrides.  Call
        # :meth:`configure` before using this method to ensure the
        # overrides are up to date.
        return dict(self._overrides)
