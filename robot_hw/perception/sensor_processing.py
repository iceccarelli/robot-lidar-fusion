"""Sensor Processing Module
===========================

This module defines the :class:`SensorProcessor`, which is responsible
for interfacing with various sensors (IMUs, encoders, cameras, etc.),
applying calibration factors and fusing the data into a coherent
state representation.  The resulting state is consumed by higher
levels of the control stack, including navigation, locomotion and
hazard management.

The design emphasises:

* **Extensibility:** Support for multiple sensor types and fusion
  algorithms.  Implementations may override methods to provide
  environment‑specific processing (e.g. underwater pressure sensors).
* **Determinism:** A consistent update interface that integrates into
  the 100 Hz control loop.
* **Safety:** Proper handling of missing or invalid sensor readings.

In stage A this class is a skeleton; future stages will implement
actual sensor reading and fusion logic.
"""

from __future__ import annotations

import logging
import random
from typing import Any, cast

from robot_hw.robot_config import RobotConfig

from .lidar_utils import compute_proximity
from .sensor_frames import CameraFrame, LidarFrame  # type: ignore

try:
    # Import predictive controller from the AI package if available.  This
    # optional dependency allows the sensor processor to refine state
    # estimates using machine learning models (e.g. Kalman filters,
    # recurrent networks).  If the import fails, a no‑op fallback is
    # used.
    from ai.predictive_controller import PredictiveController  # type: ignore
except Exception:
    PredictiveController = None  # type: ignore


class SensorProcessor:
    """Fuse raw sensor inputs into a unified state.

    Instances of this class are responsible for reading raw sensor
    measurements, applying calibration factors and producing a
    dictionary representing the robot's current state (pose, velocity,
    acceleration, etc.).  The exact keys and semantics of the state
    dictionary should be documented alongside the modules that
    consume it.

    Parameters
    ----------
    config : RobotConfig
        The system configuration, including calibration factors and
        enabled sensors.  See :mod:`robot_config` for details.
    """

    def __init__(self, config: RobotConfig) -> None:
        self._config = config
        # Internal storage for the latest fused state
        self._state: dict[str, Any] = {}
        # Store previous linear velocity and timestamp to compute acceleration
        self._prev_lin_vel: tuple[float, float, float] | None = None
        self._prev_timestamp: float | None = None
        # Instantiate a predictive controller if available
        self._predictor = PredictiveController(config) if PredictiveController is not None else None
        # Configure a module‑level logger
        self._logger = logging.getLogger(self.__class__.__name__)
        # Standard deviation of Gaussian noise applied to orientation and velocity
        try:
            self._noise_std = float(getattr(config, 'sensor_noise_std', 0.0))
        except Exception:
            self._noise_std = 0.0
        # Additional sensor types enabled for this environment
        self._sensor_types = tuple(getattr(config, 'environment_sensor_types', ()))

    def read_sensors(self) -> dict[str, Any]:
        """Read raw sensor data.

        In a production system this method would interface with
        hardware drivers or simulation hooks to retrieve IMU, encoder
        and other sensor data.  The default implementation simply
        raises ``NotImplementedError``.  The orchestrator is expected
        to provide raw state via :meth:`fuse_sensors` directly.

        Returns
        -------
        dict[str, Any]
            Raw sensor readings keyed by sensor identifier.
        """
        raise NotImplementedError("Sensor reading is provided externally in this implementation")

    def fuse_sensors(self, raw_data: dict[str, Any]) -> dict[str, Any]:
        """Fuse raw sensor data into a unified state.

        Applies calibration factors and combines measurements from
        different modalities (e.g. IMU + encoders) to estimate the
        robot's pose, linear velocity and acceleration.  This
        implementation provides a simple reference fusion:

        * **Orientation:** A placeholder tuple ``(0.0, 0.0, 0.0)`` for
          roll, pitch and yaw.  Future implementations should derive
          orientation from IMU data.
        * **Linear velocity:** The average of joint velocities in the
          x‑direction.  If joint velocities are unavailable, zero
          velocity is assumed.
        * **Acceleration:** The derivative of linear velocity across
          control cycles.  If insufficient history exists, zero
          acceleration is returned.

        Calibration factors (``sensor_calibration_factors``) can be
        used to scale raw measurements.  They are applied element‑wise
        to the orientation and velocity vectors if provided.

        Parameters
        ----------
        raw_data : dict[str, Any]
            Raw state dictionary containing at least ``positions`` and
            optionally ``velocities`` and ``timestamp``.

        Returns
        -------
        dict[str, Any]
            A fused state dictionary containing the original fields
            along with ``orientation``, ``linear_velocity`` and
            ``acceleration`` entries.
        """
        fused = dict(raw_data)
        # Extract timestamp for acceleration calculation
        timestamp = None
        try:
            ts_val = raw_data.get("timestamp")
            timestamp = float(ts_val) if ts_val is not None else None
        except Exception:
            timestamp = None
        # Orientation: use raw orientation if provided, otherwise placeholder zeros
        orientation: tuple[float, float, float] = (0.0, 0.0, 0.0)
        try:
            raw_orientation = raw_data.get("orientation")
            if isinstance(raw_orientation, (list, tuple)) and len(raw_orientation) >= 3:
                orientation = cast(tuple[float, float, float], (float(raw_orientation[0]), float(raw_orientation[1]), float(raw_orientation[2])))
        except Exception:
            orientation = (0.0, 0.0, 0.0)
        # Apply calibration factors if present (first three entries)
        calib = self._config.sensor_calibration_factors
        if calib and len(calib) >= 3:
            orientation = (
                float(calib[0]) * orientation[0],
                float(calib[1]) * orientation[1],
                float(calib[2]) * orientation[2],
            )  # still zeros
        # Use predictive controller to refine orientation if available
        if self._predictor is not None:
            try:
                orientation = self._predictor.predict_orientation(orientation, raw_data)
            except Exception as e:
                # Log but proceed; predictive failure should not disrupt fusion
                self._logger.debug("Predictive controller error in orientation: %s", e)
        # Apply Gaussian noise to orientation
        if self._noise_std > 0.0:
            orientation = (
                orientation[0] + random.gauss(0.0, self._noise_std),
                orientation[1] + random.gauss(0.0, self._noise_std),
                orientation[2] + random.gauss(0.0, self._noise_std),
            )
        fused["orientation"] = orientation
        # Linear velocity: average of joint velocities if available
        lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        velocities = raw_data.get("velocities")
        if isinstance(velocities, dict) and velocities:
            # Compute the mean of absolute joint velocities as a scalar
            try:
                vals = [abs(float(v)) for v in velocities.values() if v is not None]
            except Exception:
                vals = []
            if vals:
                mean_vel = sum(vals) / len(vals)
                lin_vel = (mean_vel, 0.0, 0.0)
        # Apply calibration factors for velocity if provided (next three entries)
        if calib and len(calib) >= 6:
            lin_vel = (
                float(calib[3]) * lin_vel[0],
                float(calib[4]) * lin_vel[1],
                float(calib[5]) * lin_vel[2],
            )  # apply scale
        # Apply Gaussian noise to velocity
        if self._noise_std > 0.0:
            lin_vel = (
                lin_vel[0] + random.gauss(0.0, self._noise_std),
                lin_vel[1] + random.gauss(0.0, self._noise_std),
                lin_vel[2] + random.gauss(0.0, self._noise_std),
            )
        fused["linear_velocity"] = lin_vel
        # Acceleration: derivative of linear velocity
        acc = (0.0, 0.0, 0.0)
        if self._prev_lin_vel is not None and self._prev_timestamp is not None and timestamp is not None:
            dt = timestamp - self._prev_timestamp
            if dt > 0.0:
                try:
                    acc = tuple((lv - pv) / dt for lv, pv in zip(lin_vel, self._prev_lin_vel, strict=False))
                except Exception:
                    acc = (0.0, 0.0, 0.0)
        # Apply calibration factors for acceleration if provided (next three entries)
        if calib and len(calib) >= 9:
            acc = tuple(float(c) * a for c, a in zip(calib[6:9], acc, strict=False))
        # Apply Gaussian noise to acceleration
        if self._noise_std > 0.0:
            acc = tuple(a + random.gauss(0.0, self._noise_std) for a in acc)
        fused["acceleration"] = acc
        # Update internal history
        self._prev_lin_vel = lin_vel
        self._prev_timestamp = timestamp
        # Include additional sensor readings if configured
        for sname in self._sensor_types:
            # Normalise key
            key = str(sname).lower()
            if key not in fused and key in raw_data:
                fused[key] = raw_data.get(key)
        # ------------------------------------------------------------------
        # Stage 1 integration: copy LiDAR and camera frames into fused state.
        # The orchestrator can insert entries 'lidar_frame' and
        # 'camera_frame' into raw_data.  When present, we expose a
        # simplified representation and derive a proximity estimate from
        # the LiDAR point cloud.  Proximity values from the LiDAR
        # override any previously set proximity (e.g. synthetic or
        # environment‑injected values), ensuring consistent semantics.
        try:
            lf = raw_data.get("lidar_frame")
            if isinstance(lf, LidarFrame):
                fused["lidar"] = {
                    "timestamp": lf.timestamp,
                    "frame_id": lf.frame_id,
                    "num_points": len(lf.points_xyz),
                    "metadata": lf.metadata,
                }
                prox = compute_proximity(lf.points_xyz, fov_degrees=60.0)
                if prox is not None:
                    try:
                        current_prox = fused.get("proximity")
                        if current_prox is None:
                            fused["proximity"] = prox
                        else:
                            fused["proximity"] = min(float(current_prox), prox)
                    except Exception:
                        fused["proximity"] = prox
        except Exception:
            pass
        try:
            cf = raw_data.get("camera_frame")
            if isinstance(cf, CameraFrame):
                fused["camera"] = {
                    "timestamp": cf.timestamp,
                    "frame_id": cf.frame_id,
                    "image": cf.image,
                    "intrinsics": cf.intrinsics,
                    "metadata": cf.metadata,
                }
        except Exception:
            pass
        return fused

    def update(self) -> dict[str, Any]:
        """Read sensors and update the fused state.

        This convenience method reads raw sensor data, fuses it and
        stores the result internally.  The fused state is also
        returned.

        Returns
        -------
        dict[str, Any]
            The latest fused state.
        """
        raw = self.read_sensors()
        fused = self.fuse_sensors(raw)
        self._state = fused
        return fused

    def get_state(self) -> dict[str, Any]:
        """Return the most recently fused state.

        Returns
        -------
        dict[str, Any]
            The state dictionary produced by the most recent call to
            :meth:`update`.  Returns an empty dictionary if no
            update has been performed yet.
        """
        return dict(self._state)
