#!/usr/bin/env python3
"""Demonstration script for live LiDAR and camera ingestion.

This script shows how to initialise the sensor I/O layer introduced
in stage 1 and print statistics about incoming LiDAR and camera
frames.  It supports both ROS2 topic ingestion and direct SDK
ingestion based on the values configured in the environment (see
``robot.env.example``).  When run, the script will:

* Start the appropriate sensor I/O object (ROS2 or direct).
* Spawn a simple main loop that polls the latest LiDAR frame,
  computes the minimum forward obstacle distance and measures
  frame rates.
* Perform a naive time synchronisation between LiDAR and camera
  frames and report the absolute time offset.
* Optionally display the camera image using OpenCV if available.

To exit the loop press Ctrl‑C.  This script is intended for manual
testing and diagnostics during development; it does not control the
robot or send any actuator commands.
"""

from __future__ import annotations

import logging
import sys
import time
from typing import Any

from robot_hw.perception.lidar_utils import compute_proximity
from robot_hw.perception.sensor_frames import CameraFrame, LidarFrame
from robot_hw.perception.time_sync import TimeSync
from robot_hw.robot_config import load as load_config


def main() -> None:
    config = load_config()
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("demo")
    use_ros = getattr(config, "use_ros2", True)
    enable_lidar = getattr(config, "enable_lidar", False)
    enable_camera = getattr(config, "enable_camera", False)
    if not (enable_lidar or enable_camera):
        logger.warning("LiDAR and camera ingestion are both disabled in the configuration.")
    time_sync = TimeSync()
    sensor_io = None
    try:
        # Decide whether to use ROS2 or direct ingestion based on configuration
        # and availability.  We only attempt to initialise ROS2 if it is
        # requested (use_ros) and at least one sensor is enabled.  We also
        # check whether ROS2 is available in this environment.  If ROS2 is
        # unavailable or initialisation fails, we fall back to direct
        # ingestion using the SDK/USB camera.
        use_direct = False
        if use_ros and (enable_lidar or enable_camera):
            try:
                from robot_hw.perception.sensor_io_ros2 import ROS2_AVAILABLE, Ros2SensorIO  # type: ignore
            except Exception:
                ROS2_AVAILABLE = False  # type: ignore
            if not ROS2_AVAILABLE:
                logger.warning("ROS2 is not available; falling back to direct sensor ingestion")
                use_direct = True
            else:
                try:
                    sensor_io = Ros2SensorIO(
                        lidar_topic=config.lidar_topic,
                        camera_topic=config.camera_topic,
                        camera_info_topic=config.camera_info_topic,
                    )
                    logger.info(
                        "Starting ROS2 sensor I/O with topics %s, %s",
                        config.lidar_topic,
                        config.camera_topic,
                    )
                except Exception as e:
                    logger.error(
                        "Failed to initialise ROS2 sensor I/O: %s; falling back to direct ingestion",
                        e,
                    )
                    use_direct = True
        else:
            use_direct = True

        if use_direct:
            from robot_hw.perception.sensor_io_direct import OusterSDKSensorIO, UvcCameraSensorIO  # type: ignore

            class _DirectProxy:
                """Simple wrapper to unify LiDAR and camera ingestion classes."""

                def __init__(self, lidar_io: Any | None, camera_io: Any | None) -> None:
                    self._lidar_io = lidar_io
                    self._camera_io = camera_io

                def start(self) -> None:
                    if self._lidar_io is not None:
                        self._lidar_io.start()
                    if self._camera_io is not None:
                        self._camera_io.start()

                def stop(self) -> None:
                    if self._lidar_io is not None:
                        self._lidar_io.stop()
                    if self._camera_io is not None:
                        self._camera_io.stop()

                def get_latest_lidar_frame(self) -> LidarFrame | None:
                    return (
                        self._lidar_io.get_latest_lidar_frame()
                        if self._lidar_io is not None
                        else None
                    )

                def get_latest_camera_frame(self) -> CameraFrame | None:
                    return (
                        self._camera_io.get_latest_camera_frame()
                        if self._camera_io is not None
                        else None
                    )

            lidar_io = None
            camera_io = None
            if enable_lidar:
                lidar_io = OusterSDKSensorIO(
                    host_ip=config.host_ip,
                    lidar_ip=config.lidar_ip,
                    lidar_port=config.lidar_port,
                    imu_port=config.imu_port,
                )
                logger.info("Initialised direct LiDAR ingestion from %s", config.lidar_ip)
            if enable_camera:
                camera_io = UvcCameraSensorIO(device=config.camera_device)
                logger.info(
                    "Initialised direct camera ingestion on device %s", config.camera_device
                )
            sensor_io = _DirectProxy(lidar_io, camera_io)
        if sensor_io is None:
            logger.error("Failed to initialise sensor ingestion; exiting.")
            sys.exit(1)
        sensor_io.start()
        logger.info("Sensor ingestion started; collecting frames...")
        last_lidar_ts: float | None = None
        last_cam_ts: float | None = None
        # Attempt to import OpenCV for optional preview
        try:
            import cv2  # type: ignore

            show_preview = True
        except Exception:
            cv2 = None  # type: ignore
            show_preview = False
        while True:
            lidar_frame = sensor_io.get_latest_lidar_frame() if enable_lidar else None
            camera_frame = sensor_io.get_latest_camera_frame() if enable_camera else None
            if camera_frame is not None:
                time_sync.add_camera_frame(camera_frame)
                if last_cam_ts is not None:
                    cam_dt = camera_frame.timestamp - last_cam_ts
                    logger.debug("Camera frame period: %.3f s", cam_dt)
                last_cam_ts = camera_frame.timestamp
                if show_preview and cv2 is not None and camera_frame.image is not None:
                    try:
                        cv2.imshow("Camera Preview", camera_frame.image)
                        cv2.waitKey(1)
                    except Exception:
                        pass
            if lidar_frame is not None:
                if last_lidar_ts is not None:
                    lidar_dt = lidar_frame.timestamp - last_lidar_ts
                    logger.info("LiDAR frame period: %.3f s", lidar_dt)
                last_lidar_ts = lidar_frame.timestamp
                # Compute proximity
                prox = compute_proximity(lidar_frame.points_xyz)
                msg = (
                    f"LiDAR points: {len(lidar_frame.points_xyz)}, Proximity: {prox:.2f} m"
                    if prox is not None
                    else f"LiDAR points: {len(lidar_frame.points_xyz)}"
                )
                # Match with camera
                sync_res = time_sync.match(lidar_frame)
                if sync_res.camera_frame is not None and sync_res.offset is not None:
                    msg += f", Camera offset: {sync_res.offset:.3f} s"
                logger.info(msg)
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Interrupted by user; shutting down.")
    finally:
        if sensor_io is not None:
            sensor_io.stop()
        try:
            import cv2  # type: ignore

            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
