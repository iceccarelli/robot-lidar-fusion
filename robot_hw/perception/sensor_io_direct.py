"""Direct Python SDK ingestion for LiDAR and camera.

This module provides an alternative to ROS2 for ingesting LiDAR and
camera data directly using vendor SDKs.  It is provided primarily for
testing and environments where ROS2 is unavailable.  However, the
default and recommended path for production is to use ROS2 because it
offers robust driver implementations, time synchronisation and
cross‑platform support.

Two concrete ingestion classes are defined:

* :class:`OusterSDKSensorIO`: Uses the ``ouster-sdk`` to ingest UDP
  packets from an Ouster OS1 LiDAR.  It runs a background thread to
  poll packets and assemble point clouds at the configured frame
  rate.
* :class:`UvcCameraSensorIO`: Uses OpenCV (cv2.VideoCapture) or
  pyrealsense2 (if available) to read frames from a USB camera or
  Intel RealSense device.  Frames are timestamped with the local
  system clock.

These classes implement the same minimal interface as
``Ros2SensorIO``, exposing ``start()``, ``stop()``,
``get_latest_lidar_frame()`` and ``get_latest_camera_frame()``
methods.  Internally they store the latest frame in a thread‑safe
attribute updated asynchronously.

This direct ingestion is less desirable than ROS2 because it
duplicates driver functionality, lacks automatic time synchronisation
and increases implementation complexity.  Use it only when necessary.
"""

from __future__ import annotations

import threading
import time
from typing import Any

try:
    import cv2  # type: ignore
except Exception:
    cv2 = None  # type: ignore
try:
    import pyrealsense2 as rs  # type: ignore
except Exception:
    rs = None  # type: ignore
try:
    from ouster import client as ouster_client  # type: ignore
except Exception:
    ouster_client = None  # type: ignore

import contextlib

import numpy as np  # type: ignore

from .sensor_frames import CameraFrame, LidarFrame


class OusterSDKSensorIO:
    """Ingest LiDAR frames directly from an Ouster sensor using the SDK."""

    def __init__(self, *, host_ip: str, lidar_ip: str,
                 lidar_port: int = 7502, imu_port: int = 7503) -> None:
        if ouster_client is None:
            raise RuntimeError('ouster-sdk is not installed; cannot initialise OusterSDKSensorIO')
        self.host_ip = host_ip
        self.lidar_ip = lidar_ip
        self.lidar_port = int(lidar_port)
        self.imu_port = int(imu_port)
        self._latest_lidar: LidarFrame | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        # Attempt to retrieve metadata from sensor; may fail if sensor
        # unreachable.  Metadata is stored for potential future use.
        self._metadata: Any | None = None
        try:
            self._metadata = ouster_client.get_sensor_info((self.lidar_ip, 7501))
        except Exception:
            self._metadata = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self._thread = None

    def get_latest_lidar_frame(self) -> LidarFrame | None:
        return self._latest_lidar

    def get_latest_camera_frame(self) -> CameraFrame | None:
        return None

    def _run(self) -> None:
        if ouster_client is None:
            return
        try:
            with ouster_client.Sensor(lidar_host=self.lidar_ip,
                                      lidar_port=self.lidar_port,
                                      imu_port=self.imu_port) as sensor:
                scans = ouster_client.Scans(sensor)
                xyzlut = None
                if self._metadata is not None:
                    try:
                        xyzlut = ouster_client.XYZLut(self._metadata)
                    except Exception:
                        xyzlut = None
                for scan in scans:
                    if self._stop_event.is_set():
                        break
                    ts = time.time()
                    points: list[tuple[float, float, float]] = []
                    if xyzlut is not None:
                        try:
                            xyz = xyzlut(scan)
                            for row in xyz:
                                for p in row:
                                    points.append((float(p[0]), float(p[1]), float(p[2])))
                        except Exception:
                            points = []
                    lf = LidarFrame(timestamp=ts,
                                    frame_id='os_sensor',
                                    points_xyz=points,
                                    intensities=None,
                                    metadata={})
                    self._latest_lidar = lf
                    if self._stop_event.is_set():
                        break
        except Exception:
            return


class UvcCameraSensorIO:
    """Ingest frames from a USB or RealSense camera using OpenCV/Pyrealsense2."""

    def __init__(self, *, device: int | None = 0, width: int = 640, height: int = 480) -> None:
        if device is not None and cv2 is None:
            raise RuntimeError('OpenCV (cv2) is not installed; cannot capture from UVC camera')
        if device is None and rs is None:
            raise RuntimeError('pyrealsense2 is not installed; cannot use RealSense camera')
        self.device = device
        self.width = width
        self.height = height
        self._latest_camera: CameraFrame | None = None
        self._cap: Any | None = None
        self._pipeline: Any | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        if self.device is not None:
            self._cap = cv2.VideoCapture(self.device) if cv2 is not None else None
            if self._cap is not None and cv2 is not None:
                try:
                    self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                except Exception:
                    pass
        else:
            if rs is not None:
                self._pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
                if self._pipeline is None:
                    raise RuntimeError('Failed to initialise RealSense pipeline')
                self._pipeline.start(config)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
        if self._pipeline is not None:
            with contextlib.suppress(Exception):
                self._pipeline.stop()
        self._thread = None

    def get_latest_lidar_frame(self) -> LidarFrame | None:
        return None

    def get_latest_camera_frame(self) -> CameraFrame | None:
        return self._latest_camera

    def _run(self) -> None:
        while not self._stop_event.is_set():
            if self.device is not None and self._cap is not None:
                ret, frame = self._cap.read()
                if not ret:
                    time.sleep(0.05)
                    continue
                ts = time.time()
                try:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if cv2 is not None else frame
                    img_data = frame_rgb
                except Exception:
                    img_data = frame
                cf = CameraFrame(timestamp=ts,
                                 frame_id='camera',
                                 image=img_data,
                                 intrinsics={},
                                 metadata={})
                self._latest_camera = cf
            elif self.device is None and self._pipeline is not None:
                frames = None
                try:
                    frames = self._pipeline.wait_for_frames(timeout_ms=100) if rs is not None else None
                except Exception:
                    frames = None
                if frames is None:
                    continue
                color_frame = frames.get_color_frame() if rs is not None else None
                if not color_frame:
                    continue
                ts = time.time()
                try:
                    img = np.asanyarray(color_frame.get_data())
                    cf = CameraFrame(timestamp=ts,
                                     frame_id='camera',
                                     image=img,
                                     intrinsics={},
                                     metadata={})
                    self._latest_camera = cf
                except Exception:
                    continue
            else:
                time.sleep(0.05)
