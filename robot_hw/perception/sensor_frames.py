"""Sensor frame dataclasses.

This module defines data structures for LiDAR and camera frames used in
the robot control system.  They encapsulate timestamped sensor data and
associated metadata in a type‑safe manner.  These classes are plain
dataclasses and do not perform any processing; downstream modules are
responsible for interpreting the contained data.

The goal of introducing these dataclasses is to provide a clear API
between the sensor ingestion layer (e.g. ROS2 subscribers or direct
hardware SDKs) and the perception fusion layer.  Each frame carries a
``timestamp`` (in seconds, ideally from a common clock), a
``frame_id`` indicating the reference coordinate frame, raw sensor
measurements and optional metadata such as intrinsic calibration
parameters.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class LidarFrame:
    """A single LiDAR point cloud frame.

    Parameters
    ----------
    timestamp : float
        The time at which the frame was captured, expressed in seconds.
        It should originate from a monotonic clock or a synchronised
        source (e.g. PTP).  The ``TimeSync`` helper can normalise
        timestamps from different sensors.
    frame_id : str
        The coordinate frame in which the points are expressed (e.g.
        ``"os_sensor"`` for Ouster sensor frame).  Frame IDs should
        correspond to entries in the robot's TF tree.
    points_xyz : List[Tuple[float, float, float]]
        List of 3D points representing the LiDAR returns.  Units are
        metres.  For performance, downstream code may convert this to
        a NumPy array, but a plain Python list is sufficient for basic
        processing.
    intensities : Optional[List[float]]
        Optional list of per‑point intensity values.  May be ``None``
        if the driver does not provide intensity information.
    metadata : Dict[str, Any]
        Additional metadata associated with the frame (e.g. sensor
        configuration, scan ID).  This field is left opaque for future
        extensions.
    """

    timestamp: float
    frame_id: str
    points_xyz: list[tuple[float, float, float]]
    intensities: list[float] | None
    metadata: dict[str, Any]


@dataclass
class CameraFrame:
    """A single camera image frame with calibration information.

    Parameters
    ----------
    timestamp : float
        The capture time of the image in seconds.  Must originate
        from the same clock domain as LiDAR frames for synchronisation.
    frame_id : str
        The coordinate frame of the camera optical centre (e.g.
        ``"camera_link"``).  Should match the TF tree definitions.
    image : Any
        Raw image data.  In a live system this will typically be a
        NumPy array with shape `(height, width, channels)` and dtype
        `uint8` for RGB images.  In this codebase, we leave it typed
        as ``Any`` to avoid a mandatory dependency on NumPy.  Storing
        images as bytes is also acceptable for external modules to
        decode.
    intrinsics : Dict[str, Any]
        Dictionary describing the intrinsic calibration of the camera.
        Keys include ``fx``, ``fy`` (focal lengths), ``cx``, ``cy``
        (principal point) and distortion coefficients ``D``.  This
        structure mirrors the ROS ``sensor_msgs/CameraInfo`` message.
    metadata : Dict[str, Any]
        Arbitrary metadata about the capture (e.g. exposure time,
        gain, sensor temperature).  Can be empty.
    """

    timestamp: float
    frame_id: str
    image: Any
    intrinsics: dict[str, Any]
    metadata: dict[str, Any]
