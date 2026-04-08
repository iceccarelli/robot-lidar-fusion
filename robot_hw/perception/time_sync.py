"""Simple time synchronisation helper.

The ``TimeSync`` class matches LiDAR and camera frames based on their
timestamps.  In stage 1 we use a naive nearest‑neighbour approach:

1. Frames from each sensor are timestamped using the system clock
   (ideally synchronised across processes via NTP or PTP).
2. When a LiDAR frame arrives, the time sync helper searches a buffer
   of recent camera frames and selects the one whose timestamp is
   closest to the LiDAR timestamp, provided the difference is below
   ``max_offset`` seconds.
3. The matched pair is returned along with latency statistics.  If
   no camera frame falls within the allowed offset, ``None`` is
   returned for the camera frame.

Future stages should implement more robust synchronisation using
hardware triggers, PPS signals or PTP to provide sub‑millisecond
alignment.  This helper is deliberately simple to support the first
integration of LiDAR and camera into the control stack.
"""

from __future__ import annotations

import collections
from dataclasses import dataclass

from .sensor_frames import CameraFrame, LidarFrame


@dataclass
class SyncResult:
    """Result of matching a LiDAR frame to a camera frame.

    Attributes
    ----------
    lidar_frame : LidarFrame
        The LiDAR frame being synchronised.
    camera_frame : Optional[CameraFrame]
        The nearest camera frame, or ``None`` if no suitable match was
        found within the allowed time offset.
    offset : Optional[float]
        Absolute time difference between frames in seconds, or ``None``
        if no match was made.
    """

    lidar_frame: LidarFrame
    camera_frame: CameraFrame | None
    offset: float | None


class TimeSync:
    """Synchronise LiDAR and camera frames by timestamp.

    Parameters
    ----------
    max_offset : float
        Maximum allowed time difference between LiDAR and camera frames
        for them to be considered synchronised (seconds).  Frames with
        a larger offset are treated as unmatched.
    buffer_size : int
        Number of camera frames to keep in the buffer for matching.
    """

    def __init__(self, max_offset: float = 0.05, buffer_size: int = 10) -> None:
        self.max_offset = float(max_offset)
        self.buffer: collections.deque[CameraFrame] = collections.deque(maxlen=buffer_size)

    def add_camera_frame(self, frame: CameraFrame) -> None:
        """Append a camera frame to the buffer for future matching."""
        self.buffer.append(frame)

    def match(self, lidar_frame: LidarFrame) -> SyncResult:
        """Match a LiDAR frame to the closest camera frame.

        A linear search over buffered camera frames identifies the
        frame with the smallest absolute time difference.  If that
        difference exceeds ``max_offset``, the camera frame is
        considered absent.

        Parameters
        ----------
        lidar_frame : LidarFrame
            The LiDAR frame to synchronise.

        Returns
        -------
        SyncResult
            An object containing the LiDAR frame, the matched camera
            frame (if any) and the absolute time offset.
        """
        if not self.buffer:
            return SyncResult(lidar_frame=lidar_frame, camera_frame=None, offset=None)
        best: CameraFrame | None = None
        best_dt: float = float("inf")
        ts = lidar_frame.timestamp
        for cf in self.buffer:
            dt = abs(cf.timestamp - ts)
            if dt < best_dt:
                best = cf
                best_dt = dt
        if best is not None and best_dt <= self.max_offset:
            return SyncResult(lidar_frame=lidar_frame, camera_frame=best, offset=best_dt)
        return SyncResult(lidar_frame=lidar_frame, camera_frame=None, offset=None)
