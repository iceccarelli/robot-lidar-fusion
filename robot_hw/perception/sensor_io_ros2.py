"""ROS2 sensor ingestion for LiDAR and camera.

This module provides an asynchronous bridge between ROS2 topics and the
robot's internal perception pipeline.  It subscribes to the point
cloud topic published by an Ouster OS1 LiDAR (via the ``ouster_ros``
driver) and to the image and camera info topics published by a
camera driver (e.g. Intel RealSense ``realsense2_camera`` or a
generic UVC node).  Incoming messages are converted into internal
``LidarFrame`` and ``CameraFrame`` instances and stored in thread‑safe
attributes.  A background thread spins the ROS2 node to process
callbacks without blocking the main control loop.

Example usage::

    from robot_hw.perception.sensor_io_ros2 import Ros2SensorIO
    sensor_io = Ros2SensorIO(lidar_topic='/os_cloud_node/points',
                             camera_topic='/camera/color/image_raw',
                             camera_info_topic='/camera/color/camera_info')
    sensor_io.start()
    # In your control loop:
    lidar_frame = sensor_io.get_latest_lidar_frame()
    camera_frame = sensor_io.get_latest_camera_frame()
    # ... process frames ...
    sensor_io.stop()

Note that ROS2 must be installed and the appropriate sensor drivers
running.  The point cloud conversion uses ``sensor_msgs_py.point_cloud2``
from the ROS2 Python library; this package should be available in a
ROS2 environment.  If conversion fails, the raw message is stored in
the frame's metadata for downstream processing.
"""

from __future__ import annotations

import contextlib
import threading
import time
from typing import TYPE_CHECKING, Any

"""
Attempt to import ROS2 types.  We use a two‑stage import to satisfy static
type checkers (e.g. Pylance) while still working correctly when ROS2 is
missing at runtime.  When running under a ROS2 environment the imports
below will succeed; otherwise we fall back to placeholders.  During type
checking, the imports in the TYPE_CHECKING branch inform the checker of
the expected types.
"""

# During type checking we want to import the real modules to satisfy the
# type system.  At runtime we wrap the import in a try/except so that
# environments without ROS2 do not crash.
if TYPE_CHECKING:
    import numpy as np  # type: ignore
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2  # type: ignore
    from sensor_msgs_py import point_cloud2  # type: ignore

try:
    import numpy as np  # type: ignore
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2  # type: ignore
    from sensor_msgs_py import point_cloud2  # type: ignore
except Exception:
    # Define placeholders so that type checkers do not complain when
    # ROS2 is unavailable in this environment.  These imports will
    # raise at runtime when used if ROS2 is not installed.
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    PointCloud2 = object  # type: ignore
    Image = object  # type: ignore
    CameraInfo = object  # type: ignore
    point_cloud2 = None  # type: ignore
    np = None  # type: ignore

# Expose a flag indicating whether ROS2 imports succeeded.  Other modules
# can check this to determine if ROS2 is available on the system.
ROS2_AVAILABLE: bool = rclpy is not None

from .sensor_frames import CameraFrame, LidarFrame  # noqa: E402


class Ros2SensorIO:
    """Ingest LiDAR and camera data from ROS2 topics."""

    def __init__(self, *,
                 lidar_topic: str,
                 camera_topic: str,
                 camera_info_topic: str,
                 node_name: str = 'sensor_io',
                 qos_depth: int = 10) -> None:
        if rclpy is None:
            raise RuntimeError('ROS2 is not available; cannot initialise Ros2SensorIO')
        self._lidar_topic = lidar_topic
        self._camera_topic = camera_topic
        self._camera_info_topic = camera_info_topic
        self._qos_depth = int(qos_depth)
        self._node_name = node_name
        # Storage for latest frames with thread‑safety
        self._latest_lidar: LidarFrame | None = None
        self._latest_camera: CameraFrame | None = None
        self._camera_intrinsics: dict[str, Any] = {}
        # Thread and ROS2 node
        self._thread: threading.Thread | None = None
        self._node: Node | None = None
        self._stop_event = threading.Event()

    def start(self) -> None:
        """Start the ROS2 node and background spin thread."""
        if self._thread is not None:
            return
        # Initialise ROS2 and create node
        rclpy.init(args=None)
        self._node = _SensorIONode(parent=self,
                                   node_name=self._node_name,
                                   lidar_topic=self._lidar_topic,
                                   camera_topic=self._camera_topic,
                                   camera_info_topic=self._camera_info_topic,
                                   qos_depth=self._qos_depth)
        self._thread = threading.Thread(target=self._spin_thread, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the ROS2 spin thread to stop and shutdown."""
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        if rclpy.ok():
            rclpy.shutdown()
        self._thread = None
        self._node = None

    def get_latest_lidar_frame(self) -> LidarFrame | None:
        return self._latest_lidar

    def get_latest_camera_frame(self) -> CameraFrame | None:
        return self._latest_camera

    def _spin_thread(self) -> None:
        assert self._node is not None
        # Use a single‑threaded executor to spin the node without blocking
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self._node)
        while not self._stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)
        executor.shutdown()
        with contextlib.suppress(Exception):
            self._node.destroy_node()


class _SensorIONode(Node):
    """ROS2 node that receives LiDAR and camera messages and converts them."""

    def __init__(self, parent: Ros2SensorIO, node_name: str,
                 lidar_topic: str, camera_topic: str, camera_info_topic: str,
                 qos_depth: int) -> None:
        super().__init__(node_name)
        self._parent = parent
        # Subscriptions: use QoS profile optimised for sensor data
        from rclpy.qos import qos_profile_sensor_data
        qos = qos_profile_sensor_data
        qos.depth = qos_depth
        # Subscribe to LiDAR point clouds
        self._lidar_sub = self.create_subscription(PointCloud2,
                                                  lidar_topic,
                                                  self._lidar_callback,
                                                  qos)
        # Subscribe to camera images
        self._camera_sub = self.create_subscription(Image,
                                                   camera_topic,
                                                   self._camera_callback,
                                                   qos)
        # Subscribe to camera info for intrinsics
        self._cinfo_sub = self.create_subscription(CameraInfo,
                                                   camera_info_topic,
                                                   self._camera_info_callback,
                                                   qos)

    def _lidar_callback(self, msg: PointCloud2) -> None:
        # Extract timestamp; fallback to current time on failure
        try:
            ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9  # type: ignore
        except Exception:
            ts = time.time()
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            x, y, z, i = p
            points.append((float(x), float(y), float(z)))
            intensity_values.append(float(i))
        intensities = intensity_values if intensity_values else None
        # Convert point cloud to Cartesian coordinates
        if point_cloud2 is not None:
            try:
                for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True):
                    x, y, z, i = p
                    points.append((float(x), float(y), float(z)))
                    intensities.append(float(i))
            except Exception:
                try:
                    for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
                        x, y, z = p
                        points.append((float(x), float(y), float(z)))
                except Exception:
                    # If conversion fails, leave points empty and store raw message
                    points = []
                    intensities = None
        else:
            intensities = None
        frame_id = getattr(msg.header, 'frame_id', 'lidar') or 'lidar'
        meta: dict[str, Any] = {}
        if not points:
            meta['raw_msg'] = msg
        lf = LidarFrame(timestamp=ts,
                        frame_id=frame_id,
                        points_xyz=points,
                        intensities=intensities,
                        metadata=meta)
        self._parent._latest_lidar = lf

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        # Extract intrinsics from camera info
        try:
            fx = float(msg.k[0])
            fy = float(msg.k[4])
            cx = float(msg.k[2])
            cy = float(msg.k[5])
            d = [float(x) for x in msg.d]
            intrinsics = {
                'fx': fx,
                'fy': fy,
                'cx': cx,
                'cy': cy,
                'D': d,
                'distortion_model': msg.distortion_model
            }
            self._parent._camera_intrinsics = intrinsics
        except Exception:
            pass

    def _camera_callback(self, msg: Image) -> None:
        # Extract timestamp; fallback to current time
        try:
            ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9  # type: ignore
        except Exception:
            ts = time.time()
        # Convert raw bytes to numpy array if available
        try:
            if np is None:
                raise ImportError('NumPy unavailable')
            channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape((msg.height, msg.width, channels))
            frame: Any = arr
        except Exception:
            frame = bytes(msg.data)
        intrinsics = dict(self._parent._camera_intrinsics) if self._parent._camera_intrinsics else {}
        frame_id = getattr(msg.header, 'frame_id', 'camera') or 'camera'
        cf = CameraFrame(timestamp=ts,
                         frame_id=frame_id,
                         image=frame,
                         intrinsics=intrinsics,
                         metadata={})
        self._parent._latest_camera = cf
