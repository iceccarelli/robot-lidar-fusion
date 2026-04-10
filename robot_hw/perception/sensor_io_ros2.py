"""ROS 2 sensor ingestion for LiDAR and camera topics.

This module provides an asynchronous bridge between ROS 2 sensor topics and
this repository's internal perception frame model. It subscribes to a LiDAR
point-cloud topic and to camera image and calibration topics, converts incoming
messages into :class:`LidarFrame` and :class:`CameraFrame` objects, and exposes
thread-safe accessors used by the orchestrator.

The implementation is intentionally conservative about dependencies and runtime
behaviour:

* Importing the module must remain safe even when ROS 2 is not installed.
* Starting and stopping the ingestion thread must be idempotent.
* The class must preserve the minimal interface shared with the direct sensor
  I/O implementations used elsewhere in the repository.
* Message decoding failures must degrade gracefully without destabilising the
  control loop.
"""

from __future__ import annotations

import contextlib
import threading
import time
from typing import TYPE_CHECKING, Any, Final

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
    rclpy = None  # type: ignore
    Node = object  # type: ignore[misc,assignment]
    PointCloud2 = object  # type: ignore[misc,assignment]
    Image = object  # type: ignore[misc,assignment]
    CameraInfo = object  # type: ignore[misc,assignment]
    point_cloud2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]

from .sensor_frames import CameraFrame, LidarFrame

ROS2_AVAILABLE: Final[bool] = rclpy is not None
_DEFAULT_FRAME_ID_LIDAR: Final[str] = "lidar"
_DEFAULT_FRAME_ID_CAMERA: Final[str] = "camera"
_DEFAULT_NODE_NAME: Final[str] = "sensor_io"
_DEFAULT_QOS_DEPTH: Final[int] = 10
_SPIN_TIMEOUT_SEC: Final[float] = 0.1
_JOIN_TIMEOUT_SEC: Final[float] = 2.0


class Ros2SensorIO:
    """Threaded ROS 2 sensor ingestion bridge.

    The public interface intentionally mirrors the direct sensor I/O classes in
    :mod:`robot_hw.perception.sensor_io_direct` so the orchestrator can switch
    between implementations without special handling.
    """

    def __init__(
        self,
        *,
        lidar_topic: str,
        camera_topic: str,
        camera_info_topic: str,
        node_name: str = _DEFAULT_NODE_NAME,
        qos_depth: int = _DEFAULT_QOS_DEPTH,
    ) -> None:
        if rclpy is None:
            raise RuntimeError("ROS2 is not available; cannot initialise Ros2SensorIO")

        self._lidar_topic = str(lidar_topic)
        self._camera_topic = str(camera_topic)
        self._camera_info_topic = str(camera_info_topic)
        self._node_name = str(node_name)
        self._qos_depth = max(1, int(qos_depth))

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._node: Node | None = None
        self._owns_rclpy_context = False

        self._latest_lidar: LidarFrame | None = None
        self._latest_camera: CameraFrame | None = None
        self._camera_intrinsics: dict[str, Any] = {}

    def start(self) -> None:
        """Start the ROS 2 subscriptions and background spin thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        if rclpy is None:
            raise RuntimeError("ROS2 is not available; cannot start Ros2SensorIO")

        self._stop_event.clear()
        self._ensure_rclpy_context()
        self._node = _SensorIONode(
            parent=self,
            node_name=self._node_name,
            lidar_topic=self._lidar_topic,
            camera_topic=self._camera_topic,
            camera_info_topic=self._camera_info_topic,
            qos_depth=self._qos_depth,
        )
        self._thread = threading.Thread(
            target=self._spin_thread,
            name=f"{self._node_name}_ros2_spin",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the spin thread and release ROS 2 resources safely."""
        thread = self._thread
        if thread is None:
            return

        self._stop_event.set()
        thread.join(timeout=_JOIN_TIMEOUT_SEC)
        self._thread = None

        node = self._node
        self._node = None
        if node is not None:
            with contextlib.suppress(Exception):
                node.destroy_node()

        if rclpy is not None and self._owns_rclpy_context:
            with contextlib.suppress(Exception):
                if rclpy.ok():
                    rclpy.shutdown()
        self._owns_rclpy_context = False

    def get_latest_lidar_frame(self) -> LidarFrame | None:
        """Return the most recent LiDAR frame, if one has been received."""
        with self._lock:
            return self._latest_lidar

    def get_latest_camera_frame(self) -> CameraFrame | None:
        """Return the most recent camera frame, if one has been received."""
        with self._lock:
            return self._latest_camera

    def _ensure_rclpy_context(self) -> None:
        if rclpy is None:
            raise RuntimeError("ROS2 is not available; cannot initialise rclpy")
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True
        else:
            self._owns_rclpy_context = False

    def _set_latest_lidar_frame(self, frame: LidarFrame) -> None:
        with self._lock:
            self._latest_lidar = frame

    def _set_latest_camera_frame(self, frame: CameraFrame) -> None:
        with self._lock:
            self._latest_camera = frame

    def _set_camera_intrinsics(self, intrinsics: dict[str, Any]) -> None:
        with self._lock:
            self._camera_intrinsics = dict(intrinsics)

    def _get_camera_intrinsics_copy(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._camera_intrinsics)

    def _spin_thread(self) -> None:
        node = self._node
        if node is None or rclpy is None:
            return

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while not self._stop_event.is_set():
                executor.spin_once(timeout_sec=_SPIN_TIMEOUT_SEC)
        finally:
            with contextlib.suppress(Exception):
                executor.shutdown()
            with contextlib.suppress(Exception):
                executor.remove_node(node)


class _SensorIONode(Node):
    """ROS 2 node that converts sensor messages into repository frame types."""

    def __init__(
        self,
        *,
        parent: Ros2SensorIO,
        node_name: str,
        lidar_topic: str,
        camera_topic: str,
        camera_info_topic: str,
        qos_depth: int,
    ) -> None:
        super().__init__(node_name)
        self._parent = parent
        qos_profile = self._build_sensor_qos(qos_depth)
        self._lidar_sub = self.create_subscription(
            PointCloud2,
            lidar_topic,
            self._lidar_callback,
            qos_profile,
        )
        self._camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            qos_profile,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile,
        )

    @staticmethod
    def _build_sensor_qos(qos_depth: int) -> Any:
        try:
            from rclpy.qos import (  # type: ignore[import-not-found]
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )

            return QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=int(qos_depth),
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
        except Exception:
            return int(qos_depth)

    @staticmethod
    def _extract_timestamp(message: Any) -> float:
        try:
            return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        except Exception:
            return time.time()

    @staticmethod
    def _extract_frame_id(message: Any, default: str) -> str:
        try:
            frame_id = str(message.header.frame_id)
        except Exception:
            frame_id = default
        return frame_id or default

    @staticmethod
    def _parse_intrinsics(msg: CameraInfo) -> dict[str, Any] | None:
        try:
            return {
                "fx": float(msg.k[0]),
                "fy": float(msg.k[4]),
                "cx": float(msg.k[2]),
                "cy": float(msg.k[5]),
                "D": [float(value) for value in msg.d],
                "distortion_model": str(msg.distortion_model),
                "width": int(msg.width),
                "height": int(msg.height),
            }
        except (AttributeError, IndexError, TypeError, ValueError):
            return None

    @staticmethod
    def _decode_image(msg: Image) -> Any:
        raw_data = bytes(getattr(msg, "data", b""))
        if np is None:
            return raw_data

        try:
            height = int(getattr(msg, "height", 0))
            width = int(getattr(msg, "width", 0))
            if height <= 0 or width <= 0:
                return raw_data

            encoding = str(getattr(msg, "encoding", "")).lower()
            channel_map = {
                "mono8": 1,
                "8uc1": 1,
                "rgb8": 3,
                "bgr8": 3,
                "8uc3": 3,
                "rgba8": 4,
                "bgra8": 4,
                "8uc4": 4,
            }
            channels = channel_map.get(encoding)
            if channels is None:
                return raw_data

            array = np.frombuffer(raw_data, dtype=np.uint8)
            expected_size = height * width * channels
            if array.size != expected_size:
                return raw_data
            if channels == 1:
                return array.reshape((height, width))
            return array.reshape((height, width, channels))
        except Exception:
            return raw_data

    @staticmethod
    def _iter_points(
        msg: PointCloud2,
        *,
        include_intensity: bool,
    ) -> tuple[list[tuple[float, float, float]], list[float] | None]:
        if point_cloud2 is None:
            return [], None

        points: list[tuple[float, float, float]] = []
        intensities: list[float] = []
        field_names: tuple[str, ...]
        if include_intensity:
            field_names = ("x", "y", "z", "intensity")
        else:
            field_names = ("x", "y", "z")

        for point in point_cloud2.read_points(  # type: ignore[union-attr]
            msg,
            field_names=field_names,
            skip_nans=True,
        ):
            if include_intensity:
                x, y, z, intensity = point
                intensities.append(float(intensity))
            else:
                x, y, z = point
            points.append((float(x), float(y), float(z)))

        return points, (intensities if include_intensity and intensities else None)

    def _lidar_callback(self, msg: PointCloud2) -> None:
        timestamp = self._extract_timestamp(msg)
        frame_id = self._extract_frame_id(msg, _DEFAULT_FRAME_ID_LIDAR)

        points: list[tuple[float, float, float]] = []
        intensities: list[float] | None = None
        metadata: dict[str, Any] = {
            "source": "ros2",
            "topic": getattr(self._lidar_sub, "topic_name", self._parent._lidar_topic),
        }

        try:
            points, intensities = self._iter_points(msg, include_intensity=True)
        except Exception:
            try:
                points, intensities = self._iter_points(msg, include_intensity=False)
            except Exception:
                points = []
                intensities = None

        if not points:
            metadata["raw_msg"] = msg

        self._parent._set_latest_lidar_frame(
            LidarFrame(
                timestamp=timestamp,
                frame_id=frame_id,
                points_xyz=points,
                intensities=intensities,
                metadata=metadata,
            )
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        intrinsics = self._parse_intrinsics(msg)
        if intrinsics is not None:
            self._parent._set_camera_intrinsics(intrinsics)

    def _camera_callback(self, msg: Image) -> None:
        timestamp = self._extract_timestamp(msg)
        frame_id = self._extract_frame_id(msg, _DEFAULT_FRAME_ID_CAMERA)
        metadata: dict[str, Any] = {
            "source": "ros2",
            "encoding": str(getattr(msg, "encoding", "")),
            "width": int(getattr(msg, "width", 0) or 0),
            "height": int(getattr(msg, "height", 0) or 0),
        }

        self._parent._set_latest_camera_frame(
            CameraFrame(
                timestamp=timestamp,
                frame_id=frame_id,
                image=self._decode_image(msg),
                intrinsics=self._parent._get_camera_intrinsics_copy(),
                metadata=metadata,
            )
        )
