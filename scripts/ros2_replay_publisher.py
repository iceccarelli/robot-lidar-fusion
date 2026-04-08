#!/usr/bin/env python3
"""Publish the bundled demo dataset as ROS 2 topics for RViz inspection.

This script is intentionally lightweight and repository-local. It provides a
reproducible, no-hardware bringup path that publishes LiDAR points, camera
images, camera intrinsics, and simple debug markers from the bundled dataset.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

DEFAULT_DATASET = Path(__file__).resolve().parent.parent / "datasets" / "sample_bags" / "fusion_demo_sequence.json"

try:
    import rclpy
    from builtin_interfaces.msg import Time as RosTime
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
except Exception as exc:  # pragma: no cover - environment dependent
    print(f"ros2 replay publisher requires a ROS 2 Python environment: {exc}", file=sys.stderr)
    raise SystemExit(2)


def load_dataset(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def to_ros_time(value: float) -> RosTime:
    seconds = int(value)
    nanoseconds = int((value - seconds) * 1_000_000_000)
    return RosTime(sec=seconds, nanosec=nanoseconds)


def build_pointcloud(header: Header, lidar_payload: dict[str, Any]) -> PointCloud2:
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    points = [tuple(map(float, point)) for point in lidar_payload["points_xyz"]]
    return point_cloud2.create_cloud(header, fields, points)


def build_image(header: Header, camera_payload: dict[str, Any]) -> Image:
    image_payload = camera_payload["image"]
    width = int(image_payload["width"])
    height = int(image_payload["height"])
    encoding = str(image_payload.get("encoding", "rgb8"))
    pixel_rows = image_payload["data"]
    flattened = bytearray()
    for row in pixel_rows:
        for pixel in row:
            flattened.extend(int(channel) for channel in pixel)
    return Image(
        header=header,
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=0,
        step=width * 3,
        data=bytes(flattened),
    )


def build_camera_info(header: Header, camera_payload: dict[str, Any]) -> CameraInfo:
    image_payload = camera_payload["image"]
    intrinsics = camera_payload.get("intrinsics", {})
    fx = float(intrinsics.get("fx", 0.0))
    fy = float(intrinsics.get("fy", 0.0))
    cx = float(intrinsics.get("cx", 0.0))
    cy = float(intrinsics.get("cy", 0.0))
    distortion = [float(value) for value in intrinsics.get("D", [0.0] * 5)]
    return CameraInfo(
        header=header,
        width=int(image_payload["width"]),
        height=int(image_payload["height"]),
        distortion_model="plumb_bob",
        d=distortion,
        k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    )


def build_debug_markers(header: Header, lidar_payload: dict[str, Any]) -> MarkerArray:
    markers: list[Marker] = []
    for index, point_xyz in enumerate(lidar_payload["points_xyz"]):
        marker = Marker()
        marker.header = header
        marker.ns = "fused_debug"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=float(point_xyz[0]), y=float(point_xyz[1]), z=float(point_xyz[2]))
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 0.95
        markers.append(marker)
    return MarkerArray(markers=markers)


class DemoReplayPublisher(Node):
    def __init__(self, dataset_path: Path, replay_rate: float) -> None:
        super().__init__("robot_lidar_fusion_demo_replay")
        self._dataset = load_dataset(dataset_path)
        self._frames = list(self._dataset.get("frames", []))
        self._replay_rate = max(float(replay_rate), 1e-6)
        self._lidar_pub = self.create_publisher(PointCloud2, "/robot_lidar_fusion/demo/lidar_points", 10)
        self._image_pub = self.create_publisher(Image, "/robot_lidar_fusion/demo/camera_image", 10)
        self._camera_info_pub = self.create_publisher(CameraInfo, "/robot_lidar_fusion/demo/camera_info", 10)
        self._debug_pub = self.create_publisher(MarkerArray, "/robot_lidar_fusion/demo/fused_debug", 10)

    def run(self) -> None:
        self.get_logger().info(
            f"publishing dataset={self._dataset.get('dataset')} frames={len(self._frames)} rate={self._replay_rate}"
        )
        previous_timestamp: float | None = None
        for item in self._frames:
            lidar_payload = item["lidar"]
            camera_payload = item["camera"]
            lidar_header = Header(frame_id=str(lidar_payload["frame_id"]), stamp=to_ros_time(float(lidar_payload["timestamp"])))
            camera_header = Header(frame_id=str(camera_payload["frame_id"]), stamp=to_ros_time(float(camera_payload["timestamp"])))
            self._lidar_pub.publish(build_pointcloud(lidar_header, lidar_payload))
            self._image_pub.publish(build_image(camera_header, camera_payload))
            self._camera_info_pub.publish(build_camera_info(camera_header, camera_payload))
            self._debug_pub.publish(build_debug_markers(lidar_header, lidar_payload))
            self.get_logger().info(
                "published seq=%s lidar_ts=%.3f camera_ts=%.3f points=%d"
                % (
                    item.get("sequence_id"),
                    float(lidar_payload["timestamp"]),
                    float(camera_payload["timestamp"]),
                    len(lidar_payload["points_xyz"]),
                )
            )
            rclpy.spin_once(self, timeout_sec=0.0)
            if previous_timestamp is not None:
                delta = max(float(lidar_payload["timestamp"]) - previous_timestamp, 0.0)
                time.sleep(delta / self._replay_rate)
            previous_timestamp = float(lidar_payload["timestamp"])
        self.get_logger().info("replay complete")


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish the bundled robot-lidar-fusion demo dataset as ROS 2 topics")
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET, help="Path to the JSON replay dataset")
    parser.add_argument("--rate", type=float, default=1.0, help="Replay rate multiplier")
    args = parser.parse_args()

    rclpy.init()
    node = DemoReplayPublisher(dataset_path=args.dataset, replay_rate=args.rate)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
