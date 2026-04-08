"""Gazebo simulation adapter for deterministic replay, mapping, and navigation workflows.

The adapter provides a light-weight contract rather than hard-binding the
repository to a specific Gazebo installation. This keeps the project honest and
reproducible in CI while still defining the exact data that a Gazebo-based
runtime integration must supply to perception, navigation, telemetry, and
benchmark tooling.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GazeboAdapterConfig:
    world_name: str = "fusion_navigation_demo"
    robot_name: str = "robot_lidar_fusion"
    lidar_topic: str = "/lidar/points"
    camera_topic: str = "/camera/color/image_raw"
    camera_info_topic: str = "/camera/color/camera_info"
    odom_topic: str = "/odom"
    clock_topic: str = "/clock"
    use_sim_time: bool = True


class GazeboAdapter:
    """Describe and normalize Gazebo-facing simulation integration points."""

    def __init__(self, config: GazeboAdapterConfig | None = None) -> None:
        self.config = config or GazeboAdapterConfig()

    def describe_topics(self) -> dict[str, str]:
        return {
            "lidar": self.config.lidar_topic,
            "camera": self.config.camera_topic,
            "camera_info": self.config.camera_info_topic,
            "odometry": self.config.odom_topic,
            "clock": self.config.clock_topic,
        }

    def build_launch_hints(self) -> dict[str, Any]:
        return {
            "simulator": "gazebo",
            "world_name": self.config.world_name,
            "robot_name": self.config.robot_name,
            "use_sim_time": self.config.use_sim_time,
            "topics": self.describe_topics(),
            "required_bridge_nodes": [
                "parameter_bridge",
                "image_transport",
                "pointcloud_to_laserscan",
            ],
        }

    def normalize_runtime_state(self, state: dict[str, Any]) -> dict[str, Any]:
        normalized = dict(state)
        normalized.setdefault("simulator", "gazebo")
        normalized.setdefault("use_sim_time", self.config.use_sim_time)
        normalized.setdefault("topic_contract", self.describe_topics())
        return normalized
