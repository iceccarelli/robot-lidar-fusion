"""Isaac Sim adapter for ROS 2-native perception, mapping, and navigation workflows.

The adapter captures the repository-facing contract for NVIDIA Isaac Sim without
pretending the simulator is bundled in CI. It defines the expected topic graph,
clock behavior, and state normalization rules needed to plug simulation output
into replay, fusion, mapping, planning, telemetry, and benchmark tooling.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class IsaacSimAdapterConfig:
    scenario_name: str = "fusion_navigation_demo"
    robot_prim_path: str = "/World/Robot"
    lidar_topic: str = "/sim/lidar/points"
    camera_topic: str = "/sim/camera/color/image_raw"
    camera_info_topic: str = "/sim/camera/color/camera_info"
    tf_topic: str = "/tf"
    odom_topic: str = "/odom"
    clock_topic: str = "/clock"
    use_sim_time: bool = True


class IsaacSimAdapter:
    """Describe and normalize Isaac Sim-facing integration points."""

    def __init__(self, config: IsaacSimAdapterConfig | None = None) -> None:
        self.config = config or IsaacSimAdapterConfig()

    def describe_topics(self) -> dict[str, str]:
        return {
            "lidar": self.config.lidar_topic,
            "camera": self.config.camera_topic,
            "camera_info": self.config.camera_info_topic,
            "tf": self.config.tf_topic,
            "odometry": self.config.odom_topic,
            "clock": self.config.clock_topic,
        }

    def build_launch_hints(self) -> dict[str, Any]:
        return {
            "simulator": "isaac_sim",
            "scenario_name": self.config.scenario_name,
            "robot_prim_path": self.config.robot_prim_path,
            "use_sim_time": self.config.use_sim_time,
            "topics": self.describe_topics(),
            "required_bridges": [
                "ros2_bridge",
                "clock_bridge",
                "tf_bridge",
            ],
        }

    def normalize_runtime_state(self, state: dict[str, Any]) -> dict[str, Any]:
        normalized = dict(state)
        normalized.setdefault("simulator", "isaac_sim")
        normalized.setdefault("use_sim_time", self.config.use_sim_time)
        normalized.setdefault("topic_contract", self.describe_topics())
        return normalized
