"""Top-level ROS 2 bringup for the robot-lidar-fusion demo workflow.

This launch file provides a stable entry point for the repository's
reproducible perception demo. It currently composes the bundled dataset
replay workflow and the default RViz configuration. As deeper fusion,
TF, and debug publishers are added, they should be integrated here rather
than creating separate ad hoc entry points.
"""

from __future__ import annotations

from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    repo_root = Path(__file__).resolve().parent.parent
    bag_replay_launch = repo_root / "launch" / "bringup_bag_replay.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset",
                default_value=str(
                    repo_root / "datasets" / "sample_bags" / "fusion_demo_sequence.json"
                ),
                description="Path to the replay dataset JSON file.",
            ),
            DeclareLaunchArgument(
                "replay_rate",
                default_value="1.0",
                description="Replay rate multiplier for the bundled dataset.",
            ),
            DeclareLaunchArgument(
                "max_offset",
                default_value="0.05",
                description="Maximum LiDAR-camera timestamp offset in seconds.",
            ),
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Whether to open RViz with the default configuration.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bag_replay_launch)),
                launch_arguments={
                    "dataset": LaunchConfiguration("dataset"),
                    "replay_rate": LaunchConfiguration("replay_rate"),
                    "max_offset": LaunchConfiguration("max_offset"),
                    "open_rviz": LaunchConfiguration("open_rviz"),
                }.items(),
            ),
        ]
    )
