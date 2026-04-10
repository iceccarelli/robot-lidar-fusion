"""ROS 2 launch entry point for the bundled robot-lidar-fusion replay demo.

This bringup publishes the repository-local sample dataset as ROS 2 topics and
optionally opens RViz with the default visualisation profile. The goal is to
provide a reproducible, no-hardware demo path built around deterministic local
assets rather than custom sensor drivers.
"""

from __future__ import annotations

from pathlib import Path

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription


def _create_actions(context, *args, **kwargs):
    repo_root = Path(__file__).resolve().parent.parent
    dataset_default = repo_root / "datasets" / "sample_bags" / "fusion_demo_sequence.json"
    rviz_default = repo_root / "rviz" / "default.rviz"

    dataset = LaunchConfiguration("dataset").perform(context)
    replay_rate = LaunchConfiguration("replay_rate").perform(context)

    publisher_cmd = [
        "python",
        str(repo_root / "scripts" / "ros2_replay_publisher.py"),
        "--dataset",
        dataset or str(dataset_default),
        "--rate",
        replay_rate,
    ]

    return [
        ExecuteProcess(
            cmd=publisher_cmd,
            output="screen",
            name="robot_lidar_fusion_demo_replay_publisher",
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("open_rviz")),
            cmd=["rviz2", "-d", str(rviz_default)],
            output="screen",
            name="robot_lidar_fusion_rviz",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    repo_root = Path(__file__).resolve().parent.parent
    dataset_default = repo_root / "datasets" / "sample_bags" / "fusion_demo_sequence.json"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset",
                default_value=str(dataset_default),
                description="Path to the replay dataset JSON file.",
            ),
            DeclareLaunchArgument(
                "replay_rate",
                default_value="1.0",
                description="Replay rate multiplier for the bundled dataset.",
            ),
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Whether to open RViz with the default configuration.",
            ),
            OpaqueFunction(function=_create_actions),
        ]
    )
