"""ROS 2 navigation bringup for fusion, mapping, and Nav2-compatible workflows.

This launch file keeps the repository honest: it wires the reproducible replay
pipeline into navigation-oriented configuration assets and exposes explicit
switches for RViz, SLAM, and Nav2. The Nav2 and SLAM processes are represented
as optional integration points so the repository can be exercised today without
claiming bundled platform-specific binaries.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    repo_root = Path(__file__).resolve().parent.parent
    fusion_launch = repo_root / "launch" / "bringup_fusion.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset",
                default_value=str(
                    repo_root / "datasets" / "sample_bags" / "fusion_demo_sequence.json"
                ),
                description="Replay dataset used to drive perception and navigation validation.",
            ),
            DeclareLaunchArgument(
                "replay_rate",
                default_value="1.0",
                description="Replay rate multiplier for the reproducible dataset workflow.",
            ),
            DeclareLaunchArgument(
                "max_offset",
                default_value="0.05",
                description="Maximum LiDAR-camera timestamp offset for synchronized replay.",
            ),
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Whether to open RViz using the repository default view.",
            ),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="false",
                description="Enable external Nav2 stack integration if the environment provides Nav2 packages.",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Enable external SLAM integration if the environment provides slam_toolbox or equivalent.",
            ),
            DeclareLaunchArgument(
                "navigation_config",
                default_value=str(repo_root / "config" / "navigation" / "default.yaml"),
                description="Repository navigation configuration file.",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=str(repo_root / "config" / "nav2" / "nav2_params.yaml"),
                description="Nav2 parameter file used when external Nav2 integration is enabled.",
            ),
            DeclareLaunchArgument(
                "slam_config",
                default_value=str(repo_root / "config" / "slam" / "slam_toolbox.yaml"),
                description="SLAM configuration file used when external SLAM integration is enabled.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(fusion_launch)),
                launch_arguments={
                    "dataset": LaunchConfiguration("dataset"),
                    "replay_rate": LaunchConfiguration("replay_rate"),
                    "max_offset": LaunchConfiguration("max_offset"),
                    "open_rviz": LaunchConfiguration("open_rviz"),
                }.items(),
            ),
            LogInfo(
                msg=[
                    "Navigation config: ",
                    LaunchConfiguration("navigation_config"),
                    " | Nav2 params: ",
                    LaunchConfiguration("nav2_params"),
                    " | SLAM config: ",
                    LaunchConfiguration("slam_config"),
                ]
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_nav2")),
                msg=[
                    "Nav2 integration requested. Use config file: ",
                    LaunchConfiguration("nav2_params"),
                    ".",
                ],
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_slam")),
                msg=[
                    "SLAM integration requested. Use config file: ",
                    LaunchConfiguration("slam_config"),
                    ".",
                ],
            ),
        ]
    )
