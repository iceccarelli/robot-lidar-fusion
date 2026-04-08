"""ROS 2 simulation bringup for Gazebo and Isaac Sim adapter workflows.

This launch file provides an honest entry point for simulation-backed
validation. It reuses the repository's deterministic replay-driven fusion and
navigation stack, while surfacing the adapter contracts and configuration files
needed to connect either Gazebo or Isaac Sim in a ROS 2 environment.
"""

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    repo_root = Path(__file__).resolve().parent.parent
    navigation_launch = repo_root / "launch" / "bringup_navigation.launch.py"

    simulator = LaunchConfiguration("simulator")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulator",
                default_value="gazebo",
                description="Simulation backend contract to target: 'gazebo' or 'isaac_sim'.",
            ),
            DeclareLaunchArgument(
                "dataset",
                default_value=str(repo_root / "datasets" / "sample_bags" / "fusion_demo_sequence.json"),
                description="Replay dataset used to validate the simulation-connected workflow.",
            ),
            DeclareLaunchArgument(
                "replay_rate",
                default_value="1.0",
                description="Replay rate multiplier for the deterministic demo sequence.",
            ),
            DeclareLaunchArgument(
                "max_offset",
                default_value="0.05",
                description="Maximum LiDAR-camera timestamp offset in seconds.",
            ),
            DeclareLaunchArgument(
                "open_rviz",
                default_value="true",
                description="Whether to open RViz during simulation bringup.",
            ),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="true",
                description="Expose Nav2-compatible configuration during simulation bringup.",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="true",
                description="Expose SLAM-compatible configuration during simulation bringup.",
            ),
            DeclareLaunchArgument(
                "navigation_config",
                default_value=str(repo_root / "config" / "navigation" / "default.yaml"),
                description="Repository navigation configuration file.",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=str(repo_root / "config" / "nav2" / "nav2_params.yaml"),
                description="Nav2 parameter file.",
            ),
            DeclareLaunchArgument(
                "slam_config",
                default_value=str(repo_root / "config" / "slam" / "slam_toolbox.yaml"),
                description="SLAM configuration file.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(navigation_launch)),
                launch_arguments={
                    "dataset": LaunchConfiguration("dataset"),
                    "replay_rate": LaunchConfiguration("replay_rate"),
                    "max_offset": LaunchConfiguration("max_offset"),
                    "open_rviz": LaunchConfiguration("open_rviz"),
                    "enable_nav2": LaunchConfiguration("enable_nav2"),
                    "enable_slam": LaunchConfiguration("enable_slam"),
                    "navigation_config": LaunchConfiguration("navigation_config"),
                    "nav2_params": LaunchConfiguration("nav2_params"),
                    "slam_config": LaunchConfiguration("slam_config"),
                }.items(),
            ),
            LogInfo(
                condition=IfCondition(PythonExpression([simulator, " == 'gazebo'"])),
                msg="Simulation contract selected: Gazebo adapter topics and bridges are expected.",
            ),
            LogInfo(
                condition=IfCondition(PythonExpression([simulator, " == 'isaac_sim'"])),
                msg="Simulation contract selected: Isaac Sim ROS 2 bridge topics are expected.",
            ),
            LogInfo(
                msg=[
                    "Simulation bringup active with backend '",
                    simulator,
                    "'. Use benchmark scenarios and artifacts for regression evidence.",
                ]
            ),
        ]
    )
