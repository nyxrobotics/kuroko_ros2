#!/usr/bin/env python3
"""Launch Gazebo Sim world only (no robot, no controllers).

This version avoids hard-coding the package name for world path resolution.
World starts PAUSED by default (no '-r').
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def _this_pkg_share() -> Path:
    return Path(__file__).resolve().parent.parent


def generate_launch_description() -> LaunchDescription:
    world_sdf = LaunchConfiguration("world_sdf")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_share = _this_pkg_share()

    # Build: <this_pkg_share>/worlds/<world_sdf>
    world_path = PathJoinSubstitution(
        [
            TextSubstitution(text=str(pkg_share)),
            "worlds",
            world_sdf,
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            # ros_gz_sim expects a single string passed to `gz sim`.
            "gz_args": world_path,
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_sdf",
                default_value="empty.sdf",
                description="World SDF file (relative to this package's 'worlds' directory).",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            gz_sim,
        ]
    )
