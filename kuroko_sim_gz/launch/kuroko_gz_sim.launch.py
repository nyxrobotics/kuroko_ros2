#!/usr/bin/env python3
"""Top-level launch: Gazebo Sim world + Kuroko spawn + /clock bridge + ros2_control controllers.

- Avoids hard-coding the *kuroko_sim_gz* package name by resolving this package's share directory via __file__.
- Starts Gazebo world (paused by default), spawns Kuroko, bridges /clock from Gazebo to ROS 2,
  then spawns ros2_control controllers.

Controller YAML lives in *kuroko_description*, so we resolve that via FindPackageShare.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _this_pkg_share() -> Path:
    # .../share/<pkg>/launch/<file>.launch.py  -> parent.parent = .../share/<pkg>
    return Path(__file__).resolve().parent.parent


def generate_launch_description() -> LaunchDescription:
    pkg_share = _this_pkg_share()
    launch_dir = pkg_share / "launch"

    world_sdf = LaunchConfiguration("world_sdf")
    world_name = LaunchConfiguration("world_name")
    robot_name = LaunchConfiguration("robot_name")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    controller_manager = LaunchConfiguration("controller_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # World (paused by default)
    spawn_world_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "spawn_world.launch.py")),
        launch_arguments={
            "world_sdf": world_sdf,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Robot
    spawn_kuroko_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "spwan_kuroko.launch.py")),
        launch_arguments={
            "robot_name": robot_name,
            "world_name": world_name,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # /clock bridge (Gazebo -> ROS 2)
    clock_bridge_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "gz_clock_bridge.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Controllers
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    controllers_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "trajectory_controller.launch.py")),
        launch_arguments={
            "controllers_yaml": controllers_yaml,
            "controller_manager": controller_manager,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    kuroko_description_share = FindPackageShare("kuroko_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_sdf",
                default_value="empty.sdf",
                description="World SDF file (relative to this package's 'worlds' directory).",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="empty",
                description="Gazebo world name (used by ros_gz_sim create -world).",
            ),
            DeclareLaunchArgument("robot_name", default_value="kuroko", description="Entity name in Gazebo."),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.35"),
            DeclareLaunchArgument("yaw", default_value="3.14", description="Yaw (rad)."),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value=[kuroko_description_share, "/config/gz_trajectory_controller.yaml"],
                description="Controller YAML (lives in kuroko_description/config).",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="controller_manager node name/namespace to contact.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            spawn_world_inc,
            spawn_kuroko_inc,
            clock_bridge_inc,
            controllers_inc,
        ]
    )
