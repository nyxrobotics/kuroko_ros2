#!/usr/bin/env python3
"""Top-level launch: Gazebo Sim world + Kuroko spawn + /clock bridge + ros2_control controllers.

- Avoids hard-coding the *kuroko_sim_gz* package name by resolving this package's share directory via __file__.
- Starts Gazebo world (paused by default), spawns Kuroko, bridges /clock from Gazebo to ROS 2,
  then spawns ros2_control controllers.

Controller YAML lives in *kuroko_description*, so we resolve that via FindPackageShare.

This launch wires xacro options:
  - use_pid (bool)
  - use_position_control (bool)

Controller selection is driven only by use_position_control:
  - true  -> position_controller.launch.py + gz_position_controller.yaml
  - false -> trajectory_controller.launch.py + gz_trajectory_controller.yaml
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _controllers_include(context, launch_dir: Path):
    use_position = LaunchConfiguration("use_position_control").perform(context).strip().lower()
    yaml_arg = LaunchConfiguration("controllers_yaml").perform(context).strip()

    kuroko_description_share = FindPackageShare("kuroko_description").perform(context)
    default_yaml = (
        Path(kuroko_description_share) / "config" / "gz_trajectory_controller.yaml"
        if use_position == "false"
        else Path(kuroko_description_share) / "config" / "gz_position_controller.yaml"
    )
    yaml_path = yaml_arg if yaml_arg != "" else str(default_yaml)

    controller_launch = (
        launch_dir / "trajectory_controller.launch.py" if use_position == "false" else launch_dir / "position_controller.launch.py"
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(controller_launch)),
            launch_arguments={
                "controllers_yaml": yaml_path,
                "controller_manager": LaunchConfiguration("controller_manager").perform(context),
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
            }.items(),
        )
    ]


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

    use_pid = LaunchConfiguration("use_pid")
    use_position_control = LaunchConfiguration("use_position_control")

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
            "use_pid": use_pid,
            "use_position_control": use_position_control,
        }.items(),
    )

    # /clock bridge (Gazebo -> ROS 2)
    gz_bridge_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "gz_bridge.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Controllers are included via OpaqueFunction so we can switch by use_position_control.
    controllers_yaml = LaunchConfiguration("controllers_yaml")

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
                "use_pid",
                default_value="true",
                description="Forwarded to xacro: use_pid",
            ),
            DeclareLaunchArgument(
                "use_position_control",
                default_value="true",
                description="Forwarded to xacro: true=position, false=trajectory",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                default_value="",
                description="Override controller YAML (optional). If empty, chosen from use_position_control.",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="controller_manager node name/namespace to contact.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            spawn_world_inc,
            spawn_kuroko_inc,
            # gz_bridge_inc,
            # OpaqueFunction(function=lambda context: _controllers_include(context, launch_dir)),
        ]
    )
