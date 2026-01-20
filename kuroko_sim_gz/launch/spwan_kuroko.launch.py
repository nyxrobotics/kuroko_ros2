#!/usr/bin/env python3
"""Spawn Kuroko into an existing Gazebo world.

Publishes TF (robot_state_publisher) and spawns the entity via ros_gz_sim/create using robot_description.

This launch forwards xacro args:
  - use_pid (bool)
  - use_position_control (bool)

Important:
  - use_sim_time must be a boolean ROS parameter. Passing LaunchConfiguration into Node(parameters=...)
    can turn it into a string in some launch setups. This file sets use_sim_time via CLI arguments
    ("--ros-args -p use_sim_time:=...") to ensure correct typing.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def _to_xacro_bool(value: str) -> str:
    v = value.strip().lower()
    if v in ("1", "true", "yes", "on"):
        return "true"
    if v in ("0", "false", "no", "off"):
        return "false"
    return "false"


def _launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)

    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context).strip().lower()
    if use_sim_time_str not in ("true", "false"):
        use_sim_time_str = "true"

    use_pid = _to_xacro_bool(LaunchConfiguration("use_pid").perform(context))
    use_position_control = _to_xacro_bool(LaunchConfiguration("use_position_control").perform(context))

    xacro_file = str(
        FindPackageShare("kuroko_description").find("kuroko_description")
        + "/xacro/kuroko/kuroko.xacro"
    )

    # robot_state_publisher description (gazebo:=false)
    doc_rviz = xacro.process_file(
        xacro_file,
        mappings={
            "gazebo": "false",
            "robot_name": robot_name,
            "use_pid": use_pid,
            "use_position_control": use_position_control,
        },
    )
    robot_description_rviz = doc_rviz.toxml()

    # Gazebo spawn description (gazebo:=true)
    doc_gz = xacro.process_file(
        xacro_file,
        mappings={
            "gazebo": "true",
            "robot_name": robot_name,
            "use_pid": use_pid,
            "use_position_control": use_position_control,
        },
    )
    robot_description_gz = doc_gz.toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_rviz,
            }
        ],
        arguments=["--ros-args", "-p", f"use_sim_time:={use_sim_time_str}"],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-world",
            world_name,
            "-string",
            robot_description_gz,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-Y",
            yaw,
        ],
    )

    return [rsp, spawn]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="kuroko", description="Entity name in Gazebo."),
            DeclareLaunchArgument("world_name", default_value="empty", description="Gazebo world name."),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0", description="Yaw (rad)."),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_pid", default_value="true", description="Forwarded to xacro: use_pid"),
            DeclareLaunchArgument(
                "use_position_control",
                default_value="true",
                description="Forwarded to xacro: use_position_control",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
