#!/usr/bin/env python3
"""Spawn Kuroko into an existing Gazebo world.

(This file name is intentionally 'spawn_kuroko.launch.py' as requested.)

Publishes TF (robot_state_publisher) and spawns the entity via ros_gz_sim/create using robot_description.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)

    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true", "yes")

    xacro_file = str(
        FindPackageShare("kuroko_description").find("kuroko_description")
        + "/xacro/kuroko/kuroko.xacro"
    )

    doc_rviz = xacro.process_file(
        xacro_file,
        mappings={
            # Rviz URDF: mimic enabled, closed loops disabled
            "gazebo": "false",
            "robot_name": robot_name,
        },
    )
    robot_description_rviz = doc_rviz.toxml()

    doc_gz = xacro.process_file(
        xacro_file,
        mappings={
            # GZ URDF: mimic disabled, closed loops enabled
            "gazebo": "true",
            "robot_name": robot_name,
        },
    )
    robot_description_gz = doc_gz.toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_rviz,
            }
        ],
    )

    create = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            world_name,
            "-name",
            robot_name,
            "-param",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-Y",
            yaw,
        ],
        parameters=[
            {
                "robot_description": robot_description_gz,
            }
        ],
    )

    return [rsp, create]


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
            OpaqueFunction(function=_launch_setup),
        ]
    )
