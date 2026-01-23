#!/usr/bin/env python3
# Launch Gazebo Classic, spawn Kuroko, then start ros2_control controllers.
#
# Policy:
# - The entity spawned into Gazebo is generated with gazebo:=true (Gazebo plugin tags included).
# - robot_state_publisher (robot_description) uses gazebo:=false (no Gazebo-specific tags).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    robot_z = LaunchConfiguration("robot_z")
    gazebo_hardware_interface = LaunchConfiguration("gazebo_hardware_interface")
    controller_yaml = LaunchConfiguration("controller_yaml")
    controller = LaunchConfiguration("controller")

    declare_args = [
        DeclareLaunchArgument(
            "world",
            default_value=TextSubstitution(
                text=os.path.join(
                    get_package_share_directory("kuroko_gazebo"),
                    "worlds",
                    "default.world",
                )
            ),
        ),
        DeclareLaunchArgument("robot_z", default_value="0.35"),
        DeclareLaunchArgument(
            "gazebo_hardware_interface",
            default_value="position",
            description="position or effort (matches ros2_control command interfaces)",
        ),
        DeclareLaunchArgument(
            "controller_yaml",
            default_value=TextSubstitution(
                text=os.path.join(
                    get_package_share_directory("kuroko_description"),
                    "config",
                    "ros2_control",
                    "joint_trajectory_controller.yaml",
                )
            ),
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="joint_trajectory_controller",
            description="Main controller name to spawn after joint_state_broadcaster",
        ),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world}.items(),
    )

    spawn_kuroko = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("kuroko_gazebo"), "launch", "spawn_kuroko_gazebo_classic.launch.py"]
            )
        ),
        launch_arguments={
            "robot_z": robot_z,
            "gazebo_hardware_interface": gazebo_hardware_interface,
            "controller_yaml": controller_yaml,
        }.items(),
    )

    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("kuroko_gazebo"), "launch", "spawn_controllers.launch.py"])
        ),
        launch_arguments={
            "controller": controller,
            "controller_manager": TextSubstitution(text="/controller_manager"),
        }.items(),
    )

    return LaunchDescription(declare_args + [gazebo, spawn_kuroko, spawn_controllers])
