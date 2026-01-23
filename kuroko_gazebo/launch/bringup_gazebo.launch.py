#!/usr/bin/env python3
# Copyright ...
#
# Launch Gazebo Classic and spawn Kuroko, then start ros2_control controllers.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
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
            description="position or effort (matches the transmissions you include)",
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
            description="Main controller name to load/start after joint_state_broadcaster",
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
            PathJoinSubstitution([FindPackageShare("kuroko_gazebo"), "launch", "spawn_kuroko_gazebo_classic.launch.py"])
        ),
        launch_arguments={
            "robot_z": robot_z,
            "gazebo": TextSubstitution(text="true"),
            "gazebo_hardware_interface": gazebo_hardware_interface,
            "controller_yaml": controller_yaml,
            "controller": controller,
        }.items(),
    )

    return LaunchDescription(declare_args + [gazebo, spawn_kuroko])
