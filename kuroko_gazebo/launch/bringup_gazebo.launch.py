#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    robot_z = LaunchConfiguration("robot_z")
    debug_control = LaunchConfiguration("debug_control")
    controller = LaunchConfiguration("controller")
    command_interface = LaunchConfiguration("command_interface")
    controller_yaml_files = LaunchConfiguration("controller_yaml_files")

    kuroko_description_share = get_package_share_directory("kuroko_description")
    kuroko_gazebo_share = get_package_share_directory("kuroko_gazebo")

    # Default YAML set for trajectory control
    default_yaml_files = ";".join(
        [
            f"{kuroko_description_share}/config/ros2_control/controller_manager.yaml",
            f"{kuroko_description_share}/config/ros2_control/joint_state_broadcaster.yaml",
            f"{kuroko_description_share}/config/ros2_control/joint_trajectory_controller.yaml",
        ]
    )

    declare_args = [
        DeclareLaunchArgument("robot_z", default_value="0.35"),
        DeclareLaunchArgument("debug_control", default_value="false"),
        DeclareLaunchArgument("controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("command_interface", default_value="position"),
        DeclareLaunchArgument("controller_yaml_files", default_value=default_yaml_files),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{get_package_share_directory('gazebo_ros')}/launch/gazebo.launch.py"
        ),
    )

    spawn_kuroko = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{kuroko_gazebo_share}/launch/spawn_kuroko_gazebo_classic.launch.py"
        ),
        launch_arguments={
            "robot_z": robot_z,
            "gazebo": TextSubstitution(text="true"),
            "gz_sim": TextSubstitution(text="false"),
            "controller": controller,
            "command_interface": command_interface,
            "controller_yaml_files": controller_yaml_files,
            "debug_control": debug_control,
        }.items(),
    )

    return LaunchDescription(
        declare_args
        + [
            gazebo,
            LogInfo(msg=["[kuroko_gazebo] controller_yaml_files=", controller_yaml_files]),
            spawn_kuroko,
        ]
    )
