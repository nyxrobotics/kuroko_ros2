#!/usr/bin/env python3

import re
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _xacro_to_urdf_one_line(xacro_path: str, xacro_args: list[str]) -> str:
    cmd = ["xacro", xacro_path] + xacro_args
    urdf = subprocess.check_output(cmd, text=True)

    # Drop XML comments to reduce size/noise
    urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.DOTALL)

    # Make it single-line (avoids rcl argument parsing issues in gazebo_ros2_control)
    urdf = urdf.replace("\n", " ").replace("\r", " ")
    urdf = re.sub(r"\s+", " ", urdf).strip()
    return urdf


def _runtime_setup(context, *args, **kwargs):
    robot_z = LaunchConfiguration("robot_z").perform(context)
    gazebo = LaunchConfiguration("gazebo").perform(context)
    gz_sim = LaunchConfiguration("gz_sim").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    command_interface = LaunchConfiguration("command_interface").perform(context)
    controller_yaml = LaunchConfiguration("controller_yaml").perform(context)
    debug_control = LaunchConfiguration("debug_control").perform(context)

    kuroko_description_share = get_package_share_directory("kuroko_description")
    xacro_file = f"{kuroko_description_share}/xacro/kuroko/kuroko.xacro"

    urdf_one_line = _xacro_to_urdf_one_line(
        xacro_file,
        [
            f"gazebo:={gazebo}",
            f"gz_sim:={gz_sim}",
            f"controller:={controller}",
            f"command_interface:={command_interface}",
            f"controller_yaml:={controller_yaml}",
            f"debug_control:={debug_control}",
        ],
    )

    robot_description = ParameterValue(urdf_one_line, value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn_kuroko = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "kuroko",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            robot_z,
            "--ros-args",
            "-r",
            "__node:=spawn_kuroko",
        ],
        output="screen",
    )

    return [robot_state_publisher, spawn_kuroko]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_z", default_value="0.35"),
            DeclareLaunchArgument("gazebo", default_value="true"),
            DeclareLaunchArgument("gz_sim", default_value="false"),
            DeclareLaunchArgument("controller", default_value="joint_trajectory_controller"),
            DeclareLaunchArgument("command_interface", default_value="position"),
            DeclareLaunchArgument("controller_yaml", default_value=""),
            DeclareLaunchArgument("debug_control", default_value="false"),
            OpaqueFunction(function=_runtime_setup),
        ]
    )
