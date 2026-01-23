#!/usr/bin/env python3
# Spawn Kuroko into Gazebo Classic.
#
# Requirement:
# - Model spawned into Gazebo must be generated with gazebo:=true (Gazebo plugin tags included).
# - Other consumers of robot_description should use gazebo:=false (no Gazebo-specific tags).

import os
import re
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _xacro_to_urdf_one_line(xacro_path: str, xacro_args: list[str]) -> str:
    cmd = ["xacro", xacro_path] + xacro_args
    urdf = subprocess.check_output(cmd, text=True)
    urdf = re.sub(r"<!--.*?-->", "", urdf, flags=re.DOTALL)
    urdf = urdf.replace("\n", " ").replace("\r", " ")
    urdf = re.sub(r"\s+", " ", urdf).strip()
    return urdf


def _runtime_setup(context, *args, **kwargs):
    robot_z = LaunchConfiguration("robot_z").perform(context)
    gazebo_hardware_interface = LaunchConfiguration("gazebo_hardware_interface").perform(context)
    controller_yaml = LaunchConfiguration("controller_yaml").perform(context)

    xacro_file = os.path.join(
        get_package_share_directory("kuroko_description"),
        "xacro",
        "kuroko",
        "kuroko.xacro",
    )

    # URDF for spawning into Gazebo (must include Gazebo plugin tags)
    urdf_spawn_one_line = _xacro_to_urdf_one_line(
        xacro_file,
        [
            "gazebo:=true",
            f"gazebo_hardware_interface:={gazebo_hardware_interface}",
            f"controller_yaml:={controller_yaml}",
        ],
    )

    # URDF for robot_description consumers (prefer no Gazebo-specific tags)
    urdf_clean_one_line = _xacro_to_urdf_one_line(
        xacro_file,
        [
            "gazebo:=false",
            f"gazebo_hardware_interface:={gazebo_hardware_interface}",
            f"controller_yaml:={controller_yaml}",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_clean_one_line}],
    )

    # Gazebo Classic spawn_entity.py (ROS 2 Humble) requires one of -file/-topic/-database/-stdin.
    # Using -file is the most robust and avoids extra topic publishers.
    tmpdir = tempfile.gettempdir()
    urdf_path = os.path.join(tmpdir, "kuroko_spawn.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(urdf_spawn_one_line)

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-file",
            urdf_path,
            "-entity",
            "kuroko",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            robot_z,
            "-Y",
            "0.0",
        ],
    )

    return [robot_state_publisher, spawn_entity]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_z", default_value="0.35"),
            DeclareLaunchArgument("gazebo_hardware_interface", default_value="position"),
            DeclareLaunchArgument("controller_yaml", default_value=""),
            OpaqueFunction(function=_runtime_setup),
        ]
    )
