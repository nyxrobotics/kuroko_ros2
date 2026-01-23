#!/usr/bin/env python3
# Spawn Kuroko into Gazebo Classic and start ros2_control controllers robustly.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _xacro_to_urdf_one_line(xacro_path: str, mappings: list[str]) -> str:
    # xacro CLI produces a single XML. We strip newlines for spawn_entity stability.
    import subprocess

    cmd = ["xacro", xacro_path] + mappings
    urdf = subprocess.check_output(cmd, text=True)
    return " ".join(urdf.split())


def _runtime_setup(context, *args, **kwargs):
    robot_z = LaunchConfiguration("robot_z").perform(context)
    gazebo = LaunchConfiguration("gazebo").perform(context)
    gazebo_hardware_interface = LaunchConfiguration("gazebo_hardware_interface").perform(context)
    controller_yaml = LaunchConfiguration("controller_yaml").perform(context)
    controller = LaunchConfiguration("controller").perform(context)

    xacro_file = os.path.join(
        get_package_share_directory("kuroko_description"),
        "xacro",
        "kuroko",
        "kuroko.xacro",
    )

    urdf_one_line = _xacro_to_urdf_one_line(
        xacro_file,
        [
            f"gazebo:={gazebo}",
            f"gazebo_hardware_interface:={gazebo_hardware_interface}",
            f"controller_yaml:={controller_yaml}",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_one_line}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
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

    # Critical: wait for controller_manager service to exist before calling ros2 control.
    wait_for_cm = ExecuteProcess(
        cmd=["ros2", "service", "wait", "/controller_manager/list_controllers"],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_main = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "start",
            controller,
        ],
        output="screen",
    )

    # Chain: spawn -> wait service -> jsb -> main
    on_spawn_wait = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[wait_for_cm])
    )
    on_wait_load_jsb = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_cm, on_exit=[load_jsb])
    )
    on_jsb_load_main = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_main])
    )

    return [
        robot_state_publisher,
        spawn_entity,
        on_spawn_wait,
        on_wait_load_jsb,
        on_jsb_load_main,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_z", default_value="0.35"),
            DeclareLaunchArgument("gazebo", default_value="true"),
            DeclareLaunchArgument("gazebo_hardware_interface", default_value="position"),
            DeclareLaunchArgument("controller_yaml", default_value=""),
            DeclareLaunchArgument("controller", default_value="joint_trajectory_controller"),
            OpaqueFunction(function=_runtime_setup),
        ]
    )
