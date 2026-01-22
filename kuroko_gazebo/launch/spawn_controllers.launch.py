#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    controller = LaunchConfiguration("controller")
    controller_manager = LaunchConfiguration("controller_manager")
    delay_sec = LaunchConfiguration("delay_sec")

    declare_args = [
        DeclareLaunchArgument("controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("delay_sec", default_value="3.0"),
    ]

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
        ],
        output="screen",
    )

    spawn_main = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller,
            "--controller-manager",
            controller_manager,
        ],
        output="screen",
    )

    # Spawn sequence: JSB -> main controller (with a small extra delay)
    delayed = TimerAction(
        period=delay_sec,
        actions=[
            LogInfo(msg=["[kuroko_gazebo] spawning controllers via ", controller_manager]),
            LogInfo(msg=["[kuroko_gazebo] main controller=", controller]),
            spawn_jsb,
            TimerAction(period=2.0, actions=[spawn_main]),
        ],
    )

    return LaunchDescription(declare_args + [delayed])
