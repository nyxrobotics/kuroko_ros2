#!/usr/bin/env python3
"""Spawn ros2_control controllers for gz_ros2_control.

- joint_state_broadcaster
- joint_group_position_controller (position command interface)

Designed to tolerate Gazebo starting PAUSED:
- controller_manager_timeout: huge (wait for /controller_manager)
- switch_timeout: huge (wait for activation)
- service_call_timeout: huge (avoid the 10s default per-call timeout)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    controller_manager = LaunchConfiguration("controller_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")

    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")
    switch_timeout = LaunchConfiguration("switch_timeout")
    service_call_timeout = LaunchConfiguration("service_call_timeout")

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            controller_manager_timeout,
            "--switch-timeout",
            switch_timeout,
            "--service-call-timeout",
            service_call_timeout,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    pos_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_group_position_controller",
            "--controller-manager",
            controller_manager,
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            controller_manager_timeout,
            "--switch-timeout",
            switch_timeout,
            "--service-call-timeout",
            service_call_timeout,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_jsb = TimerAction(period=3.0, actions=[jsb_spawner])
    delayed_pos = TimerAction(period=4.0, actions=[pos_spawner])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                description="YAML file for controller_manager",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="controller_manager node name/namespace to contact.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="86400"),
            DeclareLaunchArgument("switch_timeout", default_value="86400"),
            DeclareLaunchArgument("service_call_timeout", default_value="86400"),
            delayed_jsb,
            delayed_pos,
        ]
    )
