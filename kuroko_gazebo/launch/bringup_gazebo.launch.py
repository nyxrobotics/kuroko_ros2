#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    robot_z = LaunchConfiguration("robot_z")
    debug_control = LaunchConfiguration("debug_control")
    controller = LaunchConfiguration("controller")
    command_interface = LaunchConfiguration("command_interface")
    controller_yaml = LaunchConfiguration("controller_yaml")

    kuroko_description_share = get_package_share_directory("kuroko_description")
    kuroko_gazebo_share = get_package_share_directory("kuroko_gazebo")

    # Default: trajectory control YAML (single file)
    default_controller_yaml = (
        f"{kuroko_description_share}/config/ros2_control/joint_trajectory_controller.yaml"
    )

    declare_args = [
        DeclareLaunchArgument("robot_z", default_value="0.35"),
        DeclareLaunchArgument("debug_control", default_value="false"),
        DeclareLaunchArgument("controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("command_interface", default_value="position"),
        DeclareLaunchArgument("controller_yaml", default_value=default_controller_yaml),
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
            "controller_yaml": controller_yaml,
            "debug_control": debug_control,
        }.items(),
    )

    # Spawn controllers after gazebo_ros2_control had time to start controller_manager.
    # Order matters: joint_state_broadcaster first, then your selected controller.
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_main = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_controllers_delayed = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=["[kuroko_gazebo] controller_yaml=", controller_yaml]),
            LogInfo(msg=["[kuroko_gazebo] controller=", controller]),
            LogInfo(msg=["[kuroko_gazebo] command_interface=", command_interface]),
            spawn_jsb,
            TimerAction(period=2.0, actions=[spawn_main]),
        ],
    )

    return LaunchDescription(
        declare_args
        + [
            gazebo,
            spawn_kuroko,
            # spawn_controllers_delayed,
        ]
    )
