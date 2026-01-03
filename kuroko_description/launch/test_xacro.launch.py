#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Args
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Paths
    pkg_share = FindPackageShare("kuroko_description")
    xacro_file = PathJoinSubstitution([pkg_share, "xacro", "kuroko", "kuroko.xacro"])

    # RViz-friendly URDF: mimic enabled, gazebo-only closed loops disabled
    robot_description = Command(
        [
            "xacro ",
            xacro_file,
            " gazebo:=false",
        ]
    )

    # Nodes
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(gui),
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 200.0,
                "use_tf_static": False,
            }
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(rviz),
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([pkg_share, "rviz", "urdf.rviz"]),
                description="RViz2 config file",
            ),
            joint_state_publisher_gui,
            robot_state_publisher,
            rviz2,
        ]
    )
