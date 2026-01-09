#!/usr/bin/env python3
"""Bridge Gazebo Sim clock (/clock) to ROS 2.

Starts ros_gz_bridge/parameter_bridge for:
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

Usage:
  ros2 launch <your_pkg> gz_bridge.launch.py

Notes:
- Provides /clock publisher in ROS 2 so nodes using use_sim_time can run.
- If Gazebo is paused, /clock will not advance (hz may show 0).
- Ensure ROS_DOMAIN_ID and RMW_IMPLEMENTATION match between Gazebo and ROS 2 processes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            clock_bridge,
        ]
    )
