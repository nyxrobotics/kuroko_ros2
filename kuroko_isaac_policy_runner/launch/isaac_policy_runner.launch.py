from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    policy_name = LaunchConfiguration("policy_name")
    bundle_path = PathJoinSubstitution([
        FindPackageShare("kuroko_isaac_policy_runner"),
        "bundle",
        policy_name,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "policy_name",
            description="Policy bundle name under share/kuroko_isaac_policy_runner/bundle/<policy_name>.",
        ),
        Node(
            package="isaac_policy_runtime",
            executable="isaac_policy_runner",
            name="isaac_policy_runner",
            output="screen",
            parameters=[{"bundle_path": bundle_path}],
        ),
    ])
