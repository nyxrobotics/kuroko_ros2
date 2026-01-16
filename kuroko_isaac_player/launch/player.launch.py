import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("kuroko_isaac_player")
    params = os.path.join(share, "config", "player.yaml")

    return LaunchDescription(
        [
            Node(
                package="kuroko_isaac_player",
                executable="policy_player",
                name="isaac_policy_player",
                output="screen",
                parameters=[params],
            )
        ]
    )
