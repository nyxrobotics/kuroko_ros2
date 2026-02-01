from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_sdf", default_value="/tmp/kuroko_usd_out/kuroko.sdf"),
            DeclareLaunchArgument("output_usda", default_value="/tmp/kuroko_usd_out/kuroko.usda"),
            # Path to Isaac Lab helper script. We run Kit Python via:
            #   isaaclab.sh --python <script.py> [args...]
            DeclareLaunchArgument("isaaclab_sh", default_value="isaaclab.sh"),
            DeclareLaunchArgument("headless", default_value="true"),
            Node(
                package="kuroko_isaac_config_generator",
                executable="sdf_to_usda_node.py",
                name="sdf_to_usda",
                output="screen",
                parameters=[
                    {
                        "input_sdf": LaunchConfiguration("input_sdf"),
                        "output_usda": LaunchConfiguration("output_usda"),
                        "isaaclab_sh": LaunchConfiguration("isaaclab_sh"),
                        "headless": LaunchConfiguration("headless"),
                    }
                ],
            ),
        ]
    )
