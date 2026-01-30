from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_sdf", default_value="/tmp/kuroko_usd_out/kuroko.sdf"),
        DeclareLaunchArgument("output_usda", default_value=""),
        DeclareLaunchArgument("isaac_python", default_value="python.sh"),
        DeclareLaunchArgument("isaac_export_script", default_value=""),
        DeclareLaunchArgument("work_dir", default_value=""),
        Node(
            package="kuroko_isaac_config_generator",
            executable="sdf_to_usda_node.py",
            name="sdf_to_usda",
            output="screen",
            parameters=[{
                "input_sdf": LaunchConfiguration("input_sdf"),
                "output_usda": LaunchConfiguration("output_usda"),
                "isaac_python": LaunchConfiguration("isaac_python"),
                "isaac_export_script": LaunchConfiguration("isaac_export_script"),
                "work_dir": LaunchConfiguration("work_dir"),
            }],
        ),
    ])
