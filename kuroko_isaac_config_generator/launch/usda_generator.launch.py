from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/kuroko_usd_out"),
        DeclareLaunchArgument("basename", default_value="kuroko"),
        DeclareLaunchArgument("gazebo", default_value="true"),
        DeclareLaunchArgument("xacro_package", default_value="kuroko_description"),
        DeclareLaunchArgument("xacro_relative_path", default_value="xacro/kuroko/kuroko.xacro"),
        DeclareLaunchArgument(
            "controller_yaml_relative_path",
            default_value="config/ros2_control/joint_group_position_controller.yaml",
        ),
        Node(
            package="kuroko_isaac_config_generator",
            executable="usda_generator_node.py",
            name="usda_generator",
            output="screen",
            parameters=[{
                "output_dir": LaunchConfiguration("output_dir"),
                "basename": LaunchConfiguration("basename"),
                "gazebo": LaunchConfiguration("gazebo"),
                "xacro_package": LaunchConfiguration("xacro_package"),
                "xacro_relative_path": LaunchConfiguration("xacro_relative_path"),
                "controller_yaml_relative_path": LaunchConfiguration("controller_yaml_relative_path"),
                "write_intermediate": True,
                "headless_isaac": True,
            }],
        )
    ])
