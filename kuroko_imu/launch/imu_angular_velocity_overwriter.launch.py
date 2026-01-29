from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/imu/data"),
        DeclareLaunchArgument("output_topic", default_value="/imu/data_overwritten"),
        DeclareLaunchArgument("use_header_stamp", default_value="true"),
        DeclareLaunchArgument("min_dt", default_value="0.0001"),
        DeclareLaunchArgument("zero_on_first_msg", default_value="true"),
        Node(
            package="kuroko_imu",
            executable="imu_angular_velocity_overwriter",
            name="imu_angular_velocity_overwriter",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "use_header_stamp": LaunchConfiguration("use_header_stamp"),
                "min_dt": LaunchConfiguration("min_dt"),
                "zero_on_first_msg": LaunchConfiguration("zero_on_first_msg"),
            }],
        ),
    ])
