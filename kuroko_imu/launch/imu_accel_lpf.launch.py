from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/kuroko/sensors/imu/data"),
        DeclareLaunchArgument("output_topic", default_value="/kuroko/sensors/imu/data_accel_lpf"),
        DeclareLaunchArgument("use_header_stamp", default_value="true"),
        DeclareLaunchArgument("min_dt", default_value="0.0001"),
        DeclareLaunchArgument("time_constant_norm_sec", default_value="0.01"),
        DeclareLaunchArgument("time_constant_dir_sec", default_value="0.01"),
        Node(
            package="kuroko_imu",
            executable="imu_accel_lpf",
            name="imu_accel_lpf",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "use_header_stamp": LaunchConfiguration("use_header_stamp"),
                "min_dt": LaunchConfiguration("min_dt"),
                "time_constant_norm_sec": LaunchConfiguration("time_constant_norm_sec"),
                "time_constant_dir_sec": LaunchConfiguration("time_constant_dir_sec"),
            }],
        ),
    ])
