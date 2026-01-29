from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Gazebo IMU commonly provides orientation and linear acceleration,
    # while angular velocity can be noisy or unavailable depending on the plugin setup.
    # This launch chains:
    #   1) imu_angular_velocity_overwriter: overwrite angular_velocity from orientation delta
    #   2) imu_accel_lpf: low-pass filter linear_acceleration (norm + direction)
    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/kuroko/sensors/imu/data"),
        DeclareLaunchArgument("intermediate_topic", default_value="/kuroko/sensors/imu/data_vel"),
        DeclareLaunchArgument("output_topic", default_value="/kuroko/sensors/imu/data_filtered"),

        # Common timing params
        DeclareLaunchArgument("use_header_stamp", default_value="true"),
        DeclareLaunchArgument("min_dt", default_value="0.0001"),

        # imu_angular_velocity_overwriter params
        DeclareLaunchArgument("zero_on_first_msg", default_value="true"),

        # imu_accel_lpf params
        DeclareLaunchArgument("time_constant_norm_sec", default_value="0.01"),
        DeclareLaunchArgument("time_constant_dir_sec", default_value="0.01"),

        Node(
            package="kuroko_imu",
            executable="imu_angular_velocity_overwriter",
            name="imu_angular_velocity_overwriter",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("intermediate_topic"),
                "use_header_stamp": LaunchConfiguration("use_header_stamp"),
                "min_dt": LaunchConfiguration("min_dt"),
                "zero_on_first_msg": LaunchConfiguration("zero_on_first_msg"),
            }],
        ),

        Node(
            package="kuroko_imu",
            executable="imu_accel_lpf",
            name="imu_accel_lpf",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("intermediate_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "use_header_stamp": LaunchConfiguration("use_header_stamp"),
                "min_dt": LaunchConfiguration("min_dt"),
                "time_constant_norm_sec": LaunchConfiguration("time_constant_norm_sec"),
                "time_constant_dir_sec": LaunchConfiguration("time_constant_dir_sec"),
            }],
        ),
    ])
