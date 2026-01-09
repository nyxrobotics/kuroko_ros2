#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    model_name = LaunchConfiguration("model_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # clock
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # IMU
            [
                TextSubstitution(text="/"),
                model_name,
                TextSubstitution(
                    text="/sensors/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU"
                ),
            ],

            # Magnetometer
            [
                TextSubstitution(text="/"),
                model_name,
                TextSubstitution(
                    text="/sensors/imu/mag@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer"
                ),
            ],

            # Camera image
            [
                TextSubstitution(text="/"),
                model_name,
                TextSubstitution(
                    text="/sensors/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
                ),
            ],

            # Camera info
            [
                TextSubstitution(text="/"),
                model_name,
                TextSubstitution(
                    text="/sensors/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
                ),
            ],
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_name",
                default_value="kuroko",
                description="Model name prefix used in Gazebo topic paths",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            bridge,
        ]
    )
