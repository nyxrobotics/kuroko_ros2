from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kuroko_imu",
            executable="imu_drift_remover",
            name="imu_drift_remover",
            output="screen",
            parameters=[{
                "imu_topic": "/imu_data",
                "joint_topic": "/joint_states",
                "output_topic": "/imu_drift",

                "joint_vel_norm_thresh": 0.02,
                "gyro_norm_thresh": 0.03,
                "accel_diff_norm_thresh": 0.5,

                "stable_time_sec": 1.5,
                "bias_time_constant_sec": 10.0,

                "publish_debug": False,
                "print_internal_state": False,
            }]
        )
    ])
