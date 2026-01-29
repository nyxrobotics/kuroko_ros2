from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Drift remover
        Node(
            package="kuroko_imu",
            executable="imu_drift_remover",
            name="imu_drift_remover",
            output="screen",
            parameters=[{
                "imu_topic": "/kuroko/sensors/imu/data",
                "joint_topic": "/joint_states",

                # ↓ 次段(Madgwick)への出力
                "output_topic": "/kuroko/sensors/imu/data_drift",

                "joint_vel_norm_thresh": 0.02,
                "gyro_norm_thresh": 0.03,
                "accel_diff_norm_thresh": 0.5,

                "stable_time_sec": 1.5,
                "bias_time_constant_sec": 10.0,

                "publish_debug": False,
                "print_internal_state": False,
            }]
        ),

        # Madgwick filter
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            remappings=[
                # input
                ("imu/data_raw", "/kuroko/sensors/imu/data_drift"),
                # output
                ("imu/data", "/kuroko/sensors/imu/data_madgwick"),
            ],
            parameters=[{
                # kuroko では基本 mag 無し想定
                "use_mag": False,
            }],
        ),
    ])

