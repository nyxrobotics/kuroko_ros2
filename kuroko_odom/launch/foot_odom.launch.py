# launch/foot_odom.launch.py
#
# Sample launch for kuroko_odom:
# - foot_odom_node publishes /body_odom and /foot_odom
# - odom_tf_node republishes TF from selected odom topic
#
# Usage:
#   ros2 launch kuroko_odom foot_odom_sample.launch.py
#
# Optional:
#   ros2 launch kuroko_odom foot_odom_sample.launch.py odom_topic:=/foot_odom publish_inverse:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")

    joint_states_topic = LaunchConfiguration("joint_states_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    initialpose_topic = LaunchConfiguration("initialpose_topic")

    body_odom_topic = LaunchConfiguration("body_odom_topic")
    foot_odom_topic = LaunchConfiguration("foot_odom_topic")

    imu_timeout_sec = LaunchConfiguration("imu_timeout_sec")

    # Which odometry topic to use for TF publishing
    odom_topic = LaunchConfiguration("odom_topic")

    # odom_tf_node option:
    # If publish_inverse==false:  odom -> base_link
    # If publish_inverse==true:   base_link -> odom   (same transform, inverted direction)
    publish_inverse = LaunchConfiguration("publish_inverse")

    right_leg_joint_names = LaunchConfiguration("right_leg_joint_names")
    left_leg_joint_names = LaunchConfiguration("left_leg_joint_names")

    right_footprint_xy = LaunchConfiguration("right_footprint_xy")
    left_footprint_xy = LaunchConfiguration("left_footprint_xy")

    stance_up_acc_threshold = LaunchConfiguration("stance_up_acc_threshold")
    stance_down_acc_threshold = LaunchConfiguration("stance_down_acc_threshold")

    cov_trans_min = LaunchConfiguration("cov_trans_min")
    cov_trans_max = LaunchConfiguration("cov_trans_max")

    foot_tilt_threshold_rad = LaunchConfiguration("foot_tilt_threshold_rad")
    cov_rot_min = LaunchConfiguration("cov_rot_min")
    cov_rot_max = LaunchConfiguration("cov_rot_max")

    foot_odom_node = Node(
        package="kuroko_odom",
        executable="foot_odom_node",
        name="foot_odom",
        output="screen",
        parameters=[{
            "odom_frame_id": odom_frame_id,
            "base_frame_id": base_frame_id,

            "joint_states_topic": joint_states_topic,
            "imu_topic": imu_topic,
            "initialpose_topic": initialpose_topic,

            "body_odom_topic": body_odom_topic,
            "foot_odom_topic": foot_odom_topic,

            "imu_timeout_sec": imu_timeout_sec,

            # Naming must match your robot's /joint_states
            "right_leg_joint_names": right_leg_joint_names,
            "left_leg_joint_names": left_leg_joint_names,

            # Flattened XY pairs in the foot-end frame (x0,y0,x1,y1,...)
            "right_footprint_xy": right_footprint_xy,
            "left_footprint_xy": left_footprint_xy,

            "stance_up_acc_threshold": stance_up_acc_threshold,
            "stance_down_acc_threshold": stance_down_acc_threshold,

            "cov_trans_min": cov_trans_min,
            "cov_trans_max": cov_trans_max,

            "foot_tilt_threshold_rad": foot_tilt_threshold_rad,
            "cov_rot_min": cov_rot_min,
            "cov_rot_max": cov_rot_max,
        }],
    )

    odom_tf_node = Node(
        package="kuroko_odom",
        executable="odom_tf_node",
        name="odom_tf",
        output="screen",
        parameters=[{
            "odom_topic": odom_topic,
            "odom_frame_id": odom_frame_id,
            "base_frame_id": base_frame_id,
            "publish_inverse": publish_inverse,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("base_frame_id", default_value="body_link"),

        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("imu_topic", default_value="/kuroko/sensors/imu/data"),
        DeclareLaunchArgument("initialpose_topic", default_value="/initialpose"),

        DeclareLaunchArgument("body_odom_topic", default_value="/body_odom"),
        DeclareLaunchArgument("foot_odom_topic", default_value="/foot_odom"),

        DeclareLaunchArgument("imu_timeout_sec", default_value="0.1"),

        # TF source
        DeclareLaunchArgument("odom_topic", default_value="/body_odom"),
        DeclareLaunchArgument("publish_inverse", default_value="false"),

        # Default joint names (kuroko_kinematics naming)
        DeclareLaunchArgument(
            "right_leg_joint_names",
            default_value='["hip_r_roll","hip_r_pitch","thigh_r_active","shin_r_active","ankle_r_roll","ankle_r_yaw"]'
        ),
        DeclareLaunchArgument(
            "left_leg_joint_names",
            default_value='["hip_l_roll","hip_l_pitch","thigh_l_active","shin_l_active","ankle_l_roll","ankle_l_yaw"]'
        ),

        # Default footprint vertices: [[0.06, 0.0375],[0.06,-0.0375],[-0.06,-0.0375],[-0.06,0.0375]]
        DeclareLaunchArgument(
            "right_footprint_xy",
            default_value="[0.06,0.0375, 0.06,-0.0375, -0.06,-0.0375, -0.06,0.0375]"
        ),
        DeclareLaunchArgument(
            "left_footprint_xy",
            default_value="[0.06,0.0375, 0.06,-0.0375, -0.06,-0.0375, -0.06,0.0375]"
        ),

        DeclareLaunchArgument("stance_up_acc_threshold", default_value="9.81"),
        DeclareLaunchArgument("stance_down_acc_threshold", default_value="0.0"),

        DeclareLaunchArgument("cov_trans_min", default_value="0.1"),
        DeclareLaunchArgument("cov_trans_max", default_value="1e6"),

        DeclareLaunchArgument("foot_tilt_threshold_rad", default_value="0.2"),
        DeclareLaunchArgument("cov_rot_min", default_value="0.1"),
        DeclareLaunchArgument("cov_rot_max", default_value="1e6"),

        foot_odom_node,
        odom_tf_node,
    ])
