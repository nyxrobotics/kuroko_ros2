from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    use_header_stamp = LaunchConfiguration('use_header_stamp')
    zero_effort = LaunchConfiguration('zero_effort')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/joint_states',
            description='Input JointState topic to subscribe.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/joint_states_vel',
            description='Output JointState topic to publish.',
        ),
        DeclareLaunchArgument(
            'use_header_stamp',
            default_value='true',
            description='If true, dt uses msg.header.stamp; if false, uses reception time (node clock).',
        ),
        DeclareLaunchArgument(
            'zero_effort',
            default_value='false',
            description='If true, effort[] is filled with zeros; if false, original effort is kept when possible.',
        ),
        Node(
            package='kuroko_joint_states',
            executable='joint_states_velocity_overwriter',
            name='joint_states_velocity_overwriter',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'use_header_stamp': use_header_stamp,
                'zero_effort': zero_effort,
            }],
        ),
    ])
