#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _launch_setup(context: LaunchContext, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    world = LaunchConfiguration('world').perform(context)

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    xacro_file = str(FindPackageShare('kuroko_description').find('kuroko_description') + '/xacro/kuroko/kuroko.xacro')

    doc = xacro.process_file(
        xacro_file,
        mappings={
            # Simulation-friendly URDF: mimic disabled, closed loops enabled
            'gazebo': 'true',
            'robot_name': robot,
        },
    )
    robot_description = doc.toxml()

    # Publish TF from the same URDF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    # Spawn entity by passing the URDF via a ROS parameter (no temp files, no custom scripts)
    create = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', world,
            '-name', robot,
            '-param', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
        parameters=[{
            'robot_description': robot_description,
        }],
    )

    return [rsp, create]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='kuroko', description='Entity name in Gazebo.'),
        DeclareLaunchArgument('world', default_value='empty', description='Gazebo world name.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw (rad).'),
        OpaqueFunction(function=_launch_setup),
    ])
