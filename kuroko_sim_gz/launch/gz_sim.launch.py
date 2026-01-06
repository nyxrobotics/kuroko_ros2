#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration('world')

    pkg_kuroko_sim_gz = FindPackageShare('kuroko_sim_gz')
    world_path = PathJoinSubstitution([pkg_kuroko_sim_gz, 'worlds', world])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            # ros_gz_sim expects a single string passed to `gz sim`.
            'gz_args': world_path,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='World SDF file (relative to kuroko_sim_gz/worlds).',
        ),
        gz_sim,
    ])
