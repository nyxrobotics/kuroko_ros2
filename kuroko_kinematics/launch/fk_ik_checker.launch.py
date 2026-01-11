from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    side = LaunchConfiguration('side')
    base_frame = LaunchConfiguration('base_frame')
    tolerance_rad = LaunchConfiguration('tolerance_rad')
    warn_tolerance_rad = LaunchConfiguration('warn_tolerance_rad')
    log_every_n = LaunchConfiguration('log_every_n')

    checker = Node(
        package='kuroko_kinematics',
        executable='fk_ik_consistency_checker',
        name='fk_ik_consistency_checker',
        output='screen',
        parameters=[{
            'side': side,
            'base_frame': base_frame,
            'tolerance_rad': tolerance_rad,
            'warn_tolerance_rad': warn_tolerance_rad,
            'log_every_n': log_every_n,
            'publish_fk_pose': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [LaunchConfiguration('rviz_config')]],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 with a prepared config.'),
        DeclareLaunchArgument('rviz_config',
                              default_value='$(find-pkg-share kuroko_kinematics)/rviz/fk_ik_checker.rviz',
                              description='RViz2 config file.'),
        DeclareLaunchArgument('side', default_value='both',
                              description='right|left|both'),
        DeclareLaunchArgument('base_frame', default_value='body_link',
                              description='Frame id for published FK poses.'),
        DeclareLaunchArgument('tolerance_rad', default_value='0.001',
                              description='Pass threshold (rad).'),
        DeclareLaunchArgument('warn_tolerance_rad', default_value='0.005',
                              description='Warn threshold (rad).'),
        DeclareLaunchArgument('log_every_n', default_value='50',
                              description='Log OK every N messages.'),
        checker,
        rviz_node,
    ])
