#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _spawner(controller_name, controller_manager_ns, param_files, condition=None):
    args = [controller_name, '--controller-manager']
    args += controller_manager_ns
    for f in param_files:
        args += ['--param-file', f]
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=args,
        condition=condition,
    )


def _setup(context, *args, **kwargs):
    controller_mode = LaunchConfiguration('controller')
    robot_name = LaunchConfiguration('robot_name')

    kuroko_gazebo_share = get_package_share_directory('kuroko_gazebo')

    cm_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'controller_manager.yaml'])
    jsb_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_state_controller.yaml'])
    pos_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_position_controller.yaml'])
    group_pos_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_position_group_controller.yaml'])
    pos_pid_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_position_pid_controller.yaml'])
    traj_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_trajectory_controller.yaml'])
    traj_pid_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_trajectory_pid_controller.yaml'])

    cm_ns = [TextSubstitution(text='/'), robot_name, TextSubstitution(text='/controller_manager')]

    # Always start joint_state_broadcaster (ROS2)
    spawn_jsb = _spawner('joint_state_broadcaster', cm_ns, [cm_yaml, jsb_yaml])

    is_group_position = PythonExpression(["'", controller_mode, "' == 'group_position' or '", controller_mode, "' == 'position'"])
    is_individual_position = PythonExpression(["'", controller_mode, "' == 'individual_position'"])
    is_position_pid = PythonExpression(["'", controller_mode, "' == 'position_pid'"])
    is_traj = PythonExpression(["'", controller_mode, "' == 'trajectory'"])
    is_traj_pid = PythonExpression(["'", controller_mode, "' == 'trajectory_pid'"])

    spawn_group_pos = _spawner(
        'joint_group_position_controller',
        cm_ns,
        [cm_yaml, group_pos_yaml],
        condition=IfCondition(is_group_position),
    )

    # Individual position controllers (one per joint)
    individual_names = [
        'chest_position',
        'shoulder_r_pitch_position',
        'shoulder_r_roll_position',
        'elbow_r_front_position',
        'elbow_r_rear_position',
        'shoulder_l_pitch_position',
        'shoulder_l_roll_position',
        'elbow_l_front_position',
        'elbow_l_rear_position',
        'hip_r_roll_position',
        'hip_r_pitch_position',
        'thigh_r_active_position',
        'shin_r_active_position',
        'ankle_r_roll_position',
        'ankle_r_yaw_position',
        'hip_l_roll_position',
        'hip_l_pitch_position',
        'thigh_l_active_position',
        'shin_l_active_position',
        'ankle_l_roll_position',
        'ankle_l_yaw_position',
    ]
    spawn_individual_pos = [
        _spawner(name, cm_ns, [cm_yaml, pos_yaml], condition=IfCondition(is_individual_position))
        for name in individual_names
    ]

    spawn_pos_pid = _spawner(
        'joint_position_pid_controller',
        cm_ns,
        [cm_yaml, pos_pid_yaml],
        condition=IfCondition(is_position_pid),
    )

    spawn_traj = _spawner(
        'trajectory_controller',
        cm_ns,
        [cm_yaml, traj_yaml],
        condition=IfCondition(is_traj),
    )

    spawn_traj_pid = _spawner(
        'joint_trajectory_effort_controller',
        cm_ns,
        [cm_yaml, traj_pid_yaml],
        condition=IfCondition(is_traj_pid),
    )

    return [spawn_jsb, spawn_group_pos, spawn_pos_pid, spawn_traj, spawn_traj_pid] + spawn_individual_pos


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='kuroko',
            description='Namespace used for controller_manager (/<robot_name>/controller_manager)',
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='group_position',
            description='group_position | position | individual_position | position_pid | trajectory | trajectory_pid',
        ),
        OpaqueFunction(function=_setup),
    ])
