from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare, FindExecutable


def _spawner(controller_name: str, cm_ns: LaunchConfiguration, param_files, condition=None):
    args = [controller_name]
    for pf in param_files:
        args += ['--param-file', pf]
    args += ['-c', cm_ns]
    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=args,
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    robot_name = LaunchConfiguration('robot_name')
    controller_mode = LaunchConfiguration('controller')
    wait_after_spawn = LaunchConfiguration('wait_after_spawn')

    kuroko_desc_share = FindPackageShare('kuroko_description')
    gazebo_share = FindPackageShare('gazebo_ros')
    kuroko_gazebo_share = FindPackageShare('kuroko_gazebo')

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            PathJoinSubstitution([kuroko_desc_share, '..']),
            TextSubstitution(text=':'),
            LaunchConfiguration('gazebo_model_path'),
        ],
    )

    world = PathJoinSubstitution([kuroko_desc_share, 'worlds', 'default.world'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_share, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'pause': 'true',
            'verbose': debug,
            'headless': headless,
        }.items(),
    )

    xacro_exec = FindExecutable(name='xacro')
    robot_xacro = PathJoinSubstitution([kuroko_desc_share, 'xacro', 'kuroko', 'kuroko.xacro'])

    robot_description = Command([xacro_exec, ' ', robot_xacro, ' gazebo:=false'])
    robot_description_gazebo = Command([xacro_exec, ' ', robot_xacro, ' gazebo:=true'])

    spawn_robot = Node(
        package='kuroko_gazebo',
        executable='spawn_entity_from_param.py',
        name='spawn_robot',
        output='screen',
        parameters=[{'robot_description_gazebo': robot_description_gazebo}],
        arguments=[
            '--name', robot_name,
            '--param', 'robot_description_gazebo',
            '--namespace', robot_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.32',
        ],
    )

    robot_state_publisher = GroupAction(
        actions=[
            PushRosNamespace(robot_name),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_description': robot_description},
                    {'publish_frequency': 200.0},
                ],
            ),
        ]
    )

    cm_ns = [TextSubstitution(text='/'), robot_name, TextSubstitution(text='/controller_manager')]

    cm_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'controller_manager.yaml'])
    jsb_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_state_controller.yaml'])
    pos_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_position_controller.yaml'])
    group_pos_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_position_group_controller.yaml'])
    pos_pid_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_position_pid_controller.yaml'])
    traj_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_trajectory_controller.yaml'])
    traj_pid_yaml = PathJoinSubstitution([kuroko_gazebo_share, 'config', 'joint_trajectory_pid_controller.yaml'])

    param_base = [cm_yaml]

    spawn_jsb = _spawner('joint_state_broadcaster', cm_ns, [*param_base, jsb_yaml])

        is_individual_position = PythonExpression(["'", controller_mode, "' == 'individual_position'"])
    is_group_position = PythonExpression(["'", controller_mode, "' == 'group_position' or '", controller_mode, "' == 'position'"])
    is_position_pid = PythonExpression(["'", controller_mode, "' == 'position_pid'"])
    is_trajectory = PythonExpression(["'", controller_mode, "' == 'trajectory'"])
    is_trajectory_pid = PythonExpression(["'", controller_mode, "' == 'trajectory_pid'"])

            spawn_group_pos = _spawner('joint_group_position_controller', cm_ns, [*param_base, group_pos_yaml], condition=IfCondition(is_group_position))
    # Individual position controllers (one per joint)
    _individual_names = ['chest_position', 'shoulder_r_pitch_position', 'shoulder_r_roll_position', 'elbow_r_front_position', 'elbow_r_rear_position', 'shoulder_l_pitch_position', 'shoulder_l_roll_position', 'elbow_l_front_position', 'elbow_l_rear_position', 'hip_r_roll_position', 'hip_r_pitch_position', 'thigh_r_active_position', 'shin_r_active_position', 'ankle_r_roll_position', 'ankle_r_yaw_position', 'hip_l_roll_position', 'hip_l_pitch_position', 'thigh_l_active_position', 'shin_l_active_position', 'ankle_l_roll_position', 'ankle_l_yaw_position']
    spawn_individual_pos = [
        _spawner(name, cm_ns, [*param_base, pos_yaml], condition=IfCondition(is_individual_position))
        for name in _individual_names
    ]
    spawn_pos_pid = _spawner('joint_position_pid_controller', cm_ns, [*param_base, pos_pid_yaml], condition=IfCondition(is_position_pid))
    spawn_traj = _spawner('joint_trajectory_controller', cm_ns, [*param_base, traj_yaml], condition=IfCondition(is_trajectory))
    spawn_traj_pid = _spawner('joint_trajectory_effort_controller', cm_ns, [*param_base, traj_pid_yaml], condition=IfCondition(is_trajectory_pid))

    unpause = Node(
        package='kuroko_gazebo',
        executable='unpause_physics.py',
        name='unpause_physics',
        output='screen',
        parameters=[
            {'expected_models': [robot_name, TextSubstitution(text=' ground_plane')]},
            {'wait_after_spawn': wait_after_spawn},
        ],
        condition=UnlessCondition(paused),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('paused', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('headless', default_value='false'),
            DeclareLaunchArgument('debug', default_value='false'),
            DeclareLaunchArgument('gazebo_model_path', default_value=''),
            DeclareLaunchArgument('robot_name', default_value='roboone'),
            DeclareLaunchArgument(
                'controller',
                default_value='group_position',
                description=(
                    'Controller mode: ' \
                    'group_position | position(=group_position) | individual_position | position_pid | trajectory | trajectory_pid'
                ),
            ),
            DeclareLaunchArgument('wait_after_spawn', default_value='1.0'),
            set_gazebo_model_path,
            gazebo,
            spawn_robot,
            robot_state_publisher,
            spawn_jsb,
            spawn_group_pos,
            *spawn_individual_pos,
            spawn_pos_pid,
            spawn_traj,
            spawn_traj_pid,
            unpause,
        ]
    )
