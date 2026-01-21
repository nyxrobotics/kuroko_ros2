from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare, FindExecutable


def generate_launch_description() -> LaunchDescription:
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    robot_name = LaunchConfiguration('robot_name')
    controller = LaunchConfiguration('controller')
    use_small_ring = LaunchConfiguration('use_small_ring')
    spawn_enemy = LaunchConfiguration('spawn_enemy')
    enemy_name = LaunchConfiguration('enemy_name')
    wait_after_spawn = LaunchConfiguration('wait_after_spawn')

    kuroko_desc_share = FindPackageShare('kuroko_description')
    gazebo_share = FindPackageShare('gazebo_ros')

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
    enemy_xacro = PathJoinSubstitution([kuroko_desc_share, 'xacro', 'kuroko', 'enemy.xacro'])
    ring_small_xacro = PathJoinSubstitution([
        kuroko_desc_share,
        'xacro',
        'roboone_ring',
        'small_ring.xacro',
    ])
    ring_large_xacro = PathJoinSubstitution([
        kuroko_desc_share,
        'xacro',
        'roboone_ring',
        'large_ring.xacro',
    ])

    robot_description = Command([xacro_exec, ' ', robot_xacro, ' gazebo:=false'])
    robot_description_gazebo = Command([xacro_exec, ' ', robot_xacro, ' gazebo:=true'])
    enemy_description_gazebo = Command([xacro_exec, ' ', enemy_xacro, ' gazebo:=true'])
    ring_small_urdf = Command([xacro_exec, ' ', ring_small_xacro])
    ring_large_urdf = Command([xacro_exec, ' ', ring_large_xacro])

    spawn_small_ring = Node(
        package='kuroko_gazebo',
        executable='spawn_entity_from_param.py',
        name='spawn_ring_small',
        output='screen',
        parameters=[{'ring_urdf': ring_small_urdf}],
        arguments=['--name', 'ring_model', '--param', 'ring_urdf', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        condition=IfCondition(use_small_ring),
    )

    spawn_large_ring = Node(
        package='kuroko_gazebo',
        executable='spawn_entity_from_param.py',
        name='spawn_ring_large',
        output='screen',
        parameters=[{'ring_urdf': ring_large_urdf}],
        arguments=['--name', 'ring_model', '--param', 'ring_urdf', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        condition=UnlessCondition(use_small_ring),
    )

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
            '-x', '-0.9',
            '-y', '0.0',
            '-z', '0.9',
            '-J', 'ankle_l_roll', '-0.17453292519943295',
            '-J', 'ankle_l_yaw', '0.0',
            '-J', 'ankle_r_roll', '0.17453292519943295',
            '-J', 'ankle_r_yaw', '0.0',
            '-J', 'chest', '0.0',
            '-J', 'elbow_l_front', '2.007128639793479',
            '-J', 'elbow_l_rear', '-2.007128639793479',
            '-J', 'elbow_r_front', '-2.007128639793479',
            '-J', 'elbow_r_rear', '2.007128639793479',
            '-J', 'hip_l_pitch', '0.0',
            '-J', 'hip_l_roll', '-0.17453292519943295',
            '-J', 'hip_r_pitch', '0.0',
            '-J', 'hip_r_roll', '-0.17453292519943295',
            '-J', 'shin_l_active', '0.0',
            '-J', 'shin_r_active', '0.0',
            '-J', 'shoulder_l_pitch', '-1.1344640137963142',
            '-J', 'shoulder_l_roll', '-1.4311699866353502',
            '-J', 'shoulder_r_pitch', '1.1344640137963142',
            '-J', 'shoulder_r_roll', '1.4311699866353502',
            '-J', 'thigh_l_active', '0.0',
            '-J', 'thigh_r_active', '0.0',
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

    spawn_enemy_node = Node(
        package='kuroko_gazebo',
        executable='spawn_entity_from_param.py',
        name='spawn_enemy',
        output='screen',
        parameters=[{'enemy_description': enemy_description_gazebo}],
        arguments=[
            '--name', enemy_name,
            '--param', 'enemy_description',
            '--namespace', enemy_name,
            '-x', '0.9',
            '-y', '0.0',
            '-z', '0.9',
            '-Y', '3.1416',
            '-J', 'ankle_l_roll', '-0.17453292519943295',
            '-J', 'ankle_l_yaw', '0.0',
            '-J', 'ankle_r_roll', '0.17453292519943295',
            '-J', 'ankle_r_yaw', '0.0',
            '-J', 'chest', '0.0',
            '-J', 'elbow_l_front', '2.007128639793479',
            '-J', 'elbow_l_rear', '-2.007128639793479',
            '-J', 'elbow_r_front', '-2.007128639793479',
            '-J', 'elbow_r_rear', '2.007128639793479',
            '-J', 'hip_l_pitch', '0.0',
            '-J', 'hip_l_roll', '-0.17453292519943295',
            '-J', 'hip_r_pitch', '0.0',
            '-J', 'hip_r_roll', '-0.17453292519943295',
            '-J', 'shin_l_active', '0.0',
            '-J', 'shin_r_active', '0.0',
            '-J', 'shoulder_l_pitch', '-1.1344640137963142',
            '-J', 'shoulder_l_roll', '-1.4311699866353502',
            '-J', 'shoulder_r_pitch', '1.1344640137963142',
            '-J', 'shoulder_r_roll', '1.4311699866353502',
            '-J', 'thigh_l_active', '0.0',
            '-J', 'thigh_r_active', '0.0',
        ],
        condition=IfCondition(spawn_enemy),
    )

    jsb_yaml = PathJoinSubstitution([FindPackageShare('kuroko_gazebo'), 'config', 'joint_state_controller.yaml'])
    traj_yaml = PathJoinSubstitution([FindPackageShare('kuroko_gazebo'), 'config', 'joint_trajectory_controller.yaml'])
    group_pos_yaml = PathJoinSubstitution([FindPackageShare('kuroko_gazebo'), 'config', 'joint_position_group_controller.yaml'])
    common_spawner_args = ['-c', [TextSubstitution(text='/'), robot_name, TextSubstitution(text='/controller_manager')]]

    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', jsb_yaml, *common_spawner_args],
        output='screen',
    )
    is_trajectory = PythonExpression(["'", controller, "' == 'trajectory'"])

    spawn_traj = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--param-file', traj_yaml, *common_spawner_args],
        output='screen',
        condition=IfCondition(is_trajectory),
    )
    spawn_pos = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller', '--param-file', group_pos_yaml, *common_spawner_args],
        output='screen',
        condition=UnlessCondition(is_trajectory),
    )

    expected_models_expr = [robot_name, TextSubstitution(text=' ring_model ground_plane')]
    unpause = Node(
        package='kuroko_gazebo',
        executable='unpause_physics.py',
        name='unpause_physics',
        output='screen',
        parameters=[
            {'expected_models': expected_models_expr},
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
            DeclareLaunchArgument('robot_name', default_value='kuroko'),
            DeclareLaunchArgument('controller', default_value='group_position'),
            DeclareLaunchArgument('use_small_ring', default_value='false'),
            DeclareLaunchArgument('spawn_enemy', default_value='true'),
            DeclareLaunchArgument('enemy_name', default_value='enemy'),
            DeclareLaunchArgument('wait_after_spawn', default_value='1.0'),
            DeclareLaunchArgument('gazebo_model_path', default_value=TextSubstitution(text='')),
            set_gazebo_model_path,
            gazebo,
            spawn_small_ring,
            spawn_large_ring,
            spawn_robot,
            robot_state_publisher,
            spawn_enemy_node,
            spawn_jsb,
            spawn_traj,
            spawn_pos,
            unpause,
        ]
    )
