from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_ros_share = FindPackageShare("gazebo_ros")
    kuroko_description_share = FindPackageShare("kuroko_description")
    kuroko_gazebo_share = FindPackageShare("kuroko_gazebo")

    world_path = PathJoinSubstitution([kuroko_gazebo_share, "worlds", "default.world"])
    xacro_path = PathJoinSubstitution([kuroko_description_share, "xacro", "kuroko", "kuroko.xacro"])
    controllers_yaml = PathJoinSubstitution(
        [kuroko_description_share, "config", "ros2_control", "joint_trajectory_controller.yaml"]
    )

    # robot_description
    # - spawn用: gazebo:=true でgazebo専用のパッシブジョイント/リンク追加
    # - controller用: gazebo:=false でジョイント追加をスキップ（xacro側の実装前提）
    robot_description_gazebo_true = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path, " ", "gazebo:=true"]),
        value_type=str,
    )

    robot_description_gazebo_false = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path, " ", "gazebo:=false"]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world_path}.items(),
    )

    # Publish robot_description(topic) for spawning (gazebo:=true)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_gazebo_true},
                    {"use_sim_time": use_sim_time}],
    )

    # Spawn entity from robot_description topic
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "kuroko"],
    )

    # ros2_control controller_manager node (gazebo:=false)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_gazebo_false},
            controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Delay controller spawning a bit
    spawn_controllers = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            gazebo,
            robot_state_publisher,
            spawn_entity,
            ros2_control_node,
            spawn_controllers,
        ]
    )
