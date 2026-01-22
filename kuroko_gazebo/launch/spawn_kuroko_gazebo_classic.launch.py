from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # --- Launch args ---
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="joint_trajectory_controller",
        description=(
            "Which ros2_control controller config to use. "
            "Examples: joint_trajectory_controller | joint_group_position_controller | "
            "joint_group_position_pid_controller | joint_trajectory_effort_controller"
        ),
    )

    # Optional override; if empty, it will be inferred from controller.
    command_interface_arg = DeclareLaunchArgument(
        "command_interface",
        default_value="",
        description="Override command_interface passed to xacro (position/effort). Empty = auto.",
    )

    robot_z_arg = DeclareLaunchArgument(
        "robot_z", default_value="0.35", description="Spawn height for the robot (z)."
    )

    # --- Infer command_interface from controller (simple rule) ---
    controller = LaunchConfiguration("controller")
    command_interface_override = LaunchConfiguration("command_interface")

    # If you choose the effort JTC, use effort, otherwise position.
    # (You can extend this mapping later if you add velocity controllers.)
    controller = LaunchConfiguration("controller")
    command_interface_override = LaunchConfiguration("command_interface")

    # If command_interface arg is set (non-empty), use it.
    # Otherwise: effort only when controller == joint_trajectory_effort_controller, else position.
    inferred_command_interface = PythonExpression([
        "'", command_interface_override, "' if '", command_interface_override, "' != '' "
        "else ('effort' if '", controller, "' == 'joint_trajectory_effort_controller' else 'position')"
    ])

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("kuroko_description"), "xacro", "kuroko", "kuroko.xacro"]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " gazebo:=true",
                " gz_sim:=false",
                " command_interface:=",
                inferred_command_interface,
            ]
        )
    }

    # --- Start Gazebo Classic ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            # empty.world usually already has a ground plane + sun.
            # Still, we'll spawn a ground_plane explicitly below to match your request.
            "verbose": "false",
        }.items(),
    )

    # --- Spawn ground plane at z=0 ---
    # Uses Gazebo model database name "ground_plane"
    spawn_ground = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ground_plane",
        output="screen",
        arguments=[
            "-entity",
            "ground_plane",
            "-database",
            "ground_plane",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
    )

    # --- Spawn robot at z=0.35 ---
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_kuroko",
        output="screen",
        arguments=[
            "-entity",
            "kuroko",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            LaunchConfiguration("robot_z"),
        ],
    )

    # --- robot_state_publisher ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            controller_arg,
            command_interface_arg,
            robot_z_arg,
            gazebo_launch,
            rsp,
            spawn_robot,
        ]
    )
