# kuroko_gazebo/launch/bringup_gazebo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="joint_trajectory_controller",
        description="Controller to use (same arg forwarded to both xacro and controller spawner)",
    )

    command_interface_arg = DeclareLaunchArgument(
        "command_interface",
        default_value="",
        description="Optional override for xacro command_interface",
    )

    robot_z_arg = DeclareLaunchArgument(
        "robot_z", default_value="0.35", description="Spawn height of kuroko"
    )

    controller_yaml_files_arg = DeclareLaunchArgument(
        "controller_yaml_files",
        default_value=(
            "$(find kuroko_description)/config/ros2_control/controller_manager.yaml;"
            "$(find kuroko_description)/config/ros2_control/joint_state_broadcaster.yaml;"
            "$(find kuroko_description)/config/ros2_control/joint_trajectory_controller.yaml"
        ),
        description="Semicolon-separated YAML list for ros2_control plugin.",
    )

    debug_control_arg = DeclareLaunchArgument(
        "debug_control",
        default_value="false",
        description="Enable xacro debug messages for ros2_control setup.",
    )

    # --- spawn robot + gazebo ---
    spawn_kuroko = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kuroko_gazebo"),
                    "launch",
                    "spawn_kuroko_gazebo_classic.launch.py",
                ]
            )
        ),
        launch_arguments={
            "controller": LaunchConfiguration("controller"),
            "command_interface": LaunchConfiguration("command_interface"),
            "controller_yaml_files": LaunchConfiguration("controller_yaml_files"),
            "debug_control": LaunchConfiguration("debug_control"),
            "robot_z": LaunchConfiguration("robot_z"),
        }.items(),
    )

    # --- spawn controllers ---
    # 少し待ってから spawner を投げる（gazebo_ros2_control の起動待ち）
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kuroko_gazebo"),
                    "launch",
                    "spawn_controllers.launch.py",
                ]
            )
        ),
        launch_arguments={
            "controller": LaunchConfiguration("controller"),
        }.items(),
    )

    delayed_spawn_controllers = TimerAction(
        period=3.0,
        actions=[spawn_controllers],
    )

    return LaunchDescription(
        [
            controller_arg,
            command_interface_arg,
            controller_yaml_files_arg,
            debug_control_arg,
            robot_z_arg,
            spawn_kuroko,
            delayed_spawn_controllers,
        ]
    )
