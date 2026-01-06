#!/usr/bin/env python3
"""Launch Gazebo Sim + spawn Kuroko + load ros2_control controllers (gz_ros2_control).

- Uses gz-sim via ros_gz_sim.
- Spawns the robot from kuroko_description.
- Loads a JointTrajectoryController that outputs effort commands (position-servo style).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world_sdf = LaunchConfiguration("world_sdf")
    world_name = LaunchConfiguration("world_name")
    robot_name = LaunchConfiguration("robot_name")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")

    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_kuroko_sim_gz = FindPackageShare("kuroko_sim_gz")
    pkg_kuroko_description = FindPackageShare("kuroko_description")
    controllers_yaml = PathJoinSubstitution([pkg_kuroko_description, "config", "gz_trajectory_controller.yaml"])

    # 1) Launch gz-sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kuroko_sim_gz, "launch", "gz_sim.launch.py"])),
        launch_arguments={"world": world_sdf}.items(),
    )

    # 2) Spawn robot (robot_state_publisher + ros_gz_sim create)
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kuroko_sim_gz, "launch", "spawn_kuroko.launch.py"])),
        launch_arguments={
            "robot": robot_name,
            "world": world_name,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
        }.items(),
    )

    # 3) Load controllers using controller_manager spawner.
    #    NOTE: With gz_ros2_control, controller_manager is typically provided by the Gazebo system plugin.
    #    The default controller manager namespace is usually /controller_manager.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,

            # practically "no timeout"
            "--controller-manager-timeout", "86400",
            "--switch-timeout", "86400",
            "--service-call-timeout", "86400", 
        ],
    )

    traj_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,

            # practically "no timeout"
            "--controller-manager-timeout", "86400",
            "--switch-timeout", "86400",
            "--service-call-timeout", "86400", 
        ],
    )

    # Start spawners slightly later to give Gazebo time to create the controller_manager node.
    delayed_jsb = TimerAction(period=3.0, actions=[jsb_spawner])
    delayed_traj = TimerAction(period=4.0, actions=[traj_spawner])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_sdf",
                default_value="empty.sdf",
                description="World SDF filename (relative to kuroko_sim_gz/worlds).",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="empty",
                description="Gazebo world name (used by ros_gz_sim create -world).",
            ),
            DeclareLaunchArgument("robot_name", default_value="kuroko", description="Entity name in Gazebo."),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.4"),
            DeclareLaunchArgument("yaw", default_value="0.0", description="Yaw (rad)."),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            gz_sim,
            spawn,
            delayed_jsb,
            delayed_traj,
        ]
    )
