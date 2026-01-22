#!/usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _make_spawner(controller_name: str, param_files: list[str]) -> Node:
    # controller_manager spawner:
    #  - loads controller type/params from param files
    #  - then spawns the controller
    args = [controller_name, "--controller-manager", "/controller_manager"]
    for f in param_files:
        args += ["--param-file", f]

    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=args,
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("kuroko_gazebo")
    cfg = os.path.join(pkg, "config")

    # Your 7 yaml files (as-is)
    controller_manager_yaml = os.path.join(cfg, "controller_manager.yaml")
    joint_state_yaml = os.path.join(cfg, "joint_state_controller.yaml")
    joint_traj_yaml = os.path.join(cfg, "joint_trajectory_controller.yaml")
    joint_traj_pid_yaml = os.path.join(cfg, "joint_trajectory_pid_controller.yaml")
    joint_group_pos_yaml = os.path.join(cfg, "joint_position_group_controller.yaml")
    joint_individual_pos_yaml = os.path.join(cfg, "joint_position_controller.yaml")
    joint_pos_pid_yaml = os.path.join(cfg, "joint_position_pid_controller.yaml")

    mode = LaunchConfiguration("controller").perform(context)
    state_controller_name = LaunchConfiguration("state_controller").perform(context)

    # Common: keep controller_manager params always applied (update_rate, etc)
    common_param_files = [controller_manager_yaml]

    nodes = []

    # 1) State controller (spawn ONLY the one you request)
    #    - default is "joint_state_broadcaster"
    #    - if you want ROS1-like name, pass state_controller:=joint_state_controller
    nodes.append(
        _make_spawner(
            state_controller_name,
            common_param_files + [joint_state_yaml, joint_group_pos_yaml],
        )
    )

    # 2) Main controller: spawn exactly one group depending on mode
    if mode == "trajectory":
        # YAML defines controller name "trajectory_controller" (per your joint_trajectory_controller.yaml)
        nodes.append(
            _make_spawner(
                "trajectory_controller",
                common_param_files + [joint_traj_yaml],
            )
        )

    elif mode == "trajectory_pid":
        # YAML defines controller name "joint_trajectory_effort_controller"
        nodes.append(
            _make_spawner(
                "joint_trajectory_effort_controller",
                common_param_files + [joint_traj_pid_yaml],
            )
        )

    elif mode == "group_position":
        # YAML defines controller name "joint_group_position_controller"
        nodes.append(
            _make_spawner(
                "joint_group_position_controller",
                common_param_files + [joint_group_pos_yaml],
            )
        )

    elif mode == "position_pid":
        # YAML defines controller name "joint_position_pid_controller"
        nodes.append(
            _make_spawner(
                "joint_position_pid_controller",
                common_param_files + [joint_pos_pid_yaml],
            )
        )

    elif mode == "individual_position":
        # YAML defines many controllers like "*_position"
        # Spawn ALL of them (no extra controllers).
        individual_names = [
            "chest_position",
            "shoulder_r_pitch_position",
            "shoulder_r_roll_position",
            "elbow_r_front_position",
            "elbow_r_rear_position",
            "shoulder_l_pitch_position",
            "shoulder_l_roll_position",
            "elbow_l_front_position",
            "elbow_l_rear_position",
            "hip_r_roll_position",
            "hip_r_pitch_position",
            "thigh_r_active_position",
            "shin_r_active_position",
            "ankle_r_roll_position",
            "ankle_r_yaw_position",
            "hip_l_roll_position",
            "hip_l_pitch_position",
            "thigh_l_active_position",
            "shin_l_active_position",
            "ankle_l_roll_position",
            "ankle_l_yaw_position",
        ]
        for name in individual_names:
            nodes.append(
                _make_spawner(
                    name,
                    common_param_files + [joint_individual_pos_yaml],
                )
            )
    else:
        raise RuntimeError(
            f"Unknown controller mode: {mode}. "
            "Use: trajectory | trajectory_pid | group_position | position_pid | individual_position"
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                default_value="group_position",
                description="trajectory | trajectory_pid | group_position | position_pid | individual_position",
            ),
            DeclareLaunchArgument(
                "state_controller",
                default_value="joint_state_broadcaster",
                description="joint_state_broadcaster (ROS2) or joint_state_controller (legacy name)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
