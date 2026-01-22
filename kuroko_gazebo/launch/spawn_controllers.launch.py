# kuroko_gazebo/launch/spawn_controllers.launch.py

from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _controller_names_from_params(merged_params: dict) -> list[str]:
    """
    Extract controller names from:
      controller_manager:
        ros__parameters:
          <name>:
            type: ...
    Ignore keys that are not controller definitions (e.g., update_rate).
    """
    cm = merged_params.get("controller_manager", {})
    ros_params = cm.get("ros__parameters", {}) if isinstance(cm, dict) else {}
    names = []
    for k, v in ros_params.items():
        if isinstance(v, dict) and "type" in v:
            names.append(k)
    return names


def _setup(context, *args, **kwargs):
    pkg_share = Path(get_package_share_directory("kuroko_description"))
    config_dir = pkg_share / "config"

    # Always-loaded yamls
    controller_manager_yaml = config_dir / "controller_manager.yaml"
    joint_state_broadcaster_yaml = config_dir / "joint_state_broadcaster.yaml"

    # Select-one yaml (4 candidates)
    # NOTE: "joint_trajectory_pid_controller.yaml" content defines
    #       joint_trajectory_effort_controller (effort-based JTC).
    selected = LaunchConfiguration("controller").perform(context)

    candidates = {
        "joint_group_position_controller": config_dir / "joint_group_position_controller.yaml",
        "joint_group_position_pid_controller": config_dir / "joint_group_position_pid_controller.yaml",
        "joint_trajectory_controller": config_dir / "joint_trajectory_controller.yaml",
        "joint_trajectory_effort_controller": config_dir / "joint_trajectory_pid_controller.yaml",
    }

    if selected not in candidates:
        raise RuntimeError(
            f"Unknown controller='{selected}'. Choose one of: {list(candidates.keys())}"
        )

    selected_yaml = candidates[selected]

    # Load & merge (deep-merge enough for this structure)
    merged = {}
    for p in [controller_manager_yaml, joint_state_broadcaster_yaml, selected_yaml]:
        merged.update(_load_yaml(p))

    controller_names = _controller_names_from_params(merged)

    actions = []
    actions.append(
        LogInfo(
            msg=(
                f"[kuroko_gazebo] selected controller yaml: {selected_yaml.name} | "
                f"controllers to spawn: {controller_names}"
            )
        )
    )

    # Spawn every controller found in controller_manager.ros__parameters
    # controller_manager node name is usually "/controller_manager"
    for name in controller_names:
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                name=f"spawner_{name}",
                output="screen",
                arguments=[
                    name,
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller",
                default_value="joint_trajectory_controller",
                description=(
                    "Select one controller yaml to load in addition to controller_manager.yaml "
                    "and joint_state_broadcaster.yaml. "
                    "Choices: joint_group_position_controller | joint_group_position_pid_controller | "
                    "joint_trajectory_controller | joint_trajectory_effort_controller"
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
