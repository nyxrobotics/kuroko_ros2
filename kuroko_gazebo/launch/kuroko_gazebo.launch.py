# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from xml.dom import minidom

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro
import yaml


def urdf_to_oneline_without_comments(xml_string: str) -> str:
    """
    Remove XML comments and serialize URDF as a single line.
    """
    dom = minidom.parseString(xml_string)

    def remove_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == child.COMMENT_NODE:
                node.removeChild(child)
            elif child.hasChildNodes():
                remove_comments(child)

    remove_comments(dom)
    return dom.toxml()


def xacro_to_urdf_xml(xacro_path: str, mappings: dict) -> str:
    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc.toxml()


def _as_bool(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")


def get_controllers_from_yaml(controller_yaml_path: str) -> dict[str, str]:
    """
    Extract controllers defined in a ros2_control YAML.

    Expected structure:
      controller_manager:
        ros__parameters:
          <controller_name>:
            type: <plugin_class>

    Returns:
      dict[name] = type_string
    """
    if not controller_yaml_path:
        raise RuntimeError("controller_yaml_path is empty")

    if not os.path.exists(controller_yaml_path):
        raise RuntimeError(f"controller_yaml does not exist: {controller_yaml_path}")

    with open(controller_yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}

    params = data.get("controller_manager", {}).get("ros__parameters", {})

    controllers: dict[str, str] = {}
    for name, cfg in params.items():
        if isinstance(cfg, dict) and "type" in cfg:
            controllers[name] = str(cfg["type"])

    if not controllers:
        raise RuntimeError(
            f"No controller with 'type' found under controller_manager/ros__parameters in: {controller_yaml_path}"
        )

    return controllers


def _create_runtime_actions(context, *args, **kwargs):
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    entity_name = LaunchConfiguration("entity_name").perform(context)

    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_roll = LaunchConfiguration("spawn_roll").perform(context)
    spawn_pitch = LaunchConfiguration("spawn_pitch").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)

    print_urdf = _as_bool(LaunchConfiguration("print_urdf").perform(context))
    dump_urdf = _as_bool(LaunchConfiguration("dump_urdf").perform(context))

    controller_yaml = LaunchConfiguration("controller_yaml").perform(context)
    controllers = get_controllers_from_yaml(controller_yaml)

    # Print controllers list for debugging
    print("\n========== [kuroko] Controllers found in YAML ==========")
    print(f"[kuroko] controller_yaml: {controller_yaml}")
    for name in sorted(controllers.keys()):
        print(f"[kuroko] - {name}: type={controllers[name]}")
    print("========== [kuroko] end controllers list ==========\n")

    # Decide activation order:
    # - Usually joint_state_broadcaster first (if present), then everything else
    names_sorted = sorted(controllers.keys())
    if "joint_state_broadcaster" in controllers:
        activation_order = ["joint_state_broadcaster"] + [n for n in names_sorted if n != "joint_state_broadcaster"]
    else:
        activation_order = names_sorted

    print("[kuroko] Controllers activation order:")
    for n in activation_order:
        print(f"[kuroko] - {n}")

    kuroko_description_share = get_package_share_directory("kuroko_description")
    xacro_file = os.path.join(kuroko_description_share, "xacro", "kuroko", "kuroko.xacro")

    # Pass the same controller_yaml into both URDF variants (gazebo:=false/true)
    urdf_control_raw = xacro_to_urdf_xml(
        xacro_file,
        {"gazebo": "false", "controller_yaml": controller_yaml},
    )
    urdf_spawn_raw = xacro_to_urdf_xml(
        xacro_file,
        {"gazebo": "true", "controller_yaml": controller_yaml},
    )

    urdf_control = urdf_to_oneline_without_comments(urdf_control_raw)
    urdf_spawn = urdf_to_oneline_without_comments(urdf_spawn_raw)

    if print_urdf:
        print(
            "\n========== [kuroko] robot_description (gazebo:=false) cleaned URDF (one-line, no comments) =========="
        )
        print(urdf_control)
        print("========== [kuroko] end robot_description ==========\n")

        print(
            "\n========== [kuroko] robot_description_gazebo (gazebo:=true) cleaned URDF (one-line, no comments) =========="
        )
        print(urdf_spawn)
        print("========== [kuroko] end robot_description_gazebo ==========\n")

    if dump_urdf:
        try:
            with open("/tmp/kuroko_robot_description.urdf", "w") as f:
                f.write(urdf_control + "\n")
            with open("/tmp/kuroko_robot_description_gazebo.urdf", "w") as f:
                f.write(urdf_spawn + "\n")
            print(
                "[kuroko] Wrote cleaned URDFs to /tmp/kuroko_robot_description.urdf and /tmp/kuroko_robot_description_gazebo.urdf"
            )
        except Exception as e:
            print(f"[kuroko] Failed to write URDF files to /tmp: {e}")

    params_control = {"robot_description": urdf_control, "use_sim_time": use_sim_time}
    params_spawn = {"robot_description": urdf_spawn, "use_sim_time": use_sim_time}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[params_control],
    )

    # Spawn-only publisher: remap its robot_description topic to robot_description_gazebo
    node_spawn_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_description_gazebo_publisher",
        output="screen",
        parameters=[{**params_spawn, "publish_frequency": 0.0}],
        remappings=[("robot_description", "robot_description_gazebo")],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description_gazebo",
            "-entity",
            entity_name,
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-R",
            spawn_roll,
            "-P",
            spawn_pitch,
            "-Y",
            spawn_yaw,
        ],
    )

    # Activate ALL controllers found in YAML, sequentially.
    # Using 'set -e' ensures the process fails fast on any error (bad yaml/type/etc.).
    cmds = [f"ros2 control load_controller --set-state active {name}" for name in activation_order]
    activate_all_controllers = ExecuteProcess(
        cmd=["bash", "-lc", "set -e; " + "; ".join(cmds)],
        output="screen",
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[activate_all_controllers],
            )
        ),
        node_robot_state_publisher,
        node_spawn_description_publisher,
        spawn_entity,
    ]


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(get_package_share_directory("kuroko_gazebo"), "worlds", "default.world")
        }.items(),
    )

    kuroko_description_share = get_package_share_directory("kuroko_description")
    default_controller_yaml = os.path.join(
        kuroko_description_share,
        "config",
        "ros2_control",
        "joint_group_position_controller.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="kuroko",
                description="Spawned entity name in Gazebo",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn position X [m]"),
            DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn position Y [m]"),
            DeclareLaunchArgument("spawn_z", default_value="0.34", description="Spawn position Z [m]"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0", description="Spawn roll [rad]"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0", description="Spawn pitch [rad]"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Spawn yaw [rad]"),
            DeclareLaunchArgument(
                "print_urdf",
                default_value="false",
                description="Print cleaned URDFs to stdout (default: false)",
            ),
            DeclareLaunchArgument(
                "dump_urdf",
                default_value="false",
                description="Write cleaned URDFs to /tmp (default: false)",
            ),
            DeclareLaunchArgument(
                "controller_yaml",
                default_value=default_controller_yaml,
                description="ros2_control controller YAML passed into xacro (and used to activate all controllers in it)",
            ),
            gazebo,
            OpaqueFunction(function=_create_runtime_actions),
        ]
    )
