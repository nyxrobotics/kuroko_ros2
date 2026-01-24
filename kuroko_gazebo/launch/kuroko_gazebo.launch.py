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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def urdf_to_oneline_without_comments(xml_string: str) -> str:
    """
    Remove XML comments and serialize URDF as a single line.
    This is useful to avoid issues when a backend tries to pass robot_description via CLI arguments.
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

    kuroko_description_share = get_package_share_directory("kuroko_description")
    xacro_file = os.path.join(kuroko_description_share, "xacro", "kuroko", "kuroko.xacro")

    # Policy:
    # - robot_description is ALWAYS gazebo:=false (control / canonical model)
    # - spawn uses gazebo:=true via a separate topic "robot_description_gazebo"
    urdf_control_raw = xacro_to_urdf_xml(xacro_file, {"gazebo": "false"})
    urdf_spawn_raw = xacro_to_urdf_xml(xacro_file, {"gazebo": "true"})

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

    # Spawn-only publisher: remap its "robot_description" topic to "robot_description_gazebo"
    node_spawn_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_description_gazebo_publisher",
        output="screen",
        parameters=[{**params_spawn, "publish_frequency": 0.0}],
        remappings=[("robot_description", "robot_description_gazebo")],
    )

    # Spawn position/orientation
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

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        node_robot_state_publisher,
        node_spawn_description_publisher,
        spawn_entity,
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": os.path.join(get_package_share_directory("kuroko_gazebo"), "worlds", "default.world")
        }.items(),
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
            # Spawn pose (default z=0.34)
            DeclareLaunchArgument("spawn_x", default_value="0.0", description="Spawn position X [m]"),
            DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn position Y [m]"),
            DeclareLaunchArgument("spawn_z", default_value="0.34", description="Spawn position Z [m]"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0", description="Spawn roll [rad]"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0", description="Spawn pitch [rad]"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Spawn yaw [rad]"),
            # Debug options (default OFF)
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
            gazebo,
            OpaqueFunction(function=_create_runtime_actions),
        ]
    )
