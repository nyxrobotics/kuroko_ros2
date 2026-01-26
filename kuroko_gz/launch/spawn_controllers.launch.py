#!/usr/bin/env python3
"""Spawn ros2_control controllers from a YAML file via controller_manager/spawner.

The YAML is expected to define controllers under:

  controller_manager:
    ros__parameters:
      <controller_name>:
        type: <plugin_class>

Any entry with a `type` field is treated as a controller to be spawned.

Activation order:
- joint_state_broadcaster first (if present)
- then the remaining controllers (sorted by name)

The default timeouts are intentionally huge to tolerate Gazebo Sim starting paused.
"""

from __future__ import annotations

import os
from typing import Dict, List

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(v: str) -> bool:
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def _get_controllers_from_yaml(yaml_path: str) -> Dict[str, str]:
    if not yaml_path:
        raise RuntimeError("controllers_yaml is empty")

    if not os.path.exists(yaml_path):
        raise RuntimeError(f"controllers_yaml does not exist: {yaml_path}")

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}

    params = data.get("controller_manager", {}).get("ros__parameters", {})

    controllers: Dict[str, str] = {}
    for name, cfg in params.items():
        if isinstance(cfg, dict) and "type" in cfg:
            controllers[str(name)] = str(cfg["type"])

    if not controllers:
        raise RuntimeError(
            "No controllers found. Expected entries with 'type' under "
            "controller_manager/ros__parameters in: " + yaml_path
        )

    return controllers


def _launch_setup(context: LaunchContext, *args, **kwargs):
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    controller_manager = LaunchConfiguration("controller_manager").perform(context)

    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))

    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout").perform(context)
    switch_timeout = LaunchConfiguration("switch_timeout").perform(context)
    service_call_timeout = LaunchConfiguration("service_call_timeout").perform(context)

    base_delay_sec = float(LaunchConfiguration("base_delay_sec").perform(context))
    delay_step_sec = float(LaunchConfiguration("delay_step_sec").perform(context))

    controllers = _get_controllers_from_yaml(controllers_yaml)

    names_sorted: List[str] = sorted(controllers.keys())
    if "joint_state_broadcaster" in controllers:
        activation_order = ["joint_state_broadcaster"] + [n for n in names_sorted if n != "joint_state_broadcaster"]
    else:
        activation_order = names_sorted

    print("\n========== [kuroko_gz] Controllers from YAML ==========")
    print(f"[kuroko_gz] controllers_yaml: {controllers_yaml}")
    for n in activation_order:
        print(f"[kuroko_gz] - {n}: type={controllers.get(n)}")
    print("========== [kuroko_gz] End controllers list ==========")

    actions = []
    for i, name in enumerate(activation_order):
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                name,
                "--controller-manager",
                controller_manager,
                "--param-file",
                controllers_yaml,
                "--controller-manager-timeout",
                controller_manager_timeout,
                "--switch-timeout",
                switch_timeout,
                "--service-call-timeout",
                service_call_timeout,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )
        actions.append(TimerAction(period=base_delay_sec + delay_step_sec * i, actions=[spawner]))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_yaml",
                description="YAML file for controller_manager (ros2_control). All controllers in it will be spawned.",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="controller_manager node name/namespace to contact.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="86400"),
            DeclareLaunchArgument("switch_timeout", default_value="86400"),
            DeclareLaunchArgument("service_call_timeout", default_value="86400"),
            DeclareLaunchArgument("base_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("delay_step_sec", default_value="1.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
