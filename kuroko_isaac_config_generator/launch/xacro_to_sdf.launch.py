#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    out_dir = LaunchConfiguration("out_dir").perform(context)
    basename = LaunchConfiguration("basename").perform(context)

    xacro_pkg = LaunchConfiguration("xacro_package").perform(context)
    xacro_rel = LaunchConfiguration("xacro_relative_path").perform(context)

    ctrl_pkg = LaunchConfiguration("controller_yaml_package").perform(context)
    ctrl_rel = LaunchConfiguration("controller_yaml_relative_path").perform(context)

    gazebo_arg = LaunchConfiguration("gazebo").perform(context)

    xacro_share = FindPackageShare(xacro_pkg).perform(context)
    ctrl_share = FindPackageShare(ctrl_pkg).perform(context)

    xacro_path = str(Path(xacro_share) / xacro_rel)
    controller_yaml = str(Path(ctrl_share) / ctrl_rel)

    urdf_path = str(Path(out_dir) / f"{basename}.urdf")
    sdf_path  = str(Path(out_dir) / f"{basename}.sdf")

    generate = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            (
                f"set -euo pipefail; "
                f"mkdir -p '{out_dir}' && "
                f"xacro '{xacro_path}' gazebo:='{gazebo_arg}' controller_yaml:='{controller_yaml}' "
                f"> '{urdf_path}' && "
                f"gz sdf -p '{urdf_path}' > '{sdf_path}'"
            ),
        ],
        output="screen",
    )

    return [generate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("out_dir", default_value="/tmp/kuroko_usd_out"),
        DeclareLaunchArgument("basename", default_value="kuroko"),
        DeclareLaunchArgument("gazebo", default_value="true"),

        DeclareLaunchArgument("xacro_package", default_value="kuroko_description"),
        DeclareLaunchArgument("xacro_relative_path", default_value="xacro/kuroko/kuroko.xacro"),

        DeclareLaunchArgument("controller_yaml_package", default_value="kuroko_description"),
        DeclareLaunchArgument(
            "controller_yaml_relative_path",
            default_value="config/ros2_control/joint_group_position_controller.yaml",
        ),

        OpaqueFunction(function=_setup),
    ])
