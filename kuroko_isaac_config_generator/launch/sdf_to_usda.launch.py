#!/usr/bin/env python3
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    input_sdf = LaunchConfiguration("input_sdf").perform(context)
    output_usda = LaunchConfiguration("output_usda").perform(context)

    # Ensure output directory exists
    out_dir = str(Path(output_usda).expanduser().resolve().parent)

    convert = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            f"mkdir -p '{out_dir}' && sdf2usd '{input_sdf}' '{output_usda}'",
        ],
        output="screen",
    )
    return [convert]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_sdf", default_value="/tmp/kuroko_usd_out/kuroko.sdf"),
        DeclareLaunchArgument("output_usda", default_value="/tmp/kuroko_usd_out/kuroko.usda"),
        OpaqueFunction(function=_setup),
    ])
