#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Package name that provides the conversion script and config files.
    pkg_name = "kuroko_isaac_config_generator"

    # Launch arguments
    input_usda = LaunchConfiguration("input_usda")
    input_sdf = LaunchConfiguration("input_sdf")
    output_usda = LaunchConfiguration("output_usda")

    # Standard ROS 2 Humble style:
    # - Build paths using ament index substitutions (no runtime filesystem probing).
    # - Execute the installed script from lib/<pkg>/ (libexec-style install).
    #
    # Note: This assumes the script is installed as an executable (install(PROGRAMS ...) makes it +x).
    script_path = PathJoinSubstitution(
        [FindPackagePrefix(pkg_name), "lib", pkg_name, "usda_for_isaaclab.py"]
    )

    exclude_yaml_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "config", "exclude_from_articulation.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_usda",
                default_value="/tmp/kuroko_usd_out/kuroko.usda",
                description="Input USDA file path.",
            ),
            DeclareLaunchArgument(
                "input_sdf",
                default_value="/tmp/kuroko_usd_out/kuroko.resolved.sdf",
                description="Input resolved SDF file path.",
            ),
            DeclareLaunchArgument(
                "output_usda",
                default_value="/tmp/kuroko_usd_out/kuroko.isaaclab.usda",
                description="Output USDA file path.",
            ),
            ExecuteProcess(
                cmd=[
                    script_path,
                    "--input_usda",
                    input_usda,
                    "--input_sdf",
                    input_sdf,
                    "--output_usda",
                    output_usda,
                    "--exclude_yaml",
                    exclude_yaml_path,
                ],
                output="screen",
            ),
        ]
    )
