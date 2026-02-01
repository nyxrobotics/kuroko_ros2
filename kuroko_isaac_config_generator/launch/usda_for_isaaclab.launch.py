from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def _setup(context, *args, **kwargs):
    input_usda = LaunchConfiguration("input_usda").perform(context)
    input_sdf = LaunchConfiguration("input_sdf").perform(context)
    output_usda = LaunchConfiguration("output_usda").perform(context)

    prefix = Path(FindPackagePrefix("kuroko_isaac_config_generator").perform(context))
    share = Path(FindPackageShare("kuroko_isaac_config_generator").perform(context))

    # Prefer libexec-installed script. Fall back to share/scripts.
    script_candidates = [
        prefix / "lib" / "kuroko_isaac_config_generator" / "usda_for_isaaclab.py",
        share / "scripts" / "usda_for_isaaclab.py",
    ]
    script = None
    for c in script_candidates:
        if c.exists():
            script = c
            break
    if script is None:
        raise RuntimeError(f"usda_for_isaaclab.py not found. Checked: {script_candidates}")

    exclude_yaml = share / "config" / "exclude_from_articulation.yaml"

    cmd = [
        "python3",
        str(script),
        "--input_usda", input_usda,
        "--input_sdf", input_sdf,
        "--output_usda", output_usda,
        "--exclude_yaml", str(exclude_yaml),
    ]

    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_usda", default_value="/tmp/kuroko_usd_out/kuroko.usda"),
        DeclareLaunchArgument("input_sdf", default_value="/tmp/kuroko_usd_out/kuroko.resolved.sdf"),
        DeclareLaunchArgument("output_usda", default_value="/tmp/kuroko_usd_out/kuroko.isaaclab.usda"),
        OpaqueFunction(function=_setup),
    ])
