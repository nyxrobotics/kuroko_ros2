from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _setup_processes(context, *args, **kwargs):
    out_dir = LaunchConfiguration("out_dir").perform(context)
    basename = LaunchConfiguration("basename").perform(context)

    kuroko_share = FindPackageShare("kuroko_description").perform(context)
    xacro_path = str(Path(kuroko_share) / "xacro/kuroko/kuroko.xacro")
    controller_yaml = str(Path(kuroko_share) / "config/ros2_control/joint_group_position_controller.yaml")

    urdf_path = str(Path(out_dir) / f"{basename}.urdf")
    sdf_path = str(Path(out_dir) / f"{basename}.sdf")

    write_urdf = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            (
                f"mkdir -p '{out_dir}' && "
                f"xacro '{xacro_path}' gazebo:=true controller_yaml:='{controller_yaml}' "
                f"> '{urdf_path}'"
            )
        ],
        output="screen",
    )

    write_sdf = ExecuteProcess(
        cmd=["bash", "-lc", f"gz sdf -p '{urdf_path}' > '{sdf_path}'"],
        output="screen",
    )

    return [write_urdf, write_sdf]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("out_dir", default_value="/tmp/kuroko_usd_out"),
            DeclareLaunchArgument("basename", default_value="kuroko"),
            OpaqueFunction(function=_setup_processes),
        ]
    )
