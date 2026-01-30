#!/usr/bin/env python3
import os
import sys
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Ensure local helper modules (installed next to this script) are importable.
_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))

from xacro_to_urdf import build_kuroko_urdf_for_spawn
from sdf_normalizer import rewrite_sdf_uris_to_absolute
from isaac_usd_export import export_sdf_to_usda_with_isaac, ensure_isaac_script_exists


class UsdaGenerator(Node):
    def __init__(self):
        super().__init__("usda_generator")

        self.declare_parameter("output_dir", "/tmp/kuroko_usd_out")
        self.declare_parameter("basename", "kuroko")
        self.declare_parameter("gazebo", True)
        self.declare_parameter("xacro_package", "kuroko_description")
        self.declare_parameter("xacro_relative_path", "xacro/kuroko/kuroko.xacro")
        self.declare_parameter("controller_yaml_relative_path", "config/ros2_control/joint_group_position_controller.yaml")
        self.declare_parameter("write_intermediate", True)

        self.declare_parameter("isaac_app", "isaacsim")
        self.declare_parameter("headless_isaac", True)
        self.declare_parameter("run_isaac_export", False)  # default off (environment-dependent)

        self._run()

    def _run(self) -> None:
        output_dir = Path(self.get_parameter("output_dir").value).expanduser().resolve()
        basename = str(self.get_parameter("basename").value)
        gazebo = bool(self.get_parameter("gazebo").value)
        xacro_pkg = str(self.get_parameter("xacro_package").value)
        xacro_rel = str(self.get_parameter("xacro_relative_path").value)
        ctrl_rel = str(self.get_parameter("controller_yaml_relative_path").value)
        write_intermediate = bool(self.get_parameter("write_intermediate").value)

        isaac_app = str(self.get_parameter("isaac_app").value)
        headless_isaac = bool(self.get_parameter("headless_isaac").value)
        run_isaac_export = bool(self.get_parameter("run_isaac_export").value)

        output_dir.mkdir(parents=True, exist_ok=True)

        xacro_path = Path(get_package_share_directory(xacro_pkg)) / xacro_rel
        controller_yaml = Path(get_package_share_directory(xacro_pkg)) / ctrl_rel

        if not xacro_path.exists():
            raise FileNotFoundError(f"xacro not found: {xacro_path}")
        if not controller_yaml.exists():
            raise FileNotFoundError(f"controller_yaml not found: {controller_yaml}")

        self.get_logger().info(f"xacro: {xacro_path}")
        self.get_logger().info(f"controller_yaml: {controller_yaml}")

        urdf = build_kuroko_urdf_for_spawn(str(xacro_path), str(controller_yaml), gazebo)

        urdf_path = output_dir / f"{basename}.urdf"
        sdf_path = output_dir / f"{basename}.sdf"
        usda_path = output_dir / f"{basename}.usda"

        if write_intermediate:
            urdf_path.write_text(urdf)
            self.get_logger().info(f"Wrote URDF: {urdf_path}")
        else:
            urdf_path.write_text(urdf)  # helper expects a file

        # Convert URDF -> SDF using packaged helper executable
        helper = _THIS_DIR / "urdf_to_sdf"
        if not helper.exists():
            # In install space: lib/<pkg>/urdf_to_sdf
            helper = _THIS_DIR.parent / "urdf_to_sdf"

        if not helper.exists():
            raise FileNotFoundError("urdf_to_sdf helper not found. Build the package first (colcon build).")

        sdf_xml = subprocess.check_output([str(helper), str(urdf_path)], text=True)

        # Rewrite URIs inside SDF so Isaac-side tools can resolve meshes reliably
        sdf_xml = rewrite_sdf_uris_to_absolute(sdf_xml, base_dir=str(output_dir))
        sdf_path.write_text(sdf_xml)
        self.get_logger().info(f"Wrote SDF: {sdf_path}")

        ensure_isaac_script_exists(_THIS_DIR / "isaac_usd_export_script.py")
        if run_isaac_export:
            self.get_logger().info("Running Isaac export (SDF->USDA)...")
            export_sdf_to_usda_with_isaac(str(sdf_path), str(usda_path), isaac_app=isaac_app, headless=headless_isaac)
            self.get_logger().info(f"Wrote USDA: {usda_path}")
        else:
            self.get_logger().warn(
                "Isaac export is disabled by default. Set run_isaac_export:=true after adapting "
                "isaac_usd_export_script.py for your Isaac Sim installation."
            )


def main():
    rclpy.init()
    node = UsdaGenerator()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
