#!/usr/bin/env python3
import os
import re
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


def _rewrite_package_uris(sdf_xml: str) -> str:
    # Convert package://<pkg>/<path> to absolute file path using ament index
    def repl(m):
        pkg = m.group(1)
        rel = m.group(2) or ""
        try:
            share = Path(get_package_share_directory(pkg))
            return str((share / rel).resolve())
        except Exception:
            return m.group(0)

    return re.sub(r"package://([^/]+)/?(.*)", repl, sdf_xml)


class SdfToUsdaNode(Node):
    def __init__(self):
        super().__init__("sdf_to_usda")

        self.declare_parameter("input_sdf", "/tmp/kuroko_usd_out/kuroko.sdf")
        self.declare_parameter("output_usda", "/tmp/kuroko_usd_out/kuroko.usda")
        self.declare_parameter("isaac_python", "python.sh")
        self.declare_parameter("headless", True)

        self._run_once()

    def _run_once(self) -> None:
        input_sdf = Path(self.get_parameter("input_sdf").value).expanduser().resolve()
        output_usda = Path(self.get_parameter("output_usda").value).expanduser().resolve()
        isaac_python = str(self.get_parameter("isaac_python").value)
        headless = self.get_parameter("headless").value

        if not input_sdf.exists():
            raise FileNotFoundError(f"input_sdf not found: {input_sdf}")

        # Isaac export script bundled in this package
        pkg_share = Path(get_package_share_directory("kuroko_isaac_config_generator"))
        export_script = pkg_share / "scripts" / "sdf_to_usda_export.py"
        if not export_script.exists():
            raise FileNotFoundError(f"export script not found in package: {export_script}")

        # Prepare a temporary SDF with resolved package:// URIs (helps Isaac find meshes)
        sdf_xml = input_sdf.read_text()
        sdf_xml = _rewrite_package_uris(sdf_xml)
        tmp_sdf = output_usda.with_suffix(".resolved.sdf")
        tmp_sdf.write_text(sdf_xml)

        cmd = [isaac_python, str(export_script), "--input_sdf", str(tmp_sdf), "--output_usda", str(output_usda)]
        if str(headless).lower() in ("true", "1", "yes"):
            cmd += ["--headless"]

        self.get_logger().info(f"Running Isaac export: {' '.join(cmd)}")
        subprocess.check_call(cmd)
        self.get_logger().info(f"Wrote USDA: {output_usda}")


def main():
    rclpy.init()
    node = SdfToUsdaNode()
    # one-shot node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
