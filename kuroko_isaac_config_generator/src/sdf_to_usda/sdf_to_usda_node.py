#!/usr/bin/env python3
"""ROS 2 node that converts an existing SDF file to a USDA file via Isaac Sim.

This node intentionally does *only* SDF -> USDA to keep responsibilities clear.
Use the companion launch file `xacro_to_sdf.launch.py` to generate the SDF.

Required external dependency:
- Isaac Sim installation that provides a `python.sh` (or equivalent) launcher.
- A user-provided export script that imports SDF and writes USDA.

The node does not ship an Isaac export script because the API and paths differ
between Isaac Sim versions and install methods.
"""

import subprocess
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node


def _run_checked(cmd: list[str], *, cwd: Optional[str] = None) -> str:
    return subprocess.check_output(cmd, text=True, cwd=cwd)


class SdfToUsdaNode(Node):
    def __init__(self) -> None:
        super().__init__("sdf_to_usda")

        # Inputs / outputs
        self.declare_parameter("input_sdf", "/tmp/kuroko_usd_out/kuroko.sdf")
        self.declare_parameter("output_usda", "")  # if empty, derive from input_sdf

        # Isaac Sim invocation
        self.declare_parameter("isaac_python", "python.sh")
        self.declare_parameter("isaac_export_script", "")

        # Optional: working directory for Isaac execution
        self.declare_parameter("work_dir", "")

        self._run_once()

    def _run_once(self) -> None:
        input_sdf = Path(str(self.get_parameter("input_sdf").value)).expanduser().resolve()
        output_usda_param = str(self.get_parameter("output_usda").value)

        isaac_python = str(self.get_parameter("isaac_python").value)
        isaac_export_script = str(self.get_parameter("isaac_export_script").value)
        work_dir_param = str(self.get_parameter("work_dir").value)

        if not input_sdf.exists():
            raise FileNotFoundError(f"input_sdf not found: {input_sdf}")

        if not isaac_export_script:
            raise RuntimeError("isaac_export_script is empty. Provide a script that imports SDF and writes USDA.")

        script_path = Path(isaac_export_script).expanduser().resolve()
        if not script_path.exists():
            raise FileNotFoundError(f"isaac_export_script not found: {script_path}")

        if output_usda_param:
            output_usda = Path(output_usda_param).expanduser().resolve()
        else:
            output_usda = input_sdf.with_suffix(".usda")

        output_usda.parent.mkdir(parents=True, exist_ok=True)

        work_dir = Path(work_dir_param).expanduser().resolve() if work_dir_param else output_usda.parent

        cmd_isaac = [
            isaac_python,
            str(script_path),
            "--input_sdf",
            str(input_sdf),
            "--output_usda",
            str(output_usda),
        ]

        self.get_logger().info(f"input_sdf: {input_sdf}")
        self.get_logger().info(f"output_usda: {output_usda}")
        self.get_logger().info(f"isaac_python: {isaac_python}")
        self.get_logger().info(f"isaac_export_script: {script_path}")
        self.get_logger().info(f"work_dir: {work_dir}")
        self.get_logger().info(f"Running: {' '.join(cmd_isaac)}")

        _run_checked(cmd_isaac, cwd=str(work_dir))

        if not output_usda.exists():
            raise RuntimeError(f"Isaac export finished but output_usda was not created: {output_usda}")

        self.get_logger().info(f"Wrote USDA: {output_usda}")


def main() -> None:
    rclpy.init()
    node = SdfToUsdaNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
