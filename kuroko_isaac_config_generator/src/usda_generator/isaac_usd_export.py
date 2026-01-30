import os
import subprocess
from pathlib import Path


def export_sdf_to_usda_with_isaac(sdf_path: str, usda_path: str, isaac_app: str = "isaacsim", headless: bool = True) -> None:
    """Export SDF to USDA via Isaac Sim/Kit.

    Notes:
      - Isaac Sim's built-in importers officially support URDF well; SDF import support can vary by version.
      - This function is implemented as an external process call so the ROS node can run without embedding Isaac.
      - You can swap the script invoked here to match your Isaac Lab / Isaac Sim version.
    """
    sdf_path = str(Path(sdf_path).resolve())
    usda_path = str(Path(usda_path).resolve())
    script = Path(__file__).with_name("isaac_usd_export_script.py").resolve()

    cmd = [isaac_app, "--no-window"] if headless else [isaac_app]
    cmd += ["--/app/quitAfter=1", "--exec", str(script), "--", "--sdf", sdf_path, "--out", usda_path]

    env = os.environ.copy()
    # Allow caller to pass Isaac paths via environment.
    subprocess.check_call(cmd, env=env)


# Minimal script template stored next to this module.
_SCRIPT = r"""import argparse
import sys

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True)
    parser.add_argument("--out", required=True)
    args, _ = parser.parse_known_args()

    # Isaac-side implementation is version-dependent.
    # Keep this as a single place to adapt as your Isaac Sim/Isaac Lab setup evolves.
    #
    # Typical approaches:
    # - Use an available SDF importer extension if present in your installation.
    # - Or, convert SDF->URDF/mesh set on the ROS side and use omni.isaac.urdf.
    #
    # For now we fail explicitly with actionable guidance.
    raise RuntimeError(
        "Isaac export script is a template. Implement SDF->USD export using your Isaac Sim version's importer. "
        "You already have the canonical SDF at: %s ; desired output: %s" % (args.sdf, args.out)
    )

if __name__ == "__main__":
    main()
"""


def ensure_isaac_script_exists(path: Path) -> None:
    if not path.exists():
        path.write_text(_SCRIPT)
