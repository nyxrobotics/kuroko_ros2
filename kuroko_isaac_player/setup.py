from setuptools import setup
import os
from glob import glob

package_name = "kuroko_isaac_player"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # trained data (top-level)
        (os.path.join("share", package_name, "trained_data", "kuroko_walk"), glob("trained_data/kuroko_walk/*.pt")),
        (os.path.join("share", package_name, "trained_data", "kuroko_walk"), glob("trained_data/kuroko_walk/*.onnx")),
        # trained data params
        (os.path.join("share", package_name, "trained_data", "kuroko_walk", "params"), glob("trained_data/kuroko_walk/params/*.yaml")),
        # exported policies (Isaac Lab)
        (os.path.join("share", package_name, "trained_data", "kuroko_walk", "exported"), glob("trained_data/kuroko_walk/exported/*.pt")),
        (os.path.join("share", package_name, "trained_data", "kuroko_walk", "exported"), glob("trained_data/kuroko_walk/exported/*.onnx")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Run Isaac Lab trained policy in ROS2 (Kuroko).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_player = kuroko_isaac_player.policy_player_node:main",
        ],
    },
)
