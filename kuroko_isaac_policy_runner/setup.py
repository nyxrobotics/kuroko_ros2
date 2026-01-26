from setuptools import setup
import os

package_name = "kuroko_isaac_policy_runner"


def _bundle_data_files():
    data_files = []
    src_root = os.path.join(os.path.dirname(__file__), "bundle")
    if not os.path.isdir(src_root):
        return data_files

    for root, _, files in os.walk(src_root):
        rel_dir = os.path.relpath(root, os.path.dirname(__file__))  # bundle/<policy_name>/...
        install_dir = os.path.join("share", package_name, rel_dir)
        paths = [os.path.join(root, f) for f in files]
        if paths:
            data_files.append((install_dir, paths))
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/isaac_policy_runner.launch.py"]),
        *_bundle_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Robot-specific wrapper package to launch isaac_policy_runtime with bundled policies.",
    license="BSD-3-Clause",
)
