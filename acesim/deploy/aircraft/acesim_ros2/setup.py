import os
from glob import glob

from setuptools import find_packages, setup

package_name = "acesim_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "pyzmq",
    ],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "acesim_play = acesim_ros2.acesim_play:main",
            "acesim_play_headless = acesim_ros2.acesim_play_headless:main",
            "acesim_bridge = acesim_ros2.acesim_bridge:main",
            "x500_arm2x_benchmark = acesim_ros2.benchmark.x500_arm2x:main",
        ],
    },
)
