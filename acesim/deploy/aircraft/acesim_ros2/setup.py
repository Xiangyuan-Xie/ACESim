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
        (os.path.join("share", package_name, "config"), ["config/px4_sim_config.yaml"]),
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
            "simulation_clock_zmq_bridge = acesim_ros2.simulation_clock_zmq_bridge:main",
            "arm_state_zmq_bridge = acesim_ros2.arm_state_zmq_bridge:main",
        ],
    },
)
