from pathlib import Path

from setuptools import find_packages, setup

this_directory = Path(__file__).parent
long_description = ""
readme = this_directory / "README.md"
if readme.exists():
    long_description = readme.read_text(encoding="utf-8")

setup(
    name="acesim",
    version="0.1.0",
    description="PX4 + MuJoCo simulation toolkit split from ACETele.",
    long_description=long_description,
    long_description_content_type="text/markdown" if long_description else None,
    author="Xiangyuan Xie",
    author_email="dragonboat_xxy@163.com",
    python_requires=">=3.9",
    packages=find_packages(where="."),
    install_requires=[
        "mujoco",
        "numpy",
        "scipy",
        "pymavlink",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    keywords="python px4 mujoco simulation",
)
