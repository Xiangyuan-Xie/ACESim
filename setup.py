from setuptools import find_namespace_packages, setup

setup(
    name="acesim",
    version="0.1.0",
    description="PX4 + MuJoCo simulation toolkit split from ACETele.",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    author="Xiangyuan Xie",
    author_email="dragonboat_xxy@163.com",
    keywords=["python", "px4", "mujoco", "simulation"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.9",
    install_requires=[
        "mujoco",
        "genesis-world",
        "numpy",
        "scipy",
        "tqdm",
        "pymavlink",
        "pyzmq",
    ],
    packages=find_namespace_packages(
        where=".",
        include=["acesim", "acesim.*"],
        exclude=[
            "acesim.third_party",
            "acesim.third_party.*",
        ],
    ),
    include_package_data=True,
)
