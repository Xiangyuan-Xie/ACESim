import importlib.util
import os
from pathlib import Path
from typing import Optional

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _detect_acesim_root() -> Path:
    spec = importlib.util.find_spec("acesim")
    if spec is not None:
        locations = spec.submodule_search_locations
        if locations:
            return Path(next(iter(locations))).resolve()
        origin = spec.origin
        if origin:
            return Path(origin).resolve().parent
    env = os.environ.get("ACESIM_ROOT")
    if env:
        return Path(env).resolve()
    raise RuntimeError("Failed to locate ACESim repository; set ACESIM_ROOT or pass px4_repo")


def _load_px4_repo_path(override: Optional[str]) -> str:
    if isinstance(override, str) and override.strip():
        value = Path(override.strip())
        if not value.is_absolute():
            base = _detect_acesim_root()
            value = (base / value).resolve()
        return str(value)
    base = _detect_acesim_root()
    return str((base / "third_party" / "aircraft" / "PX4-Autopilot").resolve())


def _launch_setup(context):
    override = LaunchConfiguration("px4_repo").perform(context)
    px4_repo_path = _load_px4_repo_path(override)
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
    )
    px4_sitl = ExecuteProcess(
        cmd=["bash", "-lc", "export PX4_SIM_MODEL=none_iris && make px4_sitl none"],
        cwd=px4_repo_path,
        output="screen",
    )
    arm_command_joint_state_pub = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--rate",
            "250",
            "/arm/command",
            "sensor_msgs/msg/JointState",
            (
                "{name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'], "
                "position: [-1.57, 3.14, 0.0, 0.0, 0.0], "
                "velocity: [0.0, 0.0, 0.0, 0.0, 0.0], "
                "effort: [0.0, 0.0, 0.0, 0.0, 0.0]}"
            ),
        ],
        # output="screen",
    )
    arm_state_joint_state_pub = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--rate",
            "250",
            "/arm/state",
            "sensor_msgs/msg/JointState",
            (
                "{name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'], "
                "position: [-1.57, 3.14, 0.0, 0.0, 0.0], "
                "velocity: [0.0, 0.0, 0.0, 0.0, 0.0], "
                "effort: [0.0, 0.0, 0.0, 0.0, 0.0]}"
            ),
        ],
        # output="screen",
    )
    return [
        micro_xrce_agent,
        px4_sitl,
        arm_command_joint_state_pub,
        arm_state_joint_state_pub,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_repo",
                default_value="",
                description="PX4-Autopilot repository path; if empty, auto-detect from ACESim",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
