from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

_STRING_OPTIONS = {
    "px4_repo": "--px4-repo",
    "jobs": "--jobs",
    "output": "--output",
    "json_output": "--json-output",
    "raw_output_dir": "--raw-output-dir",
    "log_dir": "--log-dir",
    "bridge_mode": "--bridge-mode",
    "profile_cycles": "--profile-cycles",
    "port_base": "--port-base",
    "xrce_port_base": "--xrce-port-base",
    "ros_domain_base": "--ros-domain-base",
    "px4_instance_base": "--px4-instance-base",
    "startup_timeout_s": "--startup-timeout-s",
    "takeoff_timeout_s": "--takeoff-timeout-s",
    "arm_motion_duration_s": "--arm-motion-duration-s",
    "post_arm_motion_settle_s": "--post-arm-motion-settle-s",
}
_PATH_OPTION_NAMES = {"px4_repo", "output", "json_output", "raw_output_dir", "log_dir"}
_BOOL_OPTIONS = {
    "verbose_process_logs": "--verbose-process-logs",
    "strict_exit_code": "--strict-exit-code",
}


def _default_config_path() -> Path:
    return Path(get_package_share_directory("acesim_ros2")) / "config" / "x500_arm2x_benchmark.yaml"


def _launch_value(context: object, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _load_config(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"x500_arm2x benchmark config must be a mapping: {path}")
    return raw


def _truthy(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _append_option(command: list[str], flag: str, value: object) -> None:
    text = str(value).strip()
    if text:
        command.extend([flag, text])


def _option_value(name: str, value: object) -> object:
    if name in _PATH_OPTION_NAMES and str(value).strip().startswith("~"):
        return str(Path(str(value).strip()).expanduser())
    return value


def _launch_setup(context: object) -> list[ExecuteProcess]:
    config_override = _launch_value(context, "config")
    config_path = Path(config_override).expanduser() if config_override else _default_config_path()
    config = _load_config(config_path)

    command = ["ros2", "run", "acesim_ros2", "x500_arm2x_benchmark"]
    case_override = _launch_value(context, "case")
    cases = [case_override] if case_override else list(config.get("cases") or [])
    for case in cases:
        _append_option(command, "--case", case)

    for name, flag in _STRING_OPTIONS.items():
        value: object = _launch_value(context, name)
        if not value:
            value = config.get(name, "")
        value = _option_value(name, value)
        _append_option(command, flag, value)

    for name, flag in _BOOL_OPTIONS.items():
        value = _launch_value(context, name)
        if value:
            enabled = _truthy(value)
        else:
            enabled = _truthy(config.get(name, False))
        if enabled:
            command.append(flag)

    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="",
                description="Benchmark YAML config path; empty uses the package default config.",
            ),
            DeclareLaunchArgument("px4_repo", default_value="", description="PX4-Autopilot repository path override."),
            DeclareLaunchArgument("case", default_value="", description="Run only one benchmark case."),
            DeclareLaunchArgument("jobs", default_value="", description="Parallel jobs: 1, 2, ..., or auto."),
            DeclareLaunchArgument("output", default_value="", description="Summary figure output path."),
            DeclareLaunchArgument("json_output", default_value="", description="Compact JSON summary output path."),
            DeclareLaunchArgument("raw_output_dir", default_value="", description="Raw per-case output directory."),
            DeclareLaunchArgument("log_dir", default_value="", description="Per-process log directory."),
            DeclareLaunchArgument("bridge_mode", default_value="", description="Bridge mode: linux or wsl."),
            DeclareLaunchArgument("profile_cycles", default_value="", description="Velocity profile cycle count."),
            DeclareLaunchArgument("verbose_process_logs", default_value="", description="Stream child process logs."),
            DeclareLaunchArgument(
                "strict_exit_code", default_value="", description="Return nonzero when any case fails."
            ),
            DeclareLaunchArgument(
                "port_base", default_value="", description="Base ZMQ port for isolated benchmark workers."
            ),
            DeclareLaunchArgument("xrce_port_base", default_value="", description="Base Micro XRCE-DDS Agent port."),
            DeclareLaunchArgument("ros_domain_base", default_value="", description="Base ROS_DOMAIN_ID."),
            DeclareLaunchArgument("px4_instance_base", default_value="", description="Base PX4 instance id."),
            DeclareLaunchArgument("startup_timeout_s", default_value="", description="PX4 startup timeout in seconds."),
            DeclareLaunchArgument("takeoff_timeout_s", default_value="", description="Takeoff timeout in seconds."),
            DeclareLaunchArgument(
                "arm_motion_duration_s", default_value="", description="Arm motion duration in seconds."
            ),
            DeclareLaunchArgument(
                "post_arm_motion_settle_s",
                default_value="",
                description="AM hold settling time after arm motion.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
