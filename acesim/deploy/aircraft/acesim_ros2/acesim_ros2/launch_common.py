from __future__ import annotations

import os
import re
import shlex
import sys
import tempfile
from pathlib import Path
from typing import Any, Literal, Mapping, Optional

if __package__ in (None, ""):
    package_parent = Path(__file__).resolve().parents[1]
    if str(package_parent) not in sys.path:
        sys.path.insert(0, str(package_parent))

import yaml
from acesim_ros2.bridge.config import load_bridge_configs
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch_ros.actions import Node

from acesim.config.config_loader import ConfigLoader
from acesim.sitl.process import build_graceful_shutdown_command
from acesim.sitl.px4_bootstrap import build_px4_env as core_build_px4_env
from acesim.sitl.px4_bootstrap import (
    build_px4_make_command,
)
from acesim.sitl.px4_bootstrap import load_px4_repo_path as core_load_px4_repo_path
from acesim.sitl.px4_bootstrap import resolve_px4_startup_env as core_resolve_px4_startup_env
from acesim.sitl.stack_plan import StackPlan
from acesim.sitl.stack_plan import build_ros_launch_stack_plan as core_build_ros_launch_stack_plan
from acesim.utils.px4_transport import PX4SensorParams

_TCP_ENDPOINT_PATTERN = re.compile(r"^tcp://(?P<host>[^:/]+):(?P<port>\d+)$")


def detect_acesim_root() -> Path:
    return Path(__file__).resolve().parents[4]


def load_px4_repo_path(override: Optional[str]) -> str:
    return str(core_load_px4_repo_path(override))


def resolve_px4_startup_env(config_loader: ConfigLoader | None = None) -> dict[str, str]:
    """Map the configured ACESim asset onto the PX4 airframe startup environment."""
    if config_loader is None:
        config_loader = ConfigLoader()
    return core_resolve_px4_startup_env(config_loader)


def build_px4_additional_env(config_loader: ConfigLoader | None = None) -> dict[str, str]:
    if config_loader is None:
        config_loader = ConfigLoader()
    return core_build_px4_env(config_loader, sensor_params_cls=PX4SensorParams)


def package_share_dir() -> Path | None:
    try:
        share_dir = Path(get_package_share_directory("acesim_ros2")).resolve()
    except Exception:
        return None
    if share_dir.exists():
        return share_dir
    return None


def bridge_config_path() -> str:
    share_dir = package_share_dir()
    if share_dir is not None:
        return str(share_dir / "config" / "bridges.yaml")
    return str(Path(__file__).resolve().parents[1] / "config" / "bridges.yaml")


def load_bridge_entries(config_file: str) -> list[dict[str, object]]:
    validated: list[dict[str, object]] = []
    for bridge in load_bridge_configs(config_file):
        endpoint = bridge.transport.endpoint
        if _TCP_ENDPOINT_PATTERN.fullmatch(endpoint) is None:
            raise ValueError(f"Bridge '{bridge.name}' endpoint must use tcp://host:port format")

        validated.append(
            {
                "name": bridge.name,
                "enabled": bridge.enabled,
                "endpoint": endpoint,
            }
        )

    return validated


def build_bridge_endpoint_overrides(
    bridge_entries: list[dict[str, object]],
    bridge_host: str,
    endpoint_overrides: Mapping[str, str] | None = None,
) -> dict[str, dict[str, dict[str, str]]]:
    endpoint_overrides = endpoint_overrides or {}
    overrides: dict[str, dict[str, dict[str, str]]] = {"overrides": {}}
    for bridge in bridge_entries:
        if not bool(bridge["enabled"]):
            continue
        bridge_name = str(bridge["name"])
        endpoint = endpoint_overrides.get(bridge_name, str(bridge["endpoint"]))
        endpoint_match = _TCP_ENDPOINT_PATTERN.fullmatch(endpoint)
        if endpoint_match is None:
            raise ValueError(f"Invalid TCP endpoint: {endpoint}")
        overrides["overrides"][bridge_name] = {"input_endpoint": f"tcp://{bridge_host}:{endpoint_match.group('port')}"}
    return overrides


def resolve_bridge_host(bridge_mode: Literal["linux", "wsl"]) -> str:
    if bridge_mode == "linux":
        return "127.0.0.1"

    resolv_conf = Path("/etc/resolv.conf")
    if not resolv_conf.exists():
        return "127.0.0.1"

    for raw_line in resolv_conf.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line.startswith("nameserver"):
            continue
        parts = re.split(r"\s+", line)
        if len(parts) >= 2:
            return parts[1]
    return "127.0.0.1"


def build_ros_launch_stack_plan(
    *,
    play_executable: Optional[str],
    enable_px4_post_start_setup: bool,
    readiness_mode: Literal["background", "wait", "off"],
) -> StackPlan:
    return core_build_ros_launch_stack_plan(
        play_executable=play_executable,
        enable_px4_post_start_setup=enable_px4_post_start_setup,
        readiness_mode=readiness_mode,
    )


def build_px4_post_start_command(sensor_params: PX4SensorParams) -> list[str]:
    gps_home_lat = 0.0
    gps_home_lon = 0.0
    gps_alt_start = 0.0
    if sensor_params.fusion_mode == "mocap":
        gps_home_lat = float(sensor_params.gps_home_lat_lon[0])
        gps_home_lon = float(sensor_params.gps_home_lat_lon[1])
        gps_alt_start = float(sensor_params.gps_alt_start)

    return [
        sys.executable,
        "-m",
        "acesim_ros2.px4_post_start_setup",
        sensor_params.fusion_mode,
        str(gps_home_lat),
        str(gps_home_lon),
        str(gps_alt_start),
    ]


def python_launch_kwargs(*, additional_env: Optional[dict[str, str]] = None) -> dict[str, Any]:
    merged_env = {"PYTHONUNBUFFERED": "1"}
    if additional_env:
        merged_env.update(additional_env)
    return {
        "output": "both",
        "emulate_tty": True,
        "additional_env": merged_env,
    }


def build_python_module_run_command(
    package: str,
    executable: str,
    additional_env: Mapping[str, str] | None = None,
    extra_args: Optional[list[str]] = None,
) -> str:
    env = {"PYTHONUNBUFFERED": "1"}
    if additional_env:
        env.update(dict(additional_env))
    exports = " ".join(f"{name}={shlex.quote(value)}" for name, value in sorted(env.items()))
    args = " ".join(shlex.quote(arg) for arg in (extra_args or []))
    suffix = f" {args}" if args else ""
    return f"env {exports} ros2 run {shlex.quote(package)} {shlex.quote(executable)}{suffix}"


def build_px4_post_start_setup_process(additional_env: Optional[dict[str, str]] = None) -> ExecuteProcess:
    sensor_params = PX4SensorParams.from_asset_params(
        ConfigLoader().get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )
    return ExecuteProcess(
        cmd=build_px4_post_start_command(sensor_params),
        **python_launch_kwargs(additional_env=additional_env),
    )


def build_play_action(play_executable: str, additional_play_env: Optional[dict[str, str]] = None):
    if play_executable == "acesim_play_headless":
        return ExecuteProcess(
            cmd=build_graceful_shutdown_command(
                build_python_module_run_command(
                    "acesim_ros2",
                    play_executable,
                    additional_play_env,
                )
            ),
            output="screen",
            emulate_tty=True,
        )
    return Node(
        package="acesim_ros2",
        executable=play_executable,
        **python_launch_kwargs(additional_env=additional_play_env),
    )


def build_launch_entities(
    px4_repo_path: str,
    *,
    bridge_mode: Literal["linux", "wsl"] = "linux",
    play_executable: Optional[str] = "acesim_play",
    additional_play_env: Optional[dict[str, str]] = None,
    enable_px4_post_start_setup: bool = True,
    play_start_delay_sec: float = 0.0,
    px4_post_start_readiness_mode: Literal["background", "wait", "off"] = "background",
    ace_follower: Literal["auto", "true", "false"] = "auto",
):
    config_loader = ConfigLoader()
    px4_additional_env = build_px4_additional_env(config_loader)
    config_file = bridge_config_path()
    bridge_entries = load_bridge_entries(config_file)
    bridge_host = resolve_bridge_host(bridge_mode)
    overrides = build_bridge_endpoint_overrides(bridge_entries, bridge_host)
    handle = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="acesim_bridge_overrides_",
        suffix=".yaml",
        delete=False,
    )
    try:
        yaml.safe_dump(overrides, handle, sort_keys=False)
        overrides_file = handle.name
    finally:
        handle.close()
    if ace_follower == "true":
        start_ace_follower = play_executable is not None
    elif ace_follower == "false":
        start_ace_follower = False
    elif ace_follower == "auto":
        start_ace_follower = play_executable is not None and (
            config_loader.get_env_type() == "am" or config_loader.get_asset_name() == "x500_arm2x"
        )
    else:
        raise ValueError("ace_follower must be 'auto', 'true', or 'false'")
    if start_ace_follower:
        additional_play_env = dict(additional_play_env or {})
        additional_play_env.setdefault("ACESIM_ARM_COMMAND_STREAM_ENABLED", "1")
        additional_play_env.setdefault("ACESIM_ARM_COMMAND_STREAM_ENDPOINT", f"tcp://{bridge_host}:5604")
    entities = [
        ExecuteProcess(cmd=build_graceful_shutdown_command("MicroXRCEAgent udp4 -p 8888"), output="screen"),
        ExecuteProcess(
            cmd=build_graceful_shutdown_command(build_px4_make_command(px4_additional_env), filter_px4_prompt=True),
            cwd=px4_repo_path,
            additional_env=px4_additional_env,
            output="screen",
        ),
        ExecuteProcess(
            cmd=build_graceful_shutdown_command(
                build_python_module_run_command(
                    "acesim_ros2",
                    "acesim_bridge",
                    extra_args=[
                        "--ros-args",
                        "-r",
                        "__node:=acesim_bridge",
                        "-p",
                        f"bridge_overrides_file:={overrides_file}",
                    ],
                )
            ),
            output="screen",
            emulate_tty=True,
        ),
    ]
    if start_ace_follower:
        entities.append(
            ExecuteProcess(
                cmd=build_graceful_shutdown_command(
                    build_python_module_run_command(
                        "acesim_ros2",
                        "acesim_ace_follower",
                        {"ACESIM_ACE_FOLLOWER_COMMAND_ENDPOINT": "tcp://0.0.0.0:5604"},
                    )
                ),
                output="screen",
                emulate_tty=True,
            )
        )

    acesim_play = build_play_action(play_executable, additional_play_env) if play_executable else None
    px4_post_start_setup = None
    if enable_px4_post_start_setup:
        post_start_env = {
            "ACESIM_PX4_READINESS_MODE": px4_post_start_readiness_mode,
            "ACESIM_PX4_VERIFY_ARMABLE": os.environ.get("ACESIM_PX4_VERIFY_ARMABLE", "1"),
        }
        px4_post_start_setup = build_px4_post_start_setup_process(post_start_env)
        entities.append(px4_post_start_setup)

    if acesim_play is not None:
        if px4_post_start_setup is None:
            entities.append(acesim_play)
        else:
            entities.append(
                RegisterEventHandler(
                    OnProcessStart(
                        target_action=px4_post_start_setup,
                        on_start=[TimerAction(period=play_start_delay_sec, actions=[acesim_play])],
                    )
                )
            )
        entities.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=acesim_play,
                    on_exit=[EmitEvent(event=Shutdown(reason="ACESim frontend exited"))],
                )
            )
        )

    return entities
