from __future__ import annotations

import importlib.util
import os
import re
import shlex
import sys
import tempfile
import textwrap
from pathlib import Path
from typing import Literal, Optional

if __package__ in (None, ""):
    package_parent = Path(__file__).resolve().parents[1]
    if str(package_parent) not in sys.path:
        sys.path.insert(0, str(package_parent))

import yaml
from acesim_ros2.bridge.registry import PLUGIN_REGISTRY
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from acesim.config.config_loader import ConfigLoader
from acesim.utils.px4_transport import PX4SensorParams

PX4_STARTUP_ENV_BY_ASSET: dict[str, dict[str, str]] = {
    "iris": {
        "PX4_SYS_AUTOSTART": "10016",
        "PX4_SIM_MODEL": "none",
    },
    "x500": {
        "PX4_SYS_AUTOSTART": "10016",
        "PX4_SIM_MODEL": "none",
    },
    "x500_arm2x": {
        "PX4_SYS_AUTOSTART": "10016",
        "PX4_SIM_MODEL": "none",
    },
    "typhoon_h480": {
        "PX4_SYS_AUTOSTART": "6011",
        "PX4_SIM_MODEL": "none",
    },
    # These assets reuse PX4's gz_* airframe parameter sets, but ACESim still
    # runs them through `make px4_sitl none` with HIL sensors/actuators. Force
    # Gazebo back off so PX4 stays on the simulator_mavlink path. Overriding
    # only SIM_GZ_EN is not enough because the gz_* airframe scripts also set
    # PX4_SIMULATOR=gz, and px4-rc.simulator enters gz whenever either signal
    # is present.
    "advanced_plane": {
        "PX4_SYS_AUTOSTART": "1039",
        "PX4_SIM_MODEL": "none",
        "PX4_SIMULATOR": "none",
        "PX4_PARAM_SIM_GZ_EN": "0",
    },
    "standard_vtol": {
        "PX4_SYS_AUTOSTART": "1040",
        "PX4_SIM_MODEL": "none",
        "PX4_SIMULATOR": "none",
        "PX4_PARAM_SIM_GZ_EN": "0",
    },
    "uuv_bluerov2_heavy": {
        "PX4_SYS_AUTOSTART": "60002",
        "PX4_SIM_MODEL": "none",
        "PX4_SIMULATOR": "none",
        "PX4_PARAM_SIM_GZ_EN": "0",
    },
}
_TCP_ENDPOINT_PATTERN = re.compile(r"^tcp://(?P<host>[^:/]+):(?P<port>\d+)$")


def detect_acesim_root() -> Path:
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


def load_px4_repo_path(override: Optional[str]) -> str:
    if isinstance(override, str) and override.strip():
        value = Path(override.strip())
        if not value.is_absolute():
            value = (detect_acesim_root() / value).resolve()
        return str(value)
    return str((detect_acesim_root() / "third_party" / "aircraft" / "PX4-Autopilot").resolve())


def resolve_px4_startup_env() -> dict[str, str]:
    """Map the configured ACESim asset onto the PX4 airframe startup environment."""
    asset_name = ConfigLoader().get_asset_name()
    startup_env = PX4_STARTUP_ENV_BY_ASSET.get(asset_name)
    if startup_env is not None:
        return dict(startup_env)
    supported_assets = ", ".join(sorted(PX4_STARTUP_ENV_BY_ASSET))
    raise ValueError(f"Unsupported PX4 startup asset: {asset_name}. Supported assets: {supported_assets}")


def build_px4_additional_env() -> dict[str, str]:
    sensor_params = PX4SensorParams.from_asset_params(
        ConfigLoader().get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )
    additional_env = resolve_px4_startup_env()
    additional_env.update(
        {
            # Keep external modes registered and checked while armed so the
            # internally hosted RL mode remains selectable after takeoff.
            "PX4_PARAM_COM_MODE_ARM_CHK": "1",
        }
    )
    if sensor_params.fusion_mode == "hil":
        additional_env.update(
            {
                "PX4_PARAM_EKF2_EV_CTRL": "0",
                "PX4_PARAM_EKF2_GPS_CTRL": "7",
                "PX4_PARAM_EKF2_HGT_REF": "1",
                "PX4_PARAM_EKF2_MAG_TYPE": "0",
                "PX4_PARAM_SYS_HAS_GPS": "1",
                "PX4_PARAM_SYS_HAS_MAG": "1",
                "PX4_PARAM_SYS_HAS_BARO": "1",
            }
        )
        return additional_env

    hgt_ref_by_name = {
        "Baro": "0",
        "GPS": "1",
        "Range sensor": "2",
        "Vision": "3",
    }
    if sensor_params.ekf2_hgt_ref not in hgt_ref_by_name:
        raise ValueError(f"Unsupported EKF2_HGT_REF value: {sensor_params.ekf2_hgt_ref}")

    additional_env.update(
        {
            "PX4_PARAM_EKF2_EV_CTRL": str(sensor_params.ekf2_ev_ctrl),
            "PX4_PARAM_EKF2_HGT_REF": hgt_ref_by_name[sensor_params.ekf2_hgt_ref],
            "PX4_PARAM_EKF2_EV_DELAY": str(sensor_params.ekf2_ev_delay_ms),
            "PX4_PARAM_EKF2_EV_POS_X": str(sensor_params.ekf2_ev_pos_body_m[0]),
            "PX4_PARAM_EKF2_EV_POS_Y": str(sensor_params.ekf2_ev_pos_body_m[1]),
            "PX4_PARAM_EKF2_EV_POS_Z": str(sensor_params.ekf2_ev_pos_body_m[2]),
            "PX4_PARAM_EKF2_EV_NOISE_MD": str(sensor_params.ekf2_ev_noise_md),
            "PX4_PARAM_EKF2_EVP_NOISE": str(sensor_params.ekf2_evp_noise),
            "PX4_PARAM_EKF2_EVV_NOISE": str(sensor_params.ekf2_evv_noise),
            "PX4_PARAM_EKF2_EVA_NOISE": str(sensor_params.ekf2_eva_noise),
            "PX4_PARAM_EKF2_GPS_CTRL": str(sensor_params.ekf2_gps_ctrl),
            "PX4_PARAM_EKF2_MAG_TYPE": str(sensor_params.ekf2_mag_type),
            "PX4_PARAM_SYS_HAS_GPS": "0",
            "PX4_PARAM_SYS_HAS_MAG": "0",
            "PX4_PARAM_SYS_HAS_BARO": "0",
        }
    )
    return additional_env


def build_px4_make_command(additional_env: dict[str, str]) -> str:
    """Launch PX4 with explicit exported overrides so gz airframes stay on mavlinksim."""

    exports = " ".join(f"{name}={shlex.quote(value)}" for name, value in sorted(additional_env.items()))
    return f"env {exports} make px4_sitl none"


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
    config_path = Path(config_file)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"Bridge config must be a mapping: {config_path}")

    bridges = config.get("bridges")
    if not isinstance(bridges, dict):
        raise ValueError(f"Bridge config must define a mapping under 'bridges': {config_path}")

    validated: list[dict[str, object]] = []
    seen_ids: set[str] = set()
    for bridge_name, bridge in bridges.items():
        if not isinstance(bridge_name, str) or not bridge_name:
            raise ValueError(f"Each bridge entry must use a non-empty string name: {config_path}")
        if not isinstance(bridge, dict):
            raise ValueError(f"Bridge '{bridge_name}' must be a mapping: {config_path}")
        if bridge_name in seen_ids:
            raise ValueError(f"Duplicate bridge name: {bridge_name}")
        seen_ids.add(bridge_name)
        if bridge_name not in PLUGIN_REGISTRY:
            supported = ", ".join(PLUGIN_REGISTRY)
            raise ValueError(f"Unsupported bridge name: {bridge_name}. Supported bridges: {supported}")
        if "handler" in bridge:
            raise ValueError(f"Bridge '{bridge_name}' must not define 'handler'; the bridge name is the type")

        transport = bridge.get("transport")
        if not isinstance(transport, dict):
            raise ValueError(f"Bridge '{bridge_name}' must define a transport mapping")

        endpoint = transport.get("endpoint")
        if not isinstance(endpoint, str) or not endpoint:
            raise ValueError(f"Bridge '{bridge_name}' transport must define a non-empty endpoint")
        if _TCP_ENDPOINT_PATTERN.fullmatch(endpoint) is None:
            raise ValueError(f"Bridge '{bridge_name}' endpoint must use tcp://host:port format")

        validated.append(
            {
                "name": bridge_name,
                "enabled": bool(bridge.get("enabled", True)),
                "endpoint": endpoint,
            }
        )

    return validated


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


def build_px4_post_start_command(sensor_params: PX4SensorParams) -> list[str]:
    gps_home_lat = 0.0
    gps_home_lon = 0.0
    gps_alt_start = 0.0
    if sensor_params.fusion_mode == "mocap":
        gps_home_lat = float(sensor_params.gps_home_lat_lon[0])
        gps_home_lon = float(sensor_params.gps_home_lat_lon[1])
        gps_alt_start = float(sensor_params.gps_alt_start)

    script = textwrap.dedent("""
        import sys
        import time
        from pymavlink import mavutil

        fusion_mode = sys.argv[1]
        gps_home_lat = float(sys.argv[2])
        gps_home_lon = float(sys.argv[3])
        gps_alt_start = float(sys.argv[4])

        mav = mavutil.mavlink_connection(
            "udpout:127.0.0.1:14580",
            source_system=250,
            source_component=190,
            autoreconnect=True,
        )

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            if mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0) is not None:
                break
        else:
            raise RuntimeError("Failed to connect to PX4 MAVLink shell on udpout:127.0.0.1:14580")

        def run_shell(command: str, expect: str | None = None, retries: int = 8) -> None:
            payload = command.strip() + "\\n"
            last_output = ""
            for _ in range(retries):
                drain_deadline = time.monotonic() + 0.2
                while time.monotonic() < drain_deadline:
                    reply = mav.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.05)
                    if reply is None:
                        continue

                remaining = payload
                while remaining:
                    chunk = remaining[:70]
                    remaining = remaining[70:]
                    data = [ord(char) for char in chunk]
                    data.extend([0] * (70 - len(data)))
                    mav.mav.serial_control_send(
                        mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                        mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                        0,
                        0,
                        len(chunk),
                        data,
                    )
                    time.sleep(0.05)

                chunks = []
                output_deadline = time.monotonic() + 1.0
                while time.monotonic() < output_deadline:
                    reply = mav.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
                    if reply is None or int(getattr(reply, "count", 0)) <= 0:
                        continue
                    chunks.append(bytes(reply.data[: reply.count]).decode("utf-8", errors="ignore"))

                last_output = "".join(chunks)
                lowered = last_output.lower()
                if "error" in lowered or "not found" in lowered or "nack" in lowered or "failed" in lowered:
                    time.sleep(0.3)
                    continue
                if expect is None or expect.lower() in lowered:
                    return
                time.sleep(0.3)

            raise RuntimeError(f"PX4 shell command failed: {command}\\nOutput:\\n{last_output.strip()}")

        try:
            run_shell("ver all")
            if fusion_mode == "mocap":
                run_shell(f"commander set_ekf_origin {gps_home_lat} {gps_home_lon} {gps_alt_start}")
                run_shell("listener vehicle_global_position 1", expect="lat")
        finally:
            mav.mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)
        """).strip()
    return [
        sys.executable,
        "-c",
        script,
        sensor_params.fusion_mode,
        str(gps_home_lat),
        str(gps_home_lon),
        str(gps_alt_start),
    ]


def build_px4_post_start_setup_process() -> ExecuteProcess:
    sensor_params = PX4SensorParams.from_asset_params(
        ConfigLoader().get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )
    return ExecuteProcess(
        cmd=build_px4_post_start_command(sensor_params),
        output="screen",
    )


def build_launch_entities(
    px4_repo_path: str,
    *,
    bridge_mode: Literal["linux", "wsl"] = "linux",
    play_executable: Optional[str] = "acesim_play",
    enable_px4_post_start_setup: bool = True,
    play_start_delay_sec: float = 2.0,
):
    px4_additional_env = build_px4_additional_env()
    config_file = bridge_config_path()
    bridge_entries = load_bridge_entries(config_file)
    bridge_host = resolve_bridge_host(bridge_mode)
    overrides: dict[str, dict[str, dict[str, str]]] = {"overrides": {}}
    for bridge in bridge_entries:
        if not bool(bridge["enabled"]):
            continue
        endpoint_match = _TCP_ENDPOINT_PATTERN.fullmatch(str(bridge["endpoint"]))
        if endpoint_match is None:
            raise ValueError(f"Invalid TCP endpoint: {bridge['endpoint']}")
        overrides["overrides"][str(bridge["name"])] = {
            "input_endpoint": f"tcp://{bridge_host}:{endpoint_match.group('port')}"
        }
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
    entities = [
        ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen"),
        ExecuteProcess(
            cmd=["bash", "-lc", build_px4_make_command(px4_additional_env)],
            cwd=px4_repo_path,
            additional_env=px4_additional_env,
            output="screen",
        ),
        Node(
            package="acesim_ros2",
            executable="acesim_bridge",
            name="acesim_bridge",
            parameters=[
                {
                    "bridge_overrides_file": overrides_file,
                },
            ],
            output="screen",
        ),
    ]

    px4_post_start_setup = None
    if enable_px4_post_start_setup:
        px4_post_start_setup = build_px4_post_start_setup_process()
        entities.append(px4_post_start_setup)

    if enable_px4_post_start_setup and play_executable:
        acesim_play = Node(
            package="acesim_ros2",
            executable=play_executable,
            output="screen",
        )
        entities.append(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=px4_post_start_setup,
                    on_start=[TimerAction(period=play_start_delay_sec, actions=[acesim_play])],
                )
            )
        )

    return entities
