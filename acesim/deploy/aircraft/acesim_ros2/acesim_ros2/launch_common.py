from __future__ import annotations

import importlib.util
import os
from pathlib import Path
from typing import Literal, Optional

from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from acesim.config.config_loader import ConfigLoader
from acesim.utils.px4_transport import PX4SensorParams

_PX4_STARTUP_ENV_BY_ASSET: dict[str, dict[str, str]] = {
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
    "plane": {
        "PX4_SYS_AUTOSTART": "4008",
        "PX4_SIM_MODEL": "none",
        "PX4_SIMULATOR": "none",
        "PX4_PARAM_SIM_GZ_EN": "0",
    },
    "standard_vtol": {
        "PX4_SYS_AUTOSTART": "4004",
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


def _resolve_hgt_ref_value(hgt_ref: str) -> str:
    mapping = {
        "Baro": "0",
        "GPS": "1",
        "Range sensor": "2",
        "Vision": "3",
    }
    if hgt_ref not in mapping:
        raise ValueError(f"Unsupported EKF2_HGT_REF value: {hgt_ref}")
    return mapping[hgt_ref]


def _resolve_px4_startup_env() -> dict[str, str]:
    """Map the configured ACESim asset onto the PX4 airframe startup environment."""
    asset_name = ConfigLoader().get_asset_name()
    startup_env = _PX4_STARTUP_ENV_BY_ASSET.get(asset_name)
    if startup_env is not None:
        return dict(startup_env)
    supported_assets = ", ".join(sorted(_PX4_STARTUP_ENV_BY_ASSET))
    raise ValueError(f"Unsupported PX4 startup asset: {asset_name}. Supported assets: {supported_assets}")


def build_px4_additional_env() -> dict[str, str]:
    sensor_params = PX4SensorParams.from_asset_params(
        ConfigLoader().get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )
    additional_env = _resolve_px4_startup_env()
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

    additional_env.update(
        {
            "PX4_PARAM_EKF2_EV_CTRL": str(sensor_params.ekf2_ev_ctrl),
            "PX4_PARAM_EKF2_HGT_REF": _resolve_hgt_ref_value(sensor_params.ekf2_hgt_ref),
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


def build_launch_entities(
    px4_repo_path: str,
    *,
    bridge_mode: Literal["linux", "wsl"] = "linux",
    play_executable: Optional[str] = "acesim_play",
    enable_px4_post_start_setup: bool = True,
    play_start_delay_sec: float = 2.0,
):
    config_loader = ConfigLoader()
    env_type = config_loader.get_env_type()
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
    )
    px4_sitl = ExecuteProcess(
        cmd=["bash", "-lc", "make px4_sitl none"],
        cwd=px4_repo_path,
        additional_env=build_px4_additional_env(),
        output="screen",
    )
    px4_post_start_setup = Node(
        package="acesim_ros2",
        executable="px4_post_start_setup",
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
    )
    clock_bridge = Node(
        package="acesim_ros2",
        executable="simulation_clock_zmq_bridge",
        arguments=["--mode", bridge_mode],
        output="screen",
    )
    arm_state_bridge = Node(
        package="acesim_ros2",
        executable="arm_state_zmq_bridge",
        arguments=["--mode", bridge_mode],
        output="screen",
    )
    entities = [
        micro_xrce_agent,
        px4_sitl,
        clock_bridge,
    ]
    if env_type == "mc_arm":
        entities.extend([arm_command_joint_state_pub, arm_state_bridge])

    if enable_px4_post_start_setup:
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
