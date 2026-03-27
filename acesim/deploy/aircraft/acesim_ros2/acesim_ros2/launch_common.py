from __future__ import annotations

import importlib.util
import os
from pathlib import Path
from typing import Optional

from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from acesim.config.config_loader import ConfigLoader
from acesim.utils.px4_transport import PX4SensorParams


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


def build_px4_additional_env() -> dict[str, str]:
    sensor_params = PX4SensorParams.from_asset_params(
        ConfigLoader().get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )
    additional_env = {"PX4_SIM_MODEL": "none_iris"}
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


def build_linux_launch_entities(px4_repo_path: str, *, play_executable: str = "acesim_play"):
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
        arguments=["--mode", "linux"],
        output="screen",
    )
    arm_state_bridge = Node(
        package="acesim_ros2",
        executable="arm_state_zmq_bridge",
        arguments=["--mode", "linux"],
        output="screen",
    )
    acesim_play = Node(
        package="acesim_ros2",
        executable=play_executable,
        output="screen",
    )
    return [
        micro_xrce_agent,
        px4_sitl,
        px4_post_start_setup,
        arm_command_joint_state_pub,
        clock_bridge,
        arm_state_bridge,
        RegisterEventHandler(
            OnProcessStart(
                target_action=px4_post_start_setup,
                on_start=[TimerAction(period=2.0, actions=[acesim_play])],
            )
        ),
    ]
