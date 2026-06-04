from __future__ import annotations

import importlib.util
import os
import shlex
from pathlib import Path
from typing import Type

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


def detect_repo_root() -> Path:
    spec = importlib.util.find_spec("acesim")
    if spec is not None:
        locations = spec.submodule_search_locations
        if locations:
            return Path(next(iter(locations))).resolve().parent
        origin = spec.origin
        if origin:
            return Path(origin).resolve().parent.parent
    env = os.environ.get("ACESIM_ROOT")
    if env:
        return Path(env).resolve()
    raise RuntimeError("Failed to locate ACESim repository; set ACESIM_ROOT or pass px4_repo")


def detect_acesim_root() -> Path:
    """Backward-compatible alias for older installed ROS launch wrappers."""

    return detect_repo_root()


def load_px4_repo_path(override: str | Path | None = None) -> Path:
    if override is not None and str(override).strip():
        value = Path(str(override).strip()).expanduser()
        if not value.is_absolute():
            value = (detect_repo_root() / value).resolve()
        return value.resolve()
    return (detect_repo_root() / "acesim" / "third_party" / "aircraft" / "PX4-Autopilot").resolve()


def resolve_px4_startup_env(config_loader: ConfigLoader | None = None) -> dict[str, str]:
    if config_loader is None:
        config_loader = ConfigLoader()
    asset_name = config_loader.get_asset_name()
    startup_env = PX4_STARTUP_ENV_BY_ASSET.get(asset_name)
    if startup_env is not None:
        return dict(startup_env)
    supported_assets = ", ".join(sorted(PX4_STARTUP_ENV_BY_ASSET))
    raise ValueError(f"Unsupported PX4 startup asset: {asset_name}. Supported assets: {supported_assets}")


def build_px4_env(
    config_loader: ConfigLoader | None = None,
    *,
    sensor_params_cls: Type[PX4SensorParams] = PX4SensorParams,
) -> dict[str, str]:
    if config_loader is None:
        config_loader = ConfigLoader()
    sensor_params = sensor_params_cls.from_asset_params(
        config_loader.get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )
    additional_env = resolve_px4_startup_env(config_loader)
    additional_env.update(
        {
            "PX4_PARAM_COM_MODE_ARM_CHK": "1",
            "PX4_PARAM_CBRK_SUPPLY_CHK": "894281",
            "PX4_PARAM_SIM_BAT_ENABLE": "1",
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
    exports = " ".join(f"{name}={shlex.quote(value)}" for name, value in sorted(additional_env.items()))
    return f"env {exports} make px4_sitl none"
