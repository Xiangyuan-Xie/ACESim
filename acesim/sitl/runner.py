from __future__ import annotations

import os
import shlex
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from pymavlink import mavutil

from acesim.config.config_loader import ConfigLoader
from acesim.sitl.process import (
    ProcessSpec,
    ProcessSupervisor,
    build_graceful_shutdown_command,
)
from acesim.sitl.px4_bootstrap import build_px4_env, build_px4_make_command, load_px4_repo_path
from acesim.sitl.readiness import (
    run_background_readiness_diagnostics,
    run_required_px4_setup,
    run_strict_px4_readiness,
    wait_for_mavlink,
)
from acesim.sitl.stack_plan import StackPlan, build_core_sitl_stack_plan
from acesim.utils.px4_transport import PX4SensorParams


@dataclass(frozen=True)
class PX4SITLConfig:
    px4_repo: Path | str | None = None
    config_path: Path | str | None = None
    headless: bool = True
    px4_instance: int = 0
    readiness_mode: Literal["background", "wait", "off"] = "background"
    play_start_delay_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.px4_instance < 0:
            raise ValueError("px4_instance must be >= 0")
        if self.px4_instance > 9:
            raise ValueError("px4_instance must be <= 9 for PX4's default MAVLink port mapping")
        if self.readiness_mode not in {"background", "wait", "off"}:
            raise ValueError("readiness_mode must be one of: background, wait, off")

    @property
    def resolved_px4_repo(self) -> Path:
        return load_px4_repo_path(self.px4_repo)

    @property
    def resolved_config_path(self) -> Path | None:
        if self.config_path is None:
            return None
        return Path(self.config_path).expanduser().resolve()

    @property
    def px4_sim_tcp_port(self) -> int:
        return 4560 + self.px4_instance

    @property
    def mavlink_udp_port(self) -> int:
        return 14540 + self.px4_instance

    @property
    def px4_mavlink_url(self) -> str:
        return f"udpin:0.0.0.0:{self.mavlink_udp_port}"


class PX4SITLRunner:
    def __init__(self, config: PX4SITLConfig) -> None:
        self._config = config

    def stack_plan(self) -> StackPlan:
        return build_core_sitl_stack_plan(headless=self._config.headless, readiness_mode=self._config.readiness_mode)

    def build_process_specs(self, px4_env: dict[str, str] | None = None) -> list[ProcessSpec]:
        px4_env = dict(px4_env or build_px4_env(self._config_loader()))
        acesim_env = {
            "ACESIM_PX4_SIM_TCP_PORT": str(self._config.px4_sim_tcp_port),
            "ACESIM_PX4_MAVLINK_URL": self._config.px4_mavlink_url,
            "PYTHONUNBUFFERED": "1",
        }
        px4_process_env = dict(px4_env)
        px4_process_env["PX4_SIM_HOSTNAME"] = "127.0.0.1"
        px4_process_env["ACESIM_PX4_MAVLINK_URL"] = self._config.px4_mavlink_url
        specs: list[ProcessSpec] = []
        acesim_command = [sys.executable, "-m", "acesim.sitl.play"]
        if self._config.resolved_config_path is not None:
            acesim_command.extend(["--config", str(self._config.resolved_config_path)])
        acesim_command.append("--headless" if self._config.headless else "--gui")
        specs.append(ProcessSpec(name="acesim", command=acesim_command, env=acesim_env))
        specs.append(
            ProcessSpec(
                name="px4",
                command=build_graceful_shutdown_command(
                    self._build_px4_command(px4_process_env),
                    filter_px4_prompt=True,
                ),
                cwd=self._config.resolved_px4_repo,
                env=px4_process_env,
            )
        )
        return specs

    def run(self) -> int:
        config_loader = self._config_loader()
        px4_env = build_px4_env(config_loader)
        sensor_params = PX4SensorParams.from_asset_params(
            config_loader.get_asset_params(),
            dynamic_hil_sensor_fields=False,
        )
        supervisor = ProcessSupervisor()
        old_mavlink_url = os.environ.get("ACESIM_PX4_MAVLINK_URL")
        os.environ["ACESIM_PX4_MAVLINK_URL"] = self._config.px4_mavlink_url
        ready_dir = tempfile.TemporaryDirectory(prefix="acesim-sitl-ready-")
        ready_path = Path(ready_dir.name) / "ready"
        try:
            specs = self.build_process_specs(px4_env)
            for spec in specs:
                if spec.name == "acesim":
                    spec.env["ACESIM_SITL_READY_FILE"] = str(ready_path)
                supervisor.start(spec)
                if spec.name == "acesim":
                    if not wait_for_acesim_ready_marker(ready_path):
                        print(
                            "[acesim] frontend did not report HIL endpoint readiness before timeout",
                            flush=True,
                        )
                        return 1
                if spec.name == "px4" and self._config.play_start_delay_sec > 0.0:
                    time.sleep(self._config.play_start_delay_sec)
                exited = supervisor.poll_exited()
                if exited is not None:
                    exited_spec, returncode = exited
                    print(f"[{exited_spec.name}] exited with code {returncode}", flush=True)
                    return returncode
            if self._config.readiness_mode != "off":
                mav = mavutil.mavlink_connection(
                    self._config.px4_mavlink_url,
                    source_system=250,
                    source_component=190,
                    autoreconnect=True,
                )
                wait_for_mavlink(mav)
                if self._config.readiness_mode == "wait":
                    run_strict_px4_readiness(
                        mav,
                        sensor_params.fusion_mode,
                        float(sensor_params.gps_home_lat_lon[0]),
                        float(sensor_params.gps_home_lat_lon[1]),
                        float(sensor_params.gps_alt_start),
                    )
                else:
                    run_required_px4_setup(
                        mav,
                        sensor_params.fusion_mode,
                        float(sensor_params.gps_home_lat_lon[0]),
                        float(sensor_params.gps_home_lat_lon[1]),
                        float(sensor_params.gps_alt_start),
                    )
                    start_background_readiness_diagnostics(
                        mav,
                        sensor_params.fusion_mode,
                        float(sensor_params.gps_home_lat_lon[0]),
                        float(sensor_params.gps_home_lat_lon[1]),
                        float(sensor_params.gps_alt_start),
                    )
            spec, returncode = supervisor.wait_for_any_exit()
            print(f"[{spec.name}] exited with code {returncode}", flush=True)
            return returncode
        except KeyboardInterrupt:
            return 0
        finally:
            supervisor.terminate_all()
            ready_dir.cleanup()
            if old_mavlink_url is None:
                os.environ.pop("ACESIM_PX4_MAVLINK_URL", None)
            else:
                os.environ["ACESIM_PX4_MAVLINK_URL"] = old_mavlink_url

    def _config_loader(self) -> ConfigLoader:
        if self._config.resolved_config_path is None:
            return ConfigLoader()
        return ConfigLoader(self._config.resolved_config_path)

    def _build_px4_command(self, px4_env: dict[str, str]) -> str:
        if self._config.px4_instance == 0:
            return build_px4_make_command(px4_env)

        build_path = self._config.resolved_px4_repo / "build" / "px4_sitl_default"
        px4_bin = build_path / "bin" / "px4"
        rootfs_path = build_path / "rootfs" / str(self._config.px4_instance)
        data_path = build_path / "etc"
        exports = " ".join(f"{name}={shlex.quote(value)}" for name, value in sorted(px4_env.items()))
        command = " ".join(
            shlex.quote(str(token))
            for token in [
                str(px4_bin),
                "-d",
                "-i",
                str(self._config.px4_instance),
                "-w",
                str(rootfs_path),
                str(data_path),
            ]
        )
        return f"mkdir -p {shlex.quote(str(rootfs_path))} && env {exports} {command}"


def wait_for_acesim_ready_marker(path: Path, timeout_sec: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if path.exists():
            return True
        time.sleep(0.05)
    return False


def start_background_readiness_diagnostics(
    mav: object,
    fusion_mode: str,
    gps_home_lat: float,
    gps_home_lon: float,
    gps_alt_start: float,
) -> threading.Thread:
    thread = threading.Thread(
        target=run_background_readiness_diagnostics,
        args=(mav, fusion_mode, gps_home_lat, gps_home_lon, gps_alt_start),
        name="acesim-px4-readiness-diagnostics",
        daemon=True,
    )
    thread.start()
    return thread
