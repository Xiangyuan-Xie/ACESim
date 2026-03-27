from __future__ import annotations

import time

from pymavlink import mavutil

from acesim.config.config_loader import ConfigLoader
from acesim.utils.px4_transport import PX4SensorParams


def _connect_shell():
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
        heartbeat = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if heartbeat is not None:
            return mav
    raise RuntimeError("Failed to connect to PX4 MAVLink shell on udpout:127.0.0.1:14580")


def _drain_shell_output(mav, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    chunks: list[str] = []
    while time.monotonic() < deadline:
        reply = mav.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
        if reply is None:
            continue
        count = int(getattr(reply, "count", 0))
        if count <= 0:
            continue
        data = bytes(reply.data[:count]).decode("utf-8", errors="ignore")
        chunks.append(data)
    return "".join(chunks)


def _run_shell_command(mav, command: str, *, expect: str | None = None, retries: int = 8) -> str:
    payload = command.strip() + "\n"
    last_output = ""
    for _ in range(retries):
        _drain_shell_output(mav, 0.2)
        remaining = payload
        while remaining:
            chunk = remaining[:70]
            remaining = remaining[70:]
            data = [ord(c) for c in chunk]
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
        last_output = _drain_shell_output(mav, 1.0)
        lowered = last_output.lower()
        if "error" in lowered or "not found" in lowered or "nack" in lowered or "failed" in lowered:
            time.sleep(0.3)
            continue
        if expect is None or expect.lower() in lowered:
            return last_output
        time.sleep(0.3)

    raise RuntimeError(f"PX4 shell command failed: {command}\nOutput:\n{last_output.strip()}")


def main() -> None:
    sensor_params = PX4SensorParams.from_asset_params(
        ConfigLoader().get_asset_params(),
        dynamic_hil_sensor_fields=False,
    )

    mav = _connect_shell()
    try:
        _run_shell_command(mav, "ver all")
        if sensor_params.fusion_mode == "mocap":
            _run_shell_command(
                mav,
                (
                    "commander set_ekf_origin "
                    f"{sensor_params.gps_home_lat_lon[0]} "
                    f"{sensor_params.gps_home_lat_lon[1]} "
                    f"{sensor_params.gps_alt_start}"
                ),
            )
            _run_shell_command(mav, "listener vehicle_global_position 1", expect="lat")
    finally:
        mav.mav.serial_control_send(mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)


if __name__ == "__main__":
    main()
