from __future__ import annotations

import math
import os
import signal
import sys
import time
import traceback
from typing import Any

from pymavlink import mavutil

PX4_CONNECT_TIMEOUT_SEC = 180.0
PX4_MAVLINK_URL = "udpin:0.0.0.0:14540"
REQUIRED_READY_STABLE_SEC = 5.0
ARMABILITY_TOTAL_TIMEOUT_SEC = 120.0
ARMABILITY_RETRY_DELAY_SEC = 1.0
READINESS_STALE_SEC = 2.0
HOME_ORIGIN_TOLERANCE_DEG = 0.001
HOME_ALT_TOLERANCE_M = 0.5
READINESS_MESSAGE_INTERVAL_HZ = 10.0
PREARM_INNOVATION_RATIO_THRESHOLD = 0.5
MAX_RECENT_PREFLIGHT_STATUSTEXTS = 5
READINESS_MESSAGE_TYPES = [
    "LOCAL_POSITION_NED",
    "GLOBAL_POSITION_INT",
    "ESTIMATOR_STATUS",
    "SYS_STATUS",
    "STATUSTEXT",
]
MAV_CMD_SET_MESSAGE_INTERVAL = getattr(mavutil.mavlink, "MAV_CMD_SET_MESSAGE_INTERVAL", 511)
MAV_CMD_RUN_PREARM_CHECKS = getattr(mavutil.mavlink, "MAV_CMD_RUN_PREARM_CHECKS", 401)
MAV_CMD_COMPONENT_ARM_DISARM = getattr(mavutil.mavlink, "MAV_CMD_COMPONENT_ARM_DISARM", 400)
MAV_RESULT_ACCEPTED = getattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED", 0)
MAV_MODE_FLAG_SAFETY_ARMED = getattr(mavutil.mavlink, "MAV_MODE_FLAG_SAFETY_ARMED", 128)
MAV_SYS_STATUS_PREARM_CHECK = getattr(mavutil.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 268435456)
SYS_STATUS_MSG_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SYS_STATUS", 1)
LOCAL_POSITION_NED_MSG_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32)
GLOBAL_POSITION_INT_MSG_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33)
GPS_GLOBAL_ORIGIN_MSG_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN", 49)
ESTIMATOR_STATUS_MSG_ID = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_ESTIMATOR_STATUS", 230)

_REQUIRED_ESTIMATOR_FLAG_SPECS = [
    ("ESTIMATOR_ATTITUDE", 1),
    ("ESTIMATOR_VELOCITY_HORIZ", 2),
    ("ESTIMATOR_VELOCITY_VERT", 4),
    ("ESTIMATOR_POS_HORIZ_REL", 8),
    ("ESTIMATOR_POS_HORIZ_ABS", 16),
    ("ESTIMATOR_POS_VERT_ABS", 32),
]
_FORBIDDEN_ESTIMATOR_FLAG_SPECS = [
    ("ESTIMATOR_GPS_GLITCH", 1024),
    ("ESTIMATOR_ACCEL_ERROR", 2048),
]


def _mavlink_flag(name: str, fallback: int) -> int:
    return int(getattr(mavutil.mavlink, name, fallback))


_ESTIMATOR_FLAG_NAMES = [
    (name, _mavlink_flag(name, fallback))
    for name, fallback in [*_REQUIRED_ESTIMATOR_FLAG_SPECS, *_FORBIDDEN_ESTIMATOR_FLAG_SPECS]
]
REQUIRED_ESTIMATOR_FLAGS = 0
for _name, _flag in _REQUIRED_ESTIMATOR_FLAG_SPECS:
    REQUIRED_ESTIMATOR_FLAGS |= _mavlink_flag(_name, _flag)

ESTIMATOR_FORBIDDEN_FLAGS = 0
for _name, _flag in _FORBIDDEN_ESTIMATOR_FLAG_SPECS:
    ESTIMATOR_FORBIDDEN_FLAGS |= _mavlink_flag(_name, _flag)


class ShutdownRequested(Exception):
    """Raised by the SIGTERM handler so launch shutdown exits quietly."""


def send_heartbeat(mav: Any) -> None:
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GENERIC,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def wait_for_mavlink(mav: Any) -> None:
    mavlink_url = _px4_mavlink_url()
    print(
        f"PX4 post-start setup: waiting for PX4 MAVLink on {mavlink_url}",
        flush=True,
    )
    deadline = time.monotonic() + PX4_CONNECT_TIMEOUT_SEC
    next_status = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0) is not None:
            send_heartbeat(mav)
            print("PX4 post-start setup: connected to PX4 MAVLink", flush=True)
            return
        now = time.monotonic()
        if now >= next_status:
            remaining = max(0.0, deadline - now)
            print(
                f"PX4 post-start setup: still waiting for PX4 MAVLink ({remaining:.0f}s left)",
                flush=True,
            )
            next_status = now + 10.0

    raise RuntimeError(f"Failed to connect to PX4 MAVLink on {mavlink_url} " f"within {PX4_CONNECT_TIMEOUT_SEC:.0f}s")


def _px4_mavlink_url() -> str:
    return os.environ.get("ACESIM_PX4_MAVLINK_URL", PX4_MAVLINK_URL)


def _format_estimator_flags(mask: int) -> str:
    names = [name for name, flag in _ESTIMATOR_FLAG_NAMES if mask & flag]
    known_mask = 0
    for _name, flag in _ESTIMATOR_FLAG_NAMES:
        known_mask |= flag
    unknown_mask = mask & ~known_mask
    if unknown_mask:
        names.append(hex(unknown_mask))
    return ", ".join(names) if names else "none"


def _check_finite_field(message: Any, message_name: str, field_name: str, failures: list[str]) -> None:
    value = getattr(message, field_name, None)
    if value is None:
        failures.append(f"{message_name} {field_name}={value!r} is not finite")
        return
    try:
        numeric_value = float(value)
    except (TypeError, ValueError):
        failures.append(f"{message_name} {field_name}={value!r} is not finite")
        return
    if not math.isfinite(numeric_value):
        failures.append(f"{message_name} {field_name}={value!r} is not finite")


def _check_ratio_field(
    message: Any,
    field_name: str,
    label: str,
    failures: list[str],
    threshold: float = PREARM_INNOVATION_RATIO_THRESHOLD,
    required: bool = False,
) -> None:
    value = getattr(message, field_name, None)
    if value is None:
        if required:
            failures.append(f"ESTIMATOR_STATUS {label} {field_name}={value!r} is not finite")
        return
    try:
        numeric_value = float(value)
    except (TypeError, ValueError):
        if required:
            failures.append(f"ESTIMATOR_STATUS {label} {field_name}={value!r} is not finite")
        return
    if not math.isfinite(numeric_value):
        if required:
            failures.append(
                f"ESTIMATOR_STATUS {label} {field_name}={numeric_value!r}, " f"expected finite value < {threshold:g}"
            )
        return
    if numeric_value >= threshold:
        failures.append(
            f"ESTIMATOR_STATUS {label} {field_name}={numeric_value!r}, " f"expected finite value < {threshold:g}"
        )


def _check_message_age(message_type: str, message_ages_sec: dict[str, float] | None, failures: list[str]) -> None:
    if message_ages_sec is None or message_type not in message_ages_sec:
        return
    age_sec = message_ages_sec[message_type]
    if age_sec > READINESS_STALE_SEC:
        failures.append(f"{message_type} is stale ({age_sec:.1f}s old)")


def _global_position_degrees(global_position: Any) -> tuple[float | None, float | None]:
    lat = getattr(global_position, "lat", None)
    lon = getattr(global_position, "lon", None)
    if lat is None or lon is None:
        return None, None
    try:
        return float(lat) / 1e7, float(lon) / 1e7
    except (TypeError, ValueError):
        return None, None


def _origin_degrees(origin: Any) -> tuple[float | None, float | None]:
    lat = getattr(origin, "latitude", None)
    lon = getattr(origin, "longitude", None)
    if lat is None or lon is None:
        return None, None
    try:
        return float(lat) / 1e7, float(lon) / 1e7
    except (TypeError, ValueError):
        return None, None


def _origin_altitude_m(origin: Any) -> float | None:
    altitude = getattr(origin, "altitude", None)
    if altitude is None:
        return None
    try:
        return float(altitude) / 1000.0
    except (TypeError, ValueError):
        return None


def ekf_origin_failures(
    gps_global_origin: Any | None,
    gps_home_lat: float,
    gps_home_lon: float,
    gps_alt_start: float,
    home_tolerance_deg: float = HOME_ORIGIN_TOLERANCE_DEG,
    alt_tolerance_m: float = HOME_ALT_TOLERANCE_M,
) -> list[str]:
    if gps_global_origin is None:
        return ["GPS_GLOBAL_ORIGIN not received"]

    lat, lon = _origin_degrees(gps_global_origin)
    alt = _origin_altitude_m(gps_global_origin)
    if (
        lat is None
        or lon is None
        or alt is None
        or not math.isfinite(lat)
        or not math.isfinite(lon)
        or not math.isfinite(alt)
    ):
        return ["GPS_GLOBAL_ORIGIN lat/lon/alt are not finite"]
    if (
        abs(lat - gps_home_lat) > home_tolerance_deg
        or abs(lon - gps_home_lon) > home_tolerance_deg
        or abs(alt - gps_alt_start) > alt_tolerance_m
    ):
        return [
            "GPS_GLOBAL_ORIGIN mismatch: "
            f"lat={lat:.7f} lon={lon:.7f} alt={alt:.3f}, expected within "
            f"{home_tolerance_deg:g} deg and {alt_tolerance_m:g} m of "
            f"lat={gps_home_lat:.7f} lon={gps_home_lon:.7f} alt={gps_alt_start:.3f}"
        ]
    return []


def readiness_failures(
    local_position: Any | None,
    global_position: Any | None,
    estimator_status: Any | None,
    gps_home_lat: float,
    gps_home_lon: float,
    *,
    sys_status: Any | None = None,
    message_ages_sec: dict[str, float] | None = None,
    home_tolerance_deg: float = HOME_ORIGIN_TOLERANCE_DEG,
) -> list[str]:
    failures: list[str] = []
    if local_position is None:
        failures.append("LOCAL_POSITION_NED not received")
    else:
        _check_message_age("LOCAL_POSITION_NED", message_ages_sec, failures)
        for field_name in ("x", "y", "z", "vx", "vy", "vz"):
            _check_finite_field(local_position, "LOCAL_POSITION_NED", field_name, failures)

    if global_position is None:
        failures.append("GLOBAL_POSITION_INT not received")
    else:
        _check_message_age("GLOBAL_POSITION_INT", message_ages_sec, failures)
        lat, lon = _global_position_degrees(global_position)
        if lat is None or lon is None or not math.isfinite(lat) or not math.isfinite(lon):
            failures.append("GLOBAL_POSITION_INT lat/lon are not finite")
        elif lat == 0.0 and lon == 0.0:
            failures.append("GLOBAL_POSITION_INT lat/lon are zero")
        elif abs(lat - gps_home_lat) > home_tolerance_deg or abs(lon - gps_home_lon) > home_tolerance_deg:
            failures.append(
                "GLOBAL_POSITION_INT origin mismatch: "
                f"lat={lat:.7f} lon={lon:.7f}, expected within "
                f"{home_tolerance_deg:g} deg of lat={gps_home_lat:.7f} lon={gps_home_lon:.7f}"
            )
        for field_name in ("alt", "relative_alt"):
            if hasattr(global_position, field_name):
                _check_finite_field(global_position, "GLOBAL_POSITION_INT", field_name, failures)

    if estimator_status is None:
        failures.append("ESTIMATOR_STATUS not received")
    else:
        _check_message_age("ESTIMATOR_STATUS", message_ages_sec, failures)
        flags_value = getattr(estimator_status, "flags", None)
        if flags_value is None:
            failures.append(f"ESTIMATOR_STATUS flags={flags_value!r} is invalid")
        else:
            try:
                flags = int(flags_value)
            except (TypeError, ValueError):
                failures.append(f"ESTIMATOR_STATUS flags={flags_value!r} is invalid")
            else:
                missing = REQUIRED_ESTIMATOR_FLAGS & ~flags
                if missing:
                    failures.append(f"ESTIMATOR_STATUS missing required flags: {_format_estimator_flags(missing)}")
                forbidden = flags & ESTIMATOR_FORBIDDEN_FLAGS
                if forbidden:
                    failures.append(f"ESTIMATOR_STATUS has forbidden flags: {_format_estimator_flags(forbidden)}")
        _check_ratio_field(estimator_status, "mag_ratio", "heading innovation", failures, required=True)
        for field_name, label in (
            ("vel_ratio", "velocity innovation"),
            ("pos_horiz_ratio", "horizontal position innovation"),
            ("pos_vert_ratio", "vertical position innovation"),
        ):
            _check_ratio_field(estimator_status, field_name, label, failures)

    if sys_status is None:
        failures.append("SYS_STATUS not received")
    else:
        _check_message_age("SYS_STATUS", message_ages_sec, failures)
        health_value = getattr(sys_status, "onboard_control_sensors_health", None)
        if health_value is None:
            failures.append(f"SYS_STATUS onboard_control_sensors_health={health_value!r} is invalid")
        else:
            try:
                health = int(health_value)
            except (TypeError, ValueError):
                failures.append(f"SYS_STATUS onboard_control_sensors_health={health_value!r} is invalid")
            else:
                if not health & MAV_SYS_STATUS_PREARM_CHECK:
                    failures.append("SYS_STATUS missing MAV_SYS_STATUS_PREARM_CHECK bit")

    return failures


def _target_system(mav: Any) -> int:
    return int(getattr(mav, "target_system", 0) or 1)


def _target_component(mav: Any) -> int:
    return int(getattr(mav, "target_component", 0) or 1)


def _latlon_to_mavlink(value: float) -> int:
    return int(round(value * 1e7))


def _alt_to_mavlink(value: float) -> int:
    return int(round(value * 1000.0))


def request_message_interval(mav: Any, message_id: int, rate_hz: float = READINESS_MESSAGE_INTERVAL_HZ) -> None:
    interval_us = int(1_000_000 / rate_hz)
    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def request_readiness_streams(mav: Any) -> None:
    for message_id in (
        SYS_STATUS_MSG_ID,
        LOCAL_POSITION_NED_MSG_ID,
        GLOBAL_POSITION_INT_MSG_ID,
        GPS_GLOBAL_ORIGIN_MSG_ID,
        ESTIMATOR_STATUS_MSG_ID,
    ):
        request_message_interval(mav, message_id)
        time.sleep(0.02)
    send_heartbeat(mav)


def send_ekf_origin(
    mav: Any,
    gps_home_lat: float,
    gps_home_lon: float,
    gps_alt_start: float,
    timeout_sec: float = 5.0,
) -> None:
    latitude = _latlon_to_mavlink(gps_home_lat)
    longitude = _latlon_to_mavlink(gps_home_lon)
    altitude = _alt_to_mavlink(gps_alt_start)
    mav.mav.set_gps_global_origin_send(_target_system(mav), latitude, longitude, altitude)

    deadline = time.monotonic() + timeout_sec
    last_failures = ["GPS_GLOBAL_ORIGIN not received"]
    while time.monotonic() < deadline:
        message = mav.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=0.2)
        failures = ekf_origin_failures(message, gps_home_lat, gps_home_lon, gps_alt_start)
        if not failures:
            return
        last_failures = failures
        time.sleep(0.1)

    raise RuntimeError(
        "PX4 EKF origin was not confirmed before timeout.\n"
        "Unmet EKF origin checks:\n" + "\n".join(f"- {failure}" for failure in last_failures)
    )


def _mavlink_message_type(message: Any) -> str:
    get_type = getattr(message, "get_type", None)
    if callable(get_type):
        return str(get_type())
    return type(message).__name__.upper()


def wait_for_estimator_ready(
    mav: Any,
    gps_home_lat: float,
    gps_home_lon: float,
    timeout_sec: float = 45.0,
) -> None:
    request_readiness_streams(mav)
    next_stream_request = time.monotonic() + 5.0
    deadline = time.monotonic() + timeout_sec
    local_position: Any | None = None
    global_position: Any | None = None
    estimator_status: Any | None = None
    sys_status: Any | None = None
    message_seen_at: dict[str, float] = {}
    stable_since: float | None = None
    last_failures = ["no MAVLink readiness samples received yet"]
    recent_preflight_status: list[str] = []
    next_status = time.monotonic() + 5.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_stream_request:
            request_readiness_streams(mav)
            next_stream_request = now + 5.0

        if now >= next_status:
            print(
                "PX4 post-start setup: waiting for MAVLink readiness: " + "; ".join(last_failures[:4]),
                flush=True,
            )
            next_status = now + 5.0

        message = mav.recv_match(
            type=READINESS_MESSAGE_TYPES,
            blocking=True,
            timeout=0.2,
        )
        if message is None:
            failures = readiness_failures(
                local_position,
                global_position,
                estimator_status,
                gps_home_lat,
                gps_home_lon,
                sys_status=sys_status,
                message_ages_sec={name: now - seen_at for name, seen_at in message_seen_at.items()},
            )
            if failures:
                stable_since = None
                last_failures = failures
            continue

        message_type = _mavlink_message_type(message)
        if message_type == "LOCAL_POSITION_NED":
            local_position = message
            message_seen_at[message_type] = now
        elif message_type == "GLOBAL_POSITION_INT":
            global_position = message
            message_seen_at[message_type] = now
        elif message_type == "ESTIMATOR_STATUS":
            estimator_status = message
            message_seen_at[message_type] = now
        elif message_type == "SYS_STATUS":
            sys_status = message
            message_seen_at[message_type] = now
        elif message_type == "STATUSTEXT":
            text = str(getattr(message, "text", "")).strip()
            if "Preflight Fail" in text:
                recent_preflight_status.append(text)
                del recent_preflight_status[:-MAX_RECENT_PREFLIGHT_STATUSTEXTS]
        else:
            continue

        failures = readiness_failures(
            local_position,
            global_position,
            estimator_status,
            gps_home_lat,
            gps_home_lon,
            sys_status=sys_status,
            message_ages_sec={name: now - seen_at for name, seen_at in message_seen_at.items()},
        )
        if failures:
            stable_since = None
            last_failures = failures
            continue

        if stable_since is None:
            stable_since = now
        if now - stable_since >= REQUIRED_READY_STABLE_SEC:
            print(
                "PX4 estimator ready for arming: MAVLink local/global position, estimator ratios, "
                "and prearm checks are stable",
                flush=True,
            )
            return

    recent_status = ""
    if recent_preflight_status:
        recent_status = "\nRecent PX4 preflight status text:\n" + "\n".join(
            f"- {text}" for text in recent_preflight_status
        )
    diagnostics = _format_readiness_diagnostics(estimator_status, sys_status)
    raise RuntimeError(
        "PX4 estimator did not become ready for arming before timeout.\n"
        "Unmet readiness checks:\n"
        + "\n".join(f"- {failure}" for failure in last_failures)
        + diagnostics
        + recent_status
    )


def wait_for_estimator_ready_quietly(
    mav: Any,
    gps_home_lat: float,
    gps_home_lon: float,
    timeout_sec: float = 45.0,
) -> bool:
    request_readiness_streams(mav)
    next_stream_request = time.monotonic() + 5.0
    deadline = time.monotonic() + timeout_sec
    local_position: Any | None = None
    global_position: Any | None = None
    estimator_status: Any | None = None
    sys_status: Any | None = None
    message_seen_at: dict[str, float] = {}
    stable_since: float | None = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_stream_request:
            request_readiness_streams(mav)
            next_stream_request = now + 5.0

        message = mav.recv_match(
            type=READINESS_MESSAGE_TYPES,
            blocking=True,
            timeout=0.2,
        )
        if message is None:
            failures = readiness_failures(
                local_position,
                global_position,
                estimator_status,
                gps_home_lat,
                gps_home_lon,
                sys_status=sys_status,
                message_ages_sec={name: now - seen_at for name, seen_at in message_seen_at.items()},
            )
            if failures:
                stable_since = None
            continue

        message_type = _mavlink_message_type(message)
        if message_type == "LOCAL_POSITION_NED":
            local_position = message
            message_seen_at[message_type] = now
        elif message_type == "GLOBAL_POSITION_INT":
            global_position = message
            message_seen_at[message_type] = now
        elif message_type == "ESTIMATOR_STATUS":
            estimator_status = message
            message_seen_at[message_type] = now
        elif message_type == "SYS_STATUS":
            sys_status = message
            message_seen_at[message_type] = now
        else:
            continue

        failures = readiness_failures(
            local_position,
            global_position,
            estimator_status,
            gps_home_lat,
            gps_home_lon,
            sys_status=sys_status,
            message_ages_sec={name: now - seen_at for name, seen_at in message_seen_at.items()},
        )
        if failures:
            stable_since = None
            continue

        if stable_since is None:
            stable_since = now
        if now - stable_since >= REQUIRED_READY_STABLE_SEC:
            print("PX4 background readiness diagnostic: estimator is ready", flush=True)
            return True
    print("PX4 background readiness diagnostic: estimator was not ready before timeout", flush=True)
    return False


def _format_readiness_diagnostics(estimator_status: Any | None, sys_status: Any | None) -> str:
    lines = ["", "Last MAVLink readiness diagnostics:"]
    if estimator_status is None:
        lines.append("- ESTIMATOR_STATUS: not received")
    else:
        for field_name in ("mag_ratio", "vel_ratio", "pos_horiz_ratio", "pos_vert_ratio"):
            lines.append(f"- ESTIMATOR_STATUS {field_name}: {getattr(estimator_status, field_name, None)!r}")

    if sys_status is None:
        lines.append("- SYS_STATUS: not received")
    else:
        health_value = getattr(sys_status, "onboard_control_sensors_health", None)
        if health_value is None:
            has_prearm_bit = False
        else:
            try:
                has_prearm_bit = bool(int(health_value) & MAV_SYS_STATUS_PREARM_CHECK)
            except (TypeError, ValueError):
                has_prearm_bit = False
        lines.append(f"- SYS_STATUS MAV_SYS_STATUS_PREARM_CHECK: {has_prearm_bit}")
    return "\n" + "\n".join(lines)


def _send_arm_command(mav: Any, arm: bool) -> None:
    mav.mav.command_long_send(
        _target_system(mav),
        _target_component(mav),
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if arm else 0.0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def _wait_for_arm_ack(mav: Any, arm: bool, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    action = "arm" if arm else "disarm"
    while time.monotonic() < deadline:
        message = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
        if message is None:
            continue
        command = getattr(message, "command", None)
        if command is None:
            continue
        if int(command) != MAV_CMD_COMPONENT_ARM_DISARM:
            continue
        result = int(getattr(message, "result", -1))
        if result != MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"PX4 {action} command rejected with MAV_RESULT {result}")
        return
    raise RuntimeError(f"Timed out waiting for PX4 {action} command ACK")


def _heartbeat_armed(message: Any) -> bool:
    motors_armed = getattr(message, "motors_armed", None)
    if callable(motors_armed):
        return bool(motors_armed())
    return bool(int(getattr(message, "base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED)


def _wait_for_armed_state(mav: Any, armed: bool, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        message = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.2)
        if message is not None and _heartbeat_armed(message) is armed:
            return
    state = "armed" if armed else "disarmed"
    raise RuntimeError(f"Timed out waiting for PX4 heartbeat to report {state}")


def verify_armable(mav: Any, arm_timeout_sec: float = 5.0, disarm_timeout_sec: float = 5.0) -> None:
    _send_arm_command(mav, True)
    _wait_for_arm_ack(mav, True, arm_timeout_sec)
    _wait_for_armed_state(mav, True, arm_timeout_sec)
    _send_arm_command(mav, False)
    _wait_for_arm_ack(mav, False, disarm_timeout_sec)
    _wait_for_armed_state(mav, False, disarm_timeout_sec)
    print("PX4 armability verified: arm/disarm succeeded", flush=True)


def _should_retry_armability_failure(exc: RuntimeError) -> bool:
    message = str(exc)
    return (
        "arm command rejected" in message
        or "Timed out waiting for PX4 arm command ACK" in message
        or "Timed out waiting for PX4 heartbeat to report armed" in message
    )


def _should_retry_readiness_failure(exc: RuntimeError) -> bool:
    return "PX4 estimator did not become ready for arming before timeout" in str(exc)


def wait_for_mocap_armability(mav: Any, gps_home_lat: float, gps_home_lon: float) -> None:
    verify_armable_enabled = os.environ.get("ACESIM_PX4_VERIFY_ARMABLE", "1") != "0"
    deadline = time.monotonic() + ARMABILITY_TOTAL_TIMEOUT_SEC
    last_armability_error: RuntimeError | None = None
    last_readiness_error: RuntimeError | None = None

    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            wait_for_estimator_ready(mav, gps_home_lat, gps_home_lon, timeout_sec=min(45.0, remaining))
        except RuntimeError as exc:
            if not _should_retry_readiness_failure(exc):
                raise
            last_readiness_error = exc
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            print(
                "PX4 post-start setup: estimator readiness did not stay stable yet; "
                f"waiting for PX4 to settle and retrying. Last error: {exc}",
                flush=True,
            )
            time.sleep(min(ARMABILITY_RETRY_DELAY_SEC, remaining))
            continue

        if not verify_armable_enabled:
            return

        try:
            verify_armable(mav)
            return
        except RuntimeError as exc:
            if not _should_retry_armability_failure(exc):
                raise
            last_armability_error = exc
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            print(
                "PX4 post-start setup: armability check did not pass after estimator readiness; "
                f"waiting for PX4 to settle and retrying. Last error: {exc}",
                flush=True,
            )
            time.sleep(min(ARMABILITY_RETRY_DELAY_SEC, remaining))

    message = "PX4 did not become armable before timeout"
    if last_armability_error is not None:
        message += f". Last armability error: {last_armability_error}"
    if last_readiness_error is not None:
        message += f". Last readiness error: {last_readiness_error}"
    raise RuntimeError(message)


def wait_for_px4_ready(
    mav: Any, fusion_mode: str, gps_home_lat: float, gps_home_lon: float, gps_alt_start: float
) -> None:
    run_strict_px4_readiness(mav, fusion_mode, gps_home_lat, gps_home_lon, gps_alt_start)


def run_required_px4_setup(
    mav: Any,
    fusion_mode: str,
    gps_home_lat: float,
    gps_home_lon: float,
    gps_alt_start: float,
) -> None:
    if fusion_mode != "mocap":
        return
    request_message_interval(mav, GPS_GLOBAL_ORIGIN_MSG_ID)
    send_heartbeat(mav)
    print(
        "PX4 post-start setup: setting EKF origin to " f"lat={gps_home_lat} lon={gps_home_lon} alt={gps_alt_start}",
        flush=True,
    )
    send_ekf_origin(mav, gps_home_lat, gps_home_lon, gps_alt_start)
    print("PX4 post-start setup: EKF origin command sent", flush=True)


def run_background_readiness_diagnostics(
    mav: Any,
    fusion_mode: str,
    gps_home_lat: float,
    gps_home_lon: float,
    gps_alt_start: float,
) -> None:
    del gps_alt_start
    if fusion_mode != "mocap":
        return
    try:
        wait_for_estimator_ready_quietly(mav, gps_home_lat, gps_home_lon)
    except Exception as exc:
        print(f"PX4 background readiness diagnostic failed: {exc}", flush=True)


def run_strict_px4_readiness(
    mav: Any,
    fusion_mode: str,
    gps_home_lat: float,
    gps_home_lon: float,
    gps_alt_start: float,
) -> None:
    run_required_px4_setup(mav, fusion_mode, gps_home_lat, gps_home_lon, gps_alt_start)
    if fusion_mode != "mocap":
        return
    wait_for_mocap_armability(mav, gps_home_lat, gps_home_lon)
    ready_context = os.environ.get("ACESIM_PX4_READY_CONTEXT", "").strip()
    if ready_context and os.environ.get("ACESIM_PX4_VERIFY_ARMABLE", "1") != "0":
        print(f"PX4 estimator and armability verified for {ready_context}", flush=True)


def run_post_start_setup(argv: list[str]) -> None:
    if len(argv) != 4:
        raise RuntimeError(
            "Usage: python -m acesim.sitl.readiness " "<fusion_mode> <gps_home_lat> <gps_home_lon> <gps_alt_start>"
        )

    fusion_mode = argv[0]
    gps_home_lat = float(argv[1])
    gps_home_lon = float(argv[2])
    gps_alt_start = float(argv[3])

    mav = mavutil.mavlink_connection(
        _px4_mavlink_url(),
        source_system=250,
        source_component=190,
        autoreconnect=True,
    )
    wait_for_mavlink(mav)
    wait_for_px4_ready(mav, fusion_mode, gps_home_lat, gps_home_lon, gps_alt_start)


def _request_shutdown(_signum: int, _frame: object) -> None:
    raise ShutdownRequested()


def main(argv: list[str] | None = None) -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    args = sys.argv[1:] if argv is None else argv
    try:
        run_post_start_setup(args)
    except (KeyboardInterrupt, ShutdownRequested):
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
