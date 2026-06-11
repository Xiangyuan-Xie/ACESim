"""Minimal PX4 MAVLink transport shared by the simulator backends.

The current ACESim usage pattern is simple: create one MAVLink endpoint, wait
for PX4 HEARTBEAT packets, send HIL sensor/GPS data once connected, and read
the latest actuator frame on the main simulation thread. This module does not
try to hide malformed input or support multiple transport strategies.
"""

import os
import threading
from dataclasses import dataclass
from typing import Any, Mapping, Sequence, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pymavlink import mavutil

from acesim.config.asset_params import get_optional_table
from acesim.utils.delay import parse_delay_range_ms

FloatArray: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True)
class PX4ActuatorParams:
    """Actuator transport timing parameters."""

    actuator_delay_ms: tuple[float, float] = (0.0, 0.0)

    @classmethod
    def from_asset_params(cls, asset_params: Mapping[str, Any]) -> "PX4ActuatorParams":
        px4_fusion = get_optional_table(asset_params, "px4_fusion")
        delay_config = get_optional_table(px4_fusion, "delay")
        return cls(
            actuator_delay_ms=parse_delay_range_ms(
                delay_config.get("actuator_delay_ms", (0.0, 0.0)),
                "actuator_delay_ms",
            )
        )


@dataclass(frozen=True)
class PX4SensorParams:
    """Sampling and noise parameters for PX4 HIL streams."""

    gps_home_lat_lon: tuple[float, float] = (39.98329, 116.34745)
    gps_alt_start: float = 50.0
    fusion_mode: str = "hil"
    hil_sensor_rate_hz: float = 250.0
    mag_rate_hz: float = 100.0
    baro_rate_hz: float = 50.0
    gps_rate_hz: float = 30.0
    vision_rate_hz: float = 100.0
    accel_noise_std_mps2: tuple[float, float, float] = (0.00637, 0.00637, 0.00686)
    gyro_noise_std_radps: float = 0.0008726646
    mag_noise_std_gauss: float = 0.003
    baro_noise_std_m: float = 0.25
    gps_pos_noise_std_m: float = 0.01
    gps_vel_noise_std_mps: float = 0.1
    dynamic_hil_sensor_fields: bool = False
    ekf2_ev_ctrl: int = 11
    ekf2_hgt_ref: str = "Vision"
    ekf2_ev_delay_ms: float = 0.0
    ekf2_ev_pos_body_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ekf2_ev_noise_md: int = 1
    ekf2_evp_noise: float = 0.003
    ekf2_evv_noise: float = 0.01
    ekf2_eva_noise: float = 0.01
    ekf2_gps_ctrl: int = 0
    ekf2_mag_type: int = 0
    hil_sensor_delay_ms: tuple[float, float] = (0.0, 0.0)
    vision_delay_ms: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        if self.fusion_mode not in {"hil", "mocap"}:
            raise ValueError("fusion_mode must be 'hil' or 'mocap'")
        if self.hil_sensor_rate_hz <= 0.0:
            raise ValueError("hil_sensor_rate_hz must be positive")
        if self.send_mag and self.mag_rate_hz <= 0.0:
            raise ValueError("mag_rate_hz must be positive when magnetometer streaming is enabled")
        if self.send_baro and self.baro_rate_hz <= 0.0:
            raise ValueError("baro_rate_hz must be positive when barometer streaming is enabled")
        if self.send_gps and self.gps_rate_hz <= 0.0:
            raise ValueError("gps_rate_hz must be positive when GPS streaming is enabled")
        if self.send_vision and self.vision_rate_hz <= 0.0:
            raise ValueError("vision_rate_hz must be positive when vision streaming is enabled")
        if len(self.ekf2_ev_pos_body_m) != 3:
            raise ValueError("ekf2_ev_pos_body_m must contain exactly three values")
        parse_delay_range_ms(self.hil_sensor_delay_ms, "hil_sensor_delay_ms")
        parse_delay_range_ms(self.vision_delay_ms, "vision_delay_ms")

    @property
    def send_mag(self) -> bool:
        return self.fusion_mode == "hil"

    @property
    def send_baro(self) -> bool:
        return self.fusion_mode == "hil"

    @property
    def send_gps(self) -> bool:
        return self.fusion_mode == "hil"

    @property
    def send_vision(self) -> bool:
        return self.fusion_mode == "mocap"

    @staticmethod
    def _parse_vec3(value: object, field_name: str) -> tuple[float, float, float]:
        """Parse a generic config value into a strict 3D float tuple."""

        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
            raise ValueError(f"{field_name} must contain exactly three values")
        values = tuple(float(v) for v in value)
        if len(values) != 3:
            raise ValueError(f"{field_name} must contain exactly three values")
        return (values[0], values[1], values[2])

    @classmethod
    def from_asset_params(
        cls,
        asset_params: Mapping[str, Any],
        *,
        dynamic_hil_sensor_fields: bool,
    ) -> "PX4SensorParams":
        px4_fusion = asset_params.get("px4_fusion")
        gps_home_lat_lon = (
            float(asset_params.get("gps_lat_start", 39.98329)),
            float(asset_params.get("gps_lon_start", 116.34745)),
        )
        gps_alt_start = float(asset_params.get("gps_alt_start", 50.0))
        if not isinstance(px4_fusion, Mapping):
            return cls(
                gps_home_lat_lon=gps_home_lat_lon,
                gps_alt_start=gps_alt_start,
                dynamic_hil_sensor_fields=dynamic_hil_sensor_fields,
                fusion_mode="hil",
                hil_sensor_rate_hz=float(asset_params.get("hil_sensor_rate_hz", 250.0)),
                mag_rate_hz=float(asset_params.get("mag_rate_hz", 100.0)),
                baro_rate_hz=float(asset_params.get("baro_rate_hz", 50.0)),
                gps_rate_hz=float(asset_params.get("gps_rate_hz", 30.0)),
            )

        mode = str(px4_fusion.get("mode", "hil"))
        mode_config = get_optional_table(px4_fusion, mode)
        delay_config = get_optional_table(px4_fusion, "delay")
        hil_sensor_delay_ms = parse_delay_range_ms(
            delay_config.get("hil_sensor_delay_ms", (0.0, 0.0)),
            "hil_sensor_delay_ms",
        )
        vision_delay_ms = parse_delay_range_ms(
            delay_config.get("vision_delay_ms", (0.0, 0.0)),
            "vision_delay_ms",
        )

        if mode == "hil":
            return cls(
                gps_home_lat_lon=gps_home_lat_lon,
                gps_alt_start=gps_alt_start,
                dynamic_hil_sensor_fields=dynamic_hil_sensor_fields,
                fusion_mode=mode,
                hil_sensor_rate_hz=float(mode_config.get("hil_sensor_rate_hz", 250.0)),
                mag_rate_hz=float(mode_config.get("mag_rate_hz", 100.0)),
                baro_rate_hz=float(mode_config.get("baro_rate_hz", 50.0)),
                gps_rate_hz=float(mode_config.get("gps_rate_hz", 30.0)),
                hil_sensor_delay_ms=hil_sensor_delay_ms,
                vision_delay_ms=vision_delay_ms,
            )
        if mode == "mocap":
            ev_pos_body_m = cls._parse_vec3(
                mode_config.get("ekf2_ev_pos_body_m", (0.0, 0.0, 0.0)), "ekf2_ev_pos_body_m"
            )
            return cls(
                gps_home_lat_lon=gps_home_lat_lon,
                gps_alt_start=gps_alt_start,
                dynamic_hil_sensor_fields=dynamic_hil_sensor_fields,
                fusion_mode=mode,
                hil_sensor_rate_hz=float(mode_config.get("hil_sensor_rate_hz", 250.0)),
                mag_rate_hz=float(mode_config.get("mag_rate_hz", 100.0)),
                baro_rate_hz=float(mode_config.get("baro_rate_hz", 50.0)),
                vision_rate_hz=float(mode_config.get("vision_rate_hz", 100.0)),
                ekf2_ev_ctrl=int(mode_config.get("ekf2_ev_ctrl", 11)),
                ekf2_hgt_ref=str(mode_config.get("ekf2_hgt_ref", "Vision")),
                ekf2_ev_delay_ms=float(mode_config.get("ekf2_ev_delay_ms", 0.0)),
                ekf2_ev_pos_body_m=ev_pos_body_m,
                ekf2_ev_noise_md=int(mode_config.get("ekf2_ev_noise_md", 1)),
                ekf2_evp_noise=float(mode_config.get("ekf2_evp_noise", 0.003)),
                ekf2_evv_noise=float(mode_config.get("ekf2_evv_noise", 0.01)),
                ekf2_eva_noise=float(mode_config.get("ekf2_eva_noise", 0.01)),
                ekf2_gps_ctrl=int(mode_config.get("ekf2_gps_ctrl", 0)),
                ekf2_mag_type=int(mode_config.get("ekf2_mag_type", 0)),
                hil_sensor_delay_ms=hil_sensor_delay_ms,
                vision_delay_ms=vision_delay_ms,
            )
        raise ValueError(f"Unsupported px4_fusion mode: {mode}")


class PX4Transport:
    """MAVLink transport used by the MuJoCo and Genesis PX4 integrations.

    The transport intentionally follows one direct path:
    1. bind one local MAVLink TCP server endpoint for PX4 SITL;
    2. wait until PX4 sends HEARTBEAT;
    3. exchange HIL sensor/GPS and actuator messages on the main thread.
    """

    HIL_SENSOR_FIELDS_ACCEL = 0b0000000000111
    HIL_SENSOR_FIELDS_GYRO = 0b0000000111000
    HIL_SENSOR_FIELDS_MAG = 0b0000111000000
    HIL_SENSOR_FIELDS_DIFF_PRESS = 0b0010000000000
    HIL_SENSOR_FIELDS_BARO = 0b1101000000000

    def __init__(
        self,
        actuator_params: PX4ActuatorParams,
        port: int = 4560,
    ) -> None:
        """Create the transport and bind the local MAVLink TCP endpoint immediately."""

        self._port: int = int(os.environ.get("ACESIM_PX4_SIM_TCP_PORT", port))
        self._actuator_params = actuator_params
        self._hil_io_lock = threading.Lock()
        self._hil_mavlink_connection: Any = mavutil.mavlink_connection(
            f"tcpin:0.0.0.0:{self._port}", source_system=254, source_component=97
        )
        self._is_connected: bool = False
        self._is_armed: bool = False
        self._applied_actuator_controls: FloatArray | None = None
        self._pending_actuator_controls: list[tuple[int, FloatArray]] = []

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def _set_connected(self) -> bool:
        """Mark the transport as connected and emit the standard status banner."""

        if self._is_connected:
            return True
        self._is_connected = True
        print("-" * 50)
        print("[PX4 SITL] Connected!")
        print("[PX4 SITL] QGC should auto-connect via UDP 14550.")
        print("-" * 50)
        return True

    def clear_actuator_controls(self) -> None:
        """Reset the last applied command."""

        self._applied_actuator_controls = None
        self._pending_actuator_controls.clear()

    def _read_latest_actuator_controls(self) -> Sequence[float] | None:
        """Drain queued HIL_ACTUATOR_CONTROLS frames and return the latest one.

        The simulator consumes actuator commands from the main loop instead of a
        background listener thread so command release stays aligned with
        simulation steps and the delayed-actuation queue. Only the newest frame
        matters to that loop, so older queued frames are discarded immediately.
        """

        if not self._is_connected:
            raise RuntimeError("PX4 actuator controls requested before HEARTBEAT connection")

        with self._hil_io_lock:
            msg = self._hil_mavlink_connection.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
            if not msg:
                return None
            latest_controls = msg.controls
            while True:
                msg = self._hil_mavlink_connection.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
                if not msg:
                    break
                latest_controls = msg.controls
        return latest_controls

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> bool:
        """Drain the latest actuator frame and expose it once its release time arrives."""

        if channel_count <= 0:
            raise ValueError("channel_count must be positive")

        controls = self._read_latest_actuator_controls()
        if controls is not None:
            normalized: FloatArray = np.asarray(controls, dtype=float)
            if normalized.ndim != 1:
                raise ValueError("HIL_ACTUATOR_CONTROLS must be a flat 1D sequence")
            if normalized.size < channel_count:
                raise ValueError("HIL_ACTUATOR_CONTROLS does not provide enough channels")
            if not np.all(np.isfinite(normalized)):
                raise ValueError("HIL_ACTUATOR_CONTROLS contains non-finite values")
            applied = np.zeros(channel_count, dtype=float)
            applied[:channel_count] = normalized[:channel_count]
            delay_min_ms, delay_max_ms = self._actuator_params.actuator_delay_ms
            if delay_max_ms <= 0.0:
                self._applied_actuator_controls = applied
                return True
            delay_ms = (
                delay_min_ms if delay_min_ms == delay_max_ms else float(np.random.uniform(delay_min_ms, delay_max_ms))
            )
            release_time_us = int(round(sim_time_us + delay_ms * 1000.0))
            self._pending_actuator_controls.append((release_time_us, applied))

        due_controls = [item for item in self._pending_actuator_controls if item[0] <= sim_time_us]
        if due_controls:
            self._applied_actuator_controls = due_controls[-1][1]
            self._pending_actuator_controls = [
                item for item in self._pending_actuator_controls if item[0] > sim_time_us
            ]
            return True
        return False

    def read_applied_actuator_controls(self, channel_count: int) -> FloatArray | None:
        """Return the latest normalized actuator controls seen so far."""

        if channel_count <= 0:
            raise ValueError("channel_count must be positive")
        controls = self._applied_actuator_controls
        if controls is None:
            return None
        applied = np.zeros(channel_count, dtype=float)
        copy_count = min(channel_count, controls.size)
        applied[:copy_count] = controls[:copy_count]
        return applied

    def update_connection_state(self) -> bool:
        """Poll one PX4 HEARTBEAT and flip into the connected state if seen."""

        if self._is_connected:
            return True

        with self._hil_io_lock:
            msg = self._hil_mavlink_connection.recv_match(type="HEARTBEAT", blocking=False)
        if msg:
            return self._set_connected()
        return False

    def send_hil_sensor(
        self,
        timestamp_us: int,
        accel_frd: FloatArray,
        gyro_frd: FloatArray,
        mag_frd: FloatArray,
        altitude_m: float,
        diff_pressure_hpa: float = 0.0,
        temperature_celsius: float = 25.0,
        fields_updated: int = HIL_SENSOR_FIELDS_ACCEL | HIL_SENSOR_FIELDS_GYRO,
    ) -> None:
        """Send one HIL_SENSOR sample after PX4 connection is established."""

        if not self._is_connected:
            raise RuntimeError("send_hil_sensor called before HEARTBEAT connection")

        abs_pressure = 1013.25 * (1 - 2.25577e-5 * altitude_m) ** 5.25588

        with self._hil_io_lock:
            self._hil_mavlink_connection.mav.hil_sensor_send(
                timestamp_us,
                accel_frd[0],
                accel_frd[1],
                accel_frd[2],
                gyro_frd[0],
                gyro_frd[1],
                gyro_frd[2],
                mag_frd[0],
                mag_frd[1],
                mag_frd[2],
                abs_pressure,
                diff_pressure_hpa,
                altitude_m,
                temperature_celsius,
                fields_updated,
                0,
            )

    def send_hil_gps(
        self,
        timestamp_us: int,
        latitude_e7: int,
        longitude_e7: int,
        altitude_mm: int,
        ground_speed_cm_s: int,
        velocity_north_cm_s: int,
        velocity_east_cm_s: int,
        velocity_down_cm_s: int,
        course_over_ground_cdeg: int,
        satellites_visible: int = 10,
    ) -> None:
        """Send one HIL_GPS sample after PX4 connection is established."""

        if not self._is_connected:
            raise RuntimeError("send_hil_gps called before HEARTBEAT connection")

        with self._hil_io_lock:
            self._hil_mavlink_connection.mav.hil_gps_send(
                timestamp_us,
                3,
                int(latitude_e7),
                int(longitude_e7),
                int(altitude_mm),
                100,
                100,
                int(ground_speed_cm_s),
                int(velocity_north_cm_s),
                int(velocity_east_cm_s),
                int(velocity_down_cm_s),
                int(course_over_ground_cdeg),
                satellites_visible,
            )

    def send_vision_position_estimate(
        self,
        timestamp_us: int,
        position_world_m: FloatArray,
        attitude_world_quat: FloatArray,
    ) -> None:
        """Send one external-vision pose sample converted into PX4's local NED/FRD frames."""

        if not self._is_connected:
            raise RuntimeError("send_vision_position_estimate called before HEARTBEAT connection")

        position_world_m = np.asarray(position_world_m, dtype=float)
        position_ned = np.array([position_world_m[0], -position_world_m[1], -position_world_m[2]], dtype=float)
        attitude_quat = np.asarray(attitude_world_quat, dtype=float)
        if attitude_quat.shape != (4,):
            raise ValueError("attitude_world_quat must be a flat scalar-first quaternion")
        w = float(attitude_quat[0])
        x = float(attitude_quat[1])
        y = -float(attitude_quat[2])
        z = -float(attitude_quat[3])
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        with self._hil_io_lock:
            self._hil_mavlink_connection.mav.vision_position_estimate_send(
                timestamp_us,
                float(position_ned[0]),
                float(position_ned[1]),
                float(position_ned[2]),
                float(roll),
                float(pitch),
                float(yaw),
            )

    def update_arming_state(self) -> bool:
        """Query PX4 motor arming state from the active MAVLink connection."""

        if not self._is_connected:
            return False
        with self._hil_io_lock:
            self._is_armed = bool(self._hil_mavlink_connection.motors_armed())
        return self._is_armed

    def close(self) -> None:
        """Close the MAVLink connection."""

        hil_connection = self._hil_mavlink_connection
        self._hil_mavlink_connection = None
        if hil_connection is not None:
            hil_connection.close()
