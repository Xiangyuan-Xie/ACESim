"""Minimal PX4 MAVLink transport shared by the simulator backends.

The current ACESim usage pattern is simple: create one MAVLink endpoint, wait
for PX4 HEARTBEAT packets, send HIL sensor/GPS data once connected, and read
the latest actuator frame on the main simulation thread. This module does not
try to hide malformed input or support multiple transport strategies.
"""

import threading
from dataclasses import dataclass
from typing import Any, Sequence, TypeAlias

import numpy as np
from numpy.typing import NDArray
from pymavlink import mavutil

FloatArray: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True)
class PX4ActuatorParams:
    """Timing parameters for delayed actuator command delivery."""

    motor_cmd_rate_hz: float
    motor_exec_delay_ms_range: tuple[float, float]

    def __post_init__(self) -> None:
        if self.motor_cmd_rate_hz <= 0.0:
            raise ValueError("motor_cmd_rate_hz must be positive")
        if len(self.motor_exec_delay_ms_range) != 2:
            raise ValueError("motor_exec_delay_ms_range must contain exactly two values")
        delay_min_ms = float(self.motor_exec_delay_ms_range[0])
        delay_max_ms = float(self.motor_exec_delay_ms_range[1])
        if delay_min_ms < 0.0 or delay_max_ms < 0.0:
            raise ValueError("motor_exec_delay_ms_range must be non-negative")
        if delay_min_ms > delay_max_ms:
            raise ValueError("motor_exec_delay_ms_range must be ordered as (min, max)")

    @classmethod
    def zero_disturbance(cls, motor_cmd_rate_hz: float) -> "PX4ActuatorParams":
        """Return a transport model with no extra execution delay."""

        return cls(
            motor_cmd_rate_hz=float(motor_cmd_rate_hz),
            motor_exec_delay_ms_range=(0.0, 0.0),
        )


@dataclass(frozen=True)
class PX4SensorParams:
    """Sampling, noise, and visualization parameters for PX4 HIL streams."""

    idle_visual_speed: float = 55.0
    gps_home_lat_lon: tuple[float, float] = (39.98329, 116.34745)
    gps_alt_start: float = 50.0
    hil_sensor_rate_hz: float = 250.0
    mag_rate_hz: float = 100.0
    baro_rate_hz: float = 50.0
    gps_rate_hz: float = 30.0
    accel_noise_std_mps2: tuple[float, float, float] = (0.00637, 0.00637, 0.00686)
    gyro_noise_std_radps: float = 0.0008726646
    mag_noise_std_gauss: float = 0.003
    baro_noise_std_m: float = 0.25
    gps_pos_noise_std_m: float = 0.01
    gps_vel_noise_std_mps: float = 0.1
    dynamic_hil_sensor_fields: bool = False


class PX4Transport:
    """MAVLink transport used by the MuJoCo and Genesis PX4 integrations.

    The transport intentionally follows one direct path:
    1. bind one local MAVLink TCP server endpoint for PX4 SITL;
    2. wait until PX4 sends HEARTBEAT;
    3. exchange HIL sensor/GPS and actuator messages on the main thread.
    """

    HIL_SENSOR_FIELDS_ACCEL = 0b0000000000111
    HIL_SENSOR_FIELDS_GYRO = 0b0000000111000
    HIL_SENSOR_FIELDS_MAG = 0b0001110000000
    HIL_SENSOR_FIELDS_BARO = 0b1101000000000

    def __init__(
        self,
        actuator_params: PX4ActuatorParams,
        port: int = 4560,
    ) -> None:
        """Create the transport and bind the local MAVLink TCP endpoint immediately."""

        self._port: int = port
        self._actuator_params = actuator_params
        self._io_lock = threading.Lock()
        self._mavlink_connection: Any = mavutil.mavlink_connection(
            f"tcpin:0.0.0.0:{self._port}", source_system=254, source_component=97
        )
        self._is_connected: bool = False
        self._is_armed: bool = False
        self._pending_actuator_commands: list[tuple[int, FloatArray]] = []
        self._applied_actuator_controls: FloatArray | None = None

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
        """Reset the delayed actuator queue and the last applied command."""

        self._pending_actuator_commands = []
        self._applied_actuator_controls = None

    def _sample_delay_us(self) -> int:
        """Sample one command execution delay in microseconds.

        Each actuator frame gets its own independent delay sample. There is no
        stateful delay model anymore, only one uniform draw inside the
        configured millisecond range.
        """

        delay_min_ms, delay_max_ms = self._actuator_params.motor_exec_delay_ms_range
        if delay_max_ms == delay_min_ms:
            return int(round(delay_min_ms * 1000.0))
        return int(round(np.random.uniform(delay_min_ms, delay_max_ms) * 1000.0))

    def _enqueue_actuator_command(self, normalized_controls: FloatArray, sim_time_us: int) -> None:
        """Queue one normalized actuator command for future release."""

        delay_us = self._sample_delay_us()
        release_time_us = sim_time_us + delay_us
        self._pending_actuator_commands.append((release_time_us, normalized_controls.copy()))

    def _apply_due_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        """Apply the newest queued command whose release time has passed."""

        if not self._pending_actuator_commands:
            return

        remaining_commands: list[tuple[int, FloatArray]] = []
        latest_due_cmd: FloatArray | None = None
        latest_due_release = -1
        for release_time_us, command in self._pending_actuator_commands:
            if release_time_us <= sim_time_us:
                if release_time_us >= latest_due_release:
                    latest_due_release = release_time_us
                    latest_due_cmd = command
            else:
                remaining_commands.append((release_time_us, command))

        if latest_due_cmd is not None:
            applied = np.zeros(channel_count, dtype=float)
            copy_count = min(channel_count, latest_due_cmd.size)
            applied[:copy_count] = latest_due_cmd[:copy_count]
            self._applied_actuator_controls = applied

        self._pending_actuator_commands = remaining_commands

    def _read_latest_actuator_controls(self) -> Sequence[float] | None:
        """Drain queued HIL_ACTUATOR_CONTROLS frames and return the latest one.

        The simulator consumes actuator commands from the main loop instead of a
        background listener thread so command release stays aligned with
        simulation steps and the delayed-actuation queue. Only the newest frame
        matters to that loop, so older queued frames are discarded immediately.
        """

        if not self._is_connected:
            raise RuntimeError("PX4 actuator controls requested before HEARTBEAT connection")

        with self._io_lock:
            msg = self._mavlink_connection.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
            if not msg:
                return None
            latest_controls = msg.controls
            while True:
                msg = self._mavlink_connection.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
                if not msg:
                    break
                latest_controls = msg.controls
        return latest_controls

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        """Advance the actuator timing model and expose any command due now."""

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
            if np.any((normalized[:channel_count] < 0.0) | (normalized[:channel_count] > 1.0)):
                raise ValueError("HIL_ACTUATOR_CONTROLS must stay within [0, 1]")
            self._enqueue_actuator_command(normalized[:channel_count], sim_time_us)

        self._apply_due_actuator_commands(sim_time_us, channel_count)

    def read_applied_actuator_controls(self, channel_count: int) -> FloatArray | None:
        """Return the currently released normalized actuator controls.

        ``None`` means no delayed actuator frame has reached its release time
        yet. Callers decide how to keep their previous control targets in that
        case.
        """

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

        with self._io_lock:
            msg = self._mavlink_connection.recv_match(type="HEARTBEAT", blocking=False)
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
        temperature_celsius: float = 25.0,
        fields_updated: int | None = None,
    ) -> None:
        """Send one HIL_SENSOR sample after PX4 connection is established."""

        if not self._is_connected:
            raise RuntimeError("send_hil_sensor called before HEARTBEAT connection")

        if fields_updated is None:
            fields_updated = 0x1BFF

        abs_pressure = 1013.25 * (1 - 2.25577e-5 * altitude_m) ** 5.25588

        with self._io_lock:
            self._mavlink_connection.mav.hil_sensor_send(
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
                0,
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

        with self._io_lock:
            self._mavlink_connection.mav.hil_gps_send(
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

    def update_arming_state(self) -> bool:
        """Query PX4 motor arming state from the active MAVLink connection."""

        if not self._is_connected:
            return False
        with self._io_lock:
            self._is_armed = bool(self._mavlink_connection.motors_armed())
        return self._is_armed

    def close(self) -> None:
        """Close the MAVLink connection."""

        connection = self._mavlink_connection
        self._mavlink_connection = None
        if connection is not None:
            connection.close()
