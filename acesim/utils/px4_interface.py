"""PX4 transport layer shared by the MuJoCo and Genesis backends.

The interface owns one MAVLink connection, a background listener thread for
actuator commands, and a small timing model that can inject delay and drops
before normalized controls are exposed to the simulator backends.
"""

import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
from pymavlink import mavutil


@dataclass(frozen=True)
class PX4ActuatorParams:
    """Timing and drop parameters for normalized actuator command delivery.

    These values belong to the PX4 transport layer rather than to any one
    backend because they shape how normalized PX4 actuator commands are delayed,
    dropped, and released into the simulator.
    """

    motor_cmd_rate_hz: float
    motor_exec_delay_steps_range: tuple[int, int]
    motor_exec_delay_update_steps_range: tuple[int, int]
    motor_exec_delay_transition_probs: tuple[float, float, float]
    motor_exec_delay_drop_prob: float

    @classmethod
    def zero_disturbance(cls, motor_cmd_rate_hz: float) -> "PX4ActuatorParams":
        """Return a transport model with no extra delay or packet drops."""

        return cls(
            motor_cmd_rate_hz=float(motor_cmd_rate_hz),
            motor_exec_delay_steps_range=(0, 0),
            motor_exec_delay_update_steps_range=(0, 0),
            motor_exec_delay_transition_probs=(0.0, 1.0, 0.0),
            motor_exec_delay_drop_prob=0.0,
        )


@dataclass(frozen=True)
class PX4SensorParams:
    """Sampling, GPS-origin, and visualization parameters for PX4 HIL sensors.

    The sensor bridge consumes these settings to schedule HIL sensor streams and
    to define the fixed geographic origin used by HIL_GPS.
    """

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


class PX4Interface:
    """Thread-safe PX4 MAVLink bridge used by all simulator backends."""

    HIL_SENSOR_FIELDS_ACCEL = 0b0000000000111
    HIL_SENSOR_FIELDS_GYRO = 0b0000000111000
    HIL_SENSOR_FIELDS_MAG = 0b0001110000000
    HIL_SENSOR_FIELDS_BARO = 0b1101000000000

    def __init__(
        self,
        actuator_params: PX4ActuatorParams,
        host: Optional[str] = None,
        port: int = 4560,
    ):
        """Create the PX4 bridge and start the background actuator listener."""

        env_host = os.environ.get("ACESIM_PX4_HOST")
        self._host = host or env_host or "0.0.0.0"
        self._port = port
        self._actuator_params = actuator_params
        self._io_lock = threading.Lock()
        self._listener_lock = threading.Lock()
        self._listener_stop_event = threading.Event()
        self._listener_thread = None
        self._mavlink_connection = None
        self._is_connected = False
        self._is_armed = False
        self._latest_actuator_controls = None
        self._latest_actuator_frame_count = 0
        self._latest_actuator_received_monotonic_s = 0.0
        self._latest_actuator_seq = 0
        self._last_delivered_actuator_seq = 0
        self._pending_actuator_commands: list[tuple[int, np.ndarray]] = []
        self._applied_actuator_controls = None
        self._delay_steps_current = self._sample_delay_steps()
        self._delay_hold_counter = self._sample_delay_hold_counter()

        self._initialize_connection()
        self._start_listener_thread()

    def _set_connected(self):
        """Mark the bridge as connected and emit the standard status banner."""

        if self._is_connected:
            return True
        self._is_connected = True
        print("-" * 50)
        print("[PX4 SITL] Connected!")
        print("[PX4 SITL] QGC should auto-connect via UDP 14550.")
        print("-" * 50)
        return True

    @property
    def is_connected(self):
        return self._is_connected

    @property
    def is_armed(self):
        return self._is_armed

    def _initialize_connection(self):
        """Open the listening MAVLink TCP endpoint if it is available."""

        try:
            with self._io_lock:
                self._mavlink_connection = mavutil.mavlink_connection(
                    f"tcpin:{self._host}:{self._port}", source_system=254, source_component=97
                )
        except OSError:
            self._mavlink_connection = None

    def _start_listener_thread(self):
        """Start the background thread that drains PX4 actuator controls."""

        with self._listener_lock:
            if self._listener_thread is not None and self._listener_thread.is_alive():
                return
            self._listener_stop_event.clear()
            self._listener_thread = threading.Thread(
                target=self._control_listener_loop,
                name="px4-control-listener",
                daemon=True,
            )
            self._listener_thread.start()

    def _stop_control_listener(self):
        """Stop the background actuator listener thread if it is running."""

        with self._listener_lock:
            self._listener_stop_event.set()
            listener_thread = self._listener_thread
            self._listener_thread = None
        if listener_thread is not None and listener_thread.is_alive():
            listener_thread.join(timeout=1.0)

    def _control_listener_loop(self):
        """Continuously cache the latest actuator frame once PX4 is connected."""

        while not self._listener_stop_event.is_set():
            if not self._is_connected:
                time.sleep(0.05)
                continue
            controls, frame_count = self.read_actuator_controls(include_metadata=True)
            if controls is None:
                time.sleep(0.002)
                continue
            with self._io_lock:
                self._latest_actuator_controls = tuple(controls)
                self._latest_actuator_frame_count = int(frame_count)
                self._latest_actuator_received_monotonic_s = time.monotonic()
                self._latest_actuator_seq += 1

    def _consume_latest_actuator_controls(self):
        """Return the newest unread actuator frame using latest-wins semantics."""

        with self._io_lock:
            if self._latest_actuator_seq == self._last_delivered_actuator_seq:
                return None
            self._last_delivered_actuator_seq = self._latest_actuator_seq
            controls = self._latest_actuator_controls
        return controls

    def clear_actuator_controls(self):
        """Reset cached actuator frames and timing-model state."""

        with self._io_lock:
            self._latest_actuator_controls = None
            self._latest_actuator_frame_count = 0
            self._latest_actuator_received_monotonic_s = 0.0
            self._latest_actuator_seq = 0
            self._last_delivered_actuator_seq = 0
            self._pending_actuator_commands = []
            self._applied_actuator_controls = None
            self._delay_steps_current = self._sample_delay_steps()
            self._delay_hold_counter = self._sample_delay_hold_counter()

    def _clip_delay_steps(self, delay_steps: int) -> int:
        """Clamp delay steps to the configured range."""

        return int(
            np.clip(
                delay_steps,
                self._actuator_params.motor_exec_delay_steps_range[0],
                self._actuator_params.motor_exec_delay_steps_range[1],
            )
        )

    def _sample_delay_steps(self) -> int:
        """Sample an initial transport delay state from the configured range."""

        delay_min, delay_max = self._actuator_params.motor_exec_delay_steps_range
        if delay_max <= delay_min:
            return int(delay_min)
        return int(np.random.randint(delay_min, delay_max + 1))

    def _sample_delay_hold_counter(self) -> int:
        """Sample how long the current delay state should be held."""

        update_min, update_max = self._actuator_params.motor_exec_delay_update_steps_range
        if update_max <= update_min:
            return int(max(update_min, 0))
        return int(np.random.randint(update_min, update_max + 1))

    def _update_delay_state(self) -> None:
        """Advance the delay-state Markov process when its hold time expires."""

        self._delay_hold_counter = max(0, self._delay_hold_counter - 1)
        if self._delay_hold_counter > 0:
            return

        p_minus, p_zero, _ = self._actuator_params.motor_exec_delay_transition_probs
        random_value = np.random.uniform(0.0, 1.0)
        if random_value < p_minus:
            delta = -1
        elif random_value < (p_minus + p_zero):
            delta = 0
        else:
            delta = 1

        self._delay_steps_current = self._clip_delay_steps(self._delay_steps_current + delta)
        self._delay_hold_counter = self._sample_delay_hold_counter()

    def _enqueue_actuator_command(self, normalized_controls: np.ndarray, sim_time_us: int) -> None:
        """Queue one normalized actuator command for future release."""

        delay_us = int(
            round(self._delay_steps_current * (1.0 / max(self._actuator_params.motor_cmd_rate_hz, 1e-6)) * 1e6)
        )
        release_time_us = sim_time_us + max(0, delay_us)
        self._pending_actuator_commands.append((release_time_us, normalized_controls.copy()))

    def _apply_due_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        """Apply the newest queued command whose release time has passed."""

        if not self._pending_actuator_commands:
            return

        remaining_commands: list[tuple[int, np.ndarray]] = []
        latest_due_cmd = None
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
            if copy_count > 0:
                applied[:copy_count] = latest_due_cmd[:copy_count]
            self._applied_actuator_controls = applied

        self._pending_actuator_commands = remaining_commands

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        """Advance the actuator timing model and expose any command due now."""

        if channel_count <= 0:
            return

        controls = self._consume_latest_actuator_controls()
        if controls is not None:
            try:
                normalized = np.asarray(controls[:channel_count], dtype=float)
            except (TypeError, ValueError):
                normalized = None
            if normalized is not None:
                normalized = np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0)
                normalized = np.clip(normalized, 0.0, 1.0)
                if np.random.uniform(0.0, 1.0) >= self._actuator_params.motor_exec_delay_drop_prob:
                    self._update_delay_state()
                    self._enqueue_actuator_command(normalized, sim_time_us)

        self._apply_due_actuator_commands(sim_time_us, channel_count)

    def read_applied_actuator_controls(self, channel_count: int):
        """Return the currently released normalized actuator controls."""

        if channel_count <= 0:
            return None
        with self._io_lock:
            controls = self._applied_actuator_controls
        if controls is None:
            return None
        applied = np.zeros(channel_count, dtype=float)
        copy_count = min(channel_count, controls.size)
        if copy_count > 0:
            applied[:copy_count] = controls[:copy_count]
        return applied

    def update_connection_state(self):
        """Update connection state from PX4 heartbeat."""
        if self._is_connected:
            return True

        if self._mavlink_connection is None:
            self._initialize_connection()
        if self._mavlink_connection:
            with self._io_lock:
                msg = self._mavlink_connection.recv_match(type="HEARTBEAT", blocking=False)
            if msg:
                return self._set_connected()

        return False

    def send_hil_sensor(
        self,
        timestamp_us,
        accel_frd,
        gyro_frd,
        mag_frd,
        altitude_m,
        temperature_celsius=25.0,
        fields_updated=None,
    ):
        """
        Send HIL_SENSOR message to PX4.

        :param timestamp_us: Timestamp in microseconds
        :param accel_frd: [ax, ay, az] in m/s^2 (body frame, FRD)
        :param gyro_frd: [gx, gy, gz] in rad/s (body frame, FRD)
        :param mag_frd: [mx, my, mz] in Gauss (body frame, FRD)
        :param altitude_m: Altitude in meters
        :param temperature_celsius: Temperature in Celsius
        :param fields_updated: Optional bitmask for updated fields (default: calculate based on inputs)
        """
        if not self._is_connected:
            return

        # Fields updated flags:
        # XACC(1) | YACC(2) | ZACC(4) = 7
        # XGYRO(8) | YGYRO(16) | ZGYRO(32) = 56
        # XMAG(64) | YMAG(128) | ZMAG(256) = 448
        # ABS_PRESSURE(512) | DIFF_PRESSURE(1024) | PRESSURE_ALT(2048) | TEMPERATURE(4096).

        if fields_updated is None:
            # Default: everything except DIFF_PRESSURE (1024).
            fields_updated = 0x1BFF

        # PX4 expects specific units:
        # Accel: m/s^2
        # Gyro: rad/s
        # Mag: Gauss
        # Abs pressure: hPa (millibar)
        # Pressure altitude: meters
        # Temperature: Celsius.

        # Calculate approximate pressure from altitude using a standard atmosphere model:
        # P = 1013.25 * (1 - 2.25577e-5 * h)^5.25588.
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
                0,  # diff_pressure (unused but set to 0)
                altitude_m,
                temperature_celsius,
                fields_updated,
                0,  # id
            )

    def send_hil_gps(
        self,
        timestamp_us,
        latitude_e7,
        longitude_e7,
        altitude_mm,
        ground_speed_cm_s,
        velocity_north_cm_s,
        velocity_east_cm_s,
        velocity_down_cm_s,
        course_over_ground_cdeg,
        satellites_visible=10,
    ):
        """
        Send HIL_GPS message to PX4.

        :param timestamp_us: Timestamp in microseconds
        :param latitude_e7: Latitude in degrees * 1E7
        :param longitude_e7: Longitude in degrees * 1E7
        :param altitude_mm: Altitude in millimeters (AMSL)
        :param ground_speed_cm_s: Speed in cm/s
        :param velocity_north_cm_s: Velocity North in cm/s
        :param velocity_east_cm_s: Velocity East in cm/s
        :param velocity_down_cm_s: Velocity Down in cm/s
        :param course_over_ground_cdeg: Course over ground in centidegrees
        """
        if not self._is_connected:
            return

        fix_type = 3  # 3D fix.
        eph = 100
        epv = 100

        with self._io_lock:
            self._mavlink_connection.mav.hil_gps_send(
                timestamp_us,
                fix_type,
                int(latitude_e7),
                int(longitude_e7),
                int(altitude_mm),
                eph,
                epv,
                int(ground_speed_cm_s),
                int(velocity_north_cm_s),
                int(velocity_east_cm_s),
                int(velocity_down_cm_s),
                int(course_over_ground_cdeg),
                satellites_visible,
            )

    def read_actuator_controls(
        self,
        blocking: bool = False,
        timeout_s: float = 0.0,
        include_metadata: bool = False,
    ):
        """Drain HIL_ACTUATOR_CONTROLS messages from MAVLink.

        The background listener uses this helper to collapse any backlog into a
        single latest frame, while callers can still request drain counts for
        diagnostics if needed.
        """
        if not self._is_connected:
            return (None, 0) if include_metadata else None

        frame_count = 0
        if self._mavlink_connection:
            with self._io_lock:
                if blocking:
                    msg = self._mavlink_connection.recv_match(
                        type="HIL_ACTUATOR_CONTROLS", blocking=True, timeout=max(timeout_s, 0.0)
                    )
                else:
                    msg = self._mavlink_connection.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
                if msg:
                    latest_controls = msg.controls
                    frame_count = 1
                    while True:
                        msg = self._mavlink_connection.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
                        if not msg:
                            break
                        latest_controls = msg.controls
                        frame_count += 1
                    return (latest_controls, frame_count) if include_metadata else latest_controls

        return (None, 0) if include_metadata else None

    def update_arming_state(self):
        """Update and return PX4 arming state."""
        if self._is_connected and self._mavlink_connection:
            with self._io_lock:
                self._is_armed = bool(self._mavlink_connection.motors_armed())

        return self._is_armed

    def close(self):
        """Stop the listener thread and close the MAVLink connection."""

        self._stop_control_listener()
        with self._io_lock:
            connection = self._mavlink_connection
            self._mavlink_connection = None
        if connection is not None:
            close_fn = getattr(connection, "close", None)
            if callable(close_fn):
                close_fn()
