"""PX4 HIL sensor scheduler shared by the MuJoCo and Genesis backends.

This scheduler owns the timing for each outgoing PX4 sensor stream. The
backend only needs to provide one canonical sensor sample in ACESim's internal
frames. The scheduler latches the most recent values that belong to slower
streams such as magnetometer, barometer, and GPS, then republishes them at
their own rates.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, TypeAlias

import numpy as np
from numpy.typing import NDArray

from acesim.utils.frame import world_nwu_to_ned
from acesim.utils.px4_transport import PX4SensorParams, PX4Transport
from acesim.utils.simulation_clock import SimulationClock

FloatArray: TypeAlias = NDArray[np.float64]


@dataclass(frozen=True)
class PX4SensorSample:
    """One backend sensor sample expressed in the scheduler's canonical frames.

    The body-frame values are already converted into PX4's FRD convention.
    Position and velocity remain in the simulator world frame, which is NWU in
    this codebase and gets converted only where PX4 requires NED.
    """

    accel_frd: FloatArray
    gyro_frd: FloatArray
    mag_frd: FloatArray
    position_world_m: FloatArray
    velocity_world_mps: FloatArray


class PX4SensorScheduler:
    """Publish PX4 HIL sensor and GPS streams from simulation time.

    The scheduler is intentionally specific to the current ACESim use case:
    simulated time drives four fixed-rate streams and each update reads one
    backend sample. There is no generic scheduler or best-effort fallback when
    the backend sample provider fails.
    """

    def __init__(
        self,
        px4_transport: PX4Transport,
        clock: SimulationClock,
        params: PX4SensorParams,
        read_sensor_sample: Callable[[], PX4SensorSample],
        reset_sensor_state: Callable[[], None] | None = None,
    ) -> None:
        self._px4_transport: PX4Transport = px4_transport
        self._clock: SimulationClock = clock
        self._params: PX4SensorParams = params
        self._read_sensor_sample: Callable[[], PX4SensorSample] = read_sensor_sample
        self._reset_sensor_state: Callable[[], None] | None = reset_sensor_state

        if self._params.hil_sensor_rate_hz <= 0.0:
            raise ValueError("hil_sensor_rate_hz must be positive")
        if self._params.mag_rate_hz <= 0.0:
            raise ValueError("mag_rate_hz must be positive")
        if self._params.baro_rate_hz <= 0.0:
            raise ValueError("baro_rate_hz must be positive")
        if self._params.gps_rate_hz <= 0.0:
            raise ValueError("gps_rate_hz must be positive")

        self._last_update_time_us: int = 0
        self._hil_sensor_period_s: float = 0.0
        self._mag_period_s: float = 0.0
        self._baro_period_s: float = 0.0
        self._gps_period_s: float = 0.0
        self._hil_sensor_elapsed_s: float = 0.0
        self._mag_elapsed_s: float = 0.0
        self._baro_elapsed_s: float = 0.0
        self._gps_elapsed_s: float = 0.0
        self._last_accel_frd: FloatArray = np.zeros(3, dtype=float)
        self._last_gyro_frd: FloatArray = np.zeros(3, dtype=float)
        self._last_mag_frd: FloatArray = np.zeros(3, dtype=float)
        self._last_baro_altitude_m: float = float(self._params.gps_alt_start)

        self.reset()

    def reset(self) -> None:
        """Reset timers and latch the current backend sample.

        Slower PX4 streams reuse the most recent values that were sampled for
        them, so reset() also seeds those latches from the current backend
        state instead of falling back to arbitrary defaults.
        """

        if self._reset_sensor_state is not None:
            self._reset_sensor_state()

        self._last_update_time_us = self._clock.current_time_us
        self._hil_sensor_period_s = 1.0 / self._params.hil_sensor_rate_hz
        self._mag_period_s = 1.0 / self._params.mag_rate_hz
        self._baro_period_s = 1.0 / self._params.baro_rate_hz
        self._gps_period_s = 1.0 / self._params.gps_rate_hz
        self._hil_sensor_elapsed_s = 0.0
        self._mag_elapsed_s = 0.0
        self._baro_elapsed_s = 0.0
        self._gps_elapsed_s = 0.0

        sample = self._read_sensor_sample()
        accel_frd = np.asarray(sample.accel_frd, dtype=float)
        gyro_frd = np.asarray(sample.gyro_frd, dtype=float)
        mag_frd = np.asarray(sample.mag_frd, dtype=float)
        position_world_m = np.asarray(sample.position_world_m, dtype=float)
        if accel_frd.ndim != 1:
            raise ValueError("accel_frd must be a flat 1D array")
        if gyro_frd.ndim != 1:
            raise ValueError("gyro_frd must be a flat 1D array")
        if mag_frd.ndim != 1:
            raise ValueError("mag_frd must be a flat 1D array")
        if position_world_m.ndim != 1:
            raise ValueError("position_world_m must be a flat 1D array")
        self._last_accel_frd = accel_frd.copy()
        self._last_gyro_frd = gyro_frd.copy()
        self._last_mag_frd = mag_frd.copy()
        self._last_baro_altitude_m = float(self._params.gps_alt_start + position_world_m[2])

    def update(self) -> bool:
        """Send any PX4 HIL messages due at the current simulation time.

        Returns whether a HIL_SENSOR packet was emitted during this call.
        Genesis uses that signal to align actuator consumption to sensor
        publication instead of reading actuator commands on every physics step.
        """

        current_time_us = self._clock.current_time_us
        dt_s = max(0.0, (current_time_us - self._last_update_time_us) * 1e-6)
        self._last_update_time_us = current_time_us

        sample = self._read_sensor_sample()
        accel_frd = np.asarray(sample.accel_frd, dtype=float)
        gyro_frd = np.asarray(sample.gyro_frd, dtype=float)
        mag_frd = np.asarray(sample.mag_frd, dtype=float)
        position_world_m = np.asarray(sample.position_world_m, dtype=float)
        velocity_world_mps = np.asarray(sample.velocity_world_mps, dtype=float)
        if accel_frd.ndim != 1:
            raise ValueError("accel_frd must be a flat 1D array")
        if gyro_frd.ndim != 1:
            raise ValueError("gyro_frd must be a flat 1D array")
        if mag_frd.ndim != 1:
            raise ValueError("mag_frd must be a flat 1D array")
        if position_world_m.ndim != 1:
            raise ValueError("position_world_m must be a flat 1D array")
        if velocity_world_mps.ndim != 1:
            raise ValueError("velocity_world_mps must be a flat 1D array")

        fields = self._px4_transport.HIL_SENSOR_FIELDS_ACCEL | self._px4_transport.HIL_SENSOR_FIELDS_GYRO

        self._mag_elapsed_s += dt_s
        mag_due = False
        while self._mag_elapsed_s + 1e-12 >= self._mag_period_s:
            self._mag_elapsed_s -= self._mag_period_s
            mag_due = True
        if mag_due:
            self._last_mag_frd = mag_frd + np.random.normal(0.0, self._params.mag_noise_std_gauss, size=3)
            fields |= self._px4_transport.HIL_SENSOR_FIELDS_MAG

        self._baro_elapsed_s += dt_s
        baro_due = False
        while self._baro_elapsed_s + 1e-12 >= self._baro_period_s:
            self._baro_elapsed_s -= self._baro_period_s
            baro_due = True
        if baro_due:
            self._last_baro_altitude_m = float(
                self._params.gps_alt_start + position_world_m[2] + np.random.normal(0.0, self._params.baro_noise_std_m)
            )
            fields |= self._px4_transport.HIL_SENSOR_FIELDS_BARO

        sensor_sent = False
        self._hil_sensor_elapsed_s += dt_s
        hil_due = False
        while self._hil_sensor_elapsed_s + 1e-12 >= self._hil_sensor_period_s:
            self._hil_sensor_elapsed_s -= self._hil_sensor_period_s
            hil_due = True
        if hil_due:
            self._last_accel_frd = accel_frd + np.random.normal(0.0, self._params.accel_noise_std_mps2)
            self._last_gyro_frd = gyro_frd + np.random.normal(0.0, self._params.gyro_noise_std_radps, size=3)
            self._px4_transport.send_hil_sensor(
                current_time_us,
                self._last_accel_frd,
                self._last_gyro_frd,
                self._last_mag_frd,
                self._last_baro_altitude_m,
                fields_updated=fields if self._params.dynamic_hil_sensor_fields else None,
            )
            sensor_sent = True

        self._gps_elapsed_s += dt_s
        gps_due = False
        while self._gps_elapsed_s + 1e-12 >= self._gps_period_s:
            self._gps_elapsed_s -= self._gps_period_s
            gps_due = True
        if gps_due:
            noisy_position_world_m = position_world_m + np.random.normal(0.0, self._params.gps_pos_noise_std_m, size=3)
            latitude_deg = self._params.gps_home_lat_lon[0] + (noisy_position_world_m[0] / 111319.9)
            longitude_deg = self._params.gps_home_lat_lon[1] - (
                noisy_position_world_m[1] / (111319.9 * np.cos(np.radians(self._params.gps_home_lat_lon[0])))
            )
            gps_altitude_m = self._params.gps_alt_start + noisy_position_world_m[2]

            noisy_velocity_world_mps = velocity_world_mps + np.random.normal(
                0.0,
                self._params.gps_vel_noise_std_mps,
                size=3,
            )
            velocity_ned_cm_s = world_nwu_to_ned(noisy_velocity_world_mps) * 100.0
            velocity_north_cm_s = float(velocity_ned_cm_s[0])
            velocity_east_cm_s = float(velocity_ned_cm_s[1])
            velocity_down_cm_s = float(velocity_ned_cm_s[2])
            ground_speed_cm_s = float(np.linalg.norm(velocity_ned_cm_s))
            course_over_ground_rad = np.arctan2(velocity_east_cm_s, velocity_north_cm_s)
            course_over_ground_deg = (np.degrees(course_over_ground_rad) + 360.0) % 360.0

            self._px4_transport.send_hil_gps(
                current_time_us,
                int(latitude_deg * 1e7),
                int(longitude_deg * 1e7),
                int(gps_altitude_m * 1000.0),
                int(ground_speed_cm_s),
                int(velocity_north_cm_s),
                int(velocity_east_cm_s),
                int(velocity_down_cm_s),
                int(course_over_ground_deg * 100.0),
            )

        return sensor_sent
