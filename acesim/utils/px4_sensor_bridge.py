"""PX4 HIL sensor bridge shared by the MuJoCo and Genesis backends."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from acesim.utils.frame import world_nwu_to_ned
from acesim.utils.px4_interface import PX4Interface, PX4SensorParams
from acesim.utils.simulation_clock_manager import SimulationClockManager


@dataclass(frozen=True)
class PX4SensorSample:
    """One backend sensor sample expressed in the bridge's canonical frames.

    `accel_frd`, `gyro_frd`, and `mag_frd` are already converted into PX4's
    body frame convention. `position_world_m` and `velocity_world_mps` stay in
    the simulator world frame, which is NWU in this codebase.
    """

    accel_frd: np.ndarray
    gyro_frd: np.ndarray
    mag_frd: np.ndarray
    position_world_m: np.ndarray
    velocity_world_mps: np.ndarray


class PX4SensorBridge:
    """Schedules HIL sensor/GPS updates and forwards them to PX4.

    The bridge owns periodic timing, sample latching, and unit conversion. The
    backend callback only needs to provide one canonical sample expressed in the
    frames documented by :class:`PX4SensorSample`.
    """

    def __init__(
        self,
        px4_interface: PX4Interface,
        clock: SimulationClockManager,
        params: PX4SensorParams,
        read_sensor_sample: Callable[[], PX4SensorSample],
        reset_sensor_state: Callable[[], None] | None = None,
    ) -> None:
        self._px4_interface = px4_interface
        self._clock = clock
        self._params = params
        self._read_sensor_sample = read_sensor_sample
        self._reset_sensor_state = reset_sensor_state
        self.reset()

    def reset(self) -> None:
        """Reset periodic timers and latch the most recent backend sample.

        Latching the latest sample lets slower streams such as barometer and
        magnetometer reuse a stable value until their own update periods elapse.
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
        self._last_accel_frd = np.zeros(3, dtype=float)
        self._last_gyro_frd = np.zeros(3, dtype=float)
        self._last_mag_frd = np.zeros(3, dtype=float)
        self._last_baro_altitude_m = float(self._params.gps_alt_start)

        try:
            sample = self._read_sensor_sample()
        except Exception:
            sample = None

        if sample is not None:
            self._last_accel_frd = np.asarray(sample.accel_frd, dtype=float).copy()
            self._last_gyro_frd = np.asarray(sample.gyro_frd, dtype=float).copy()
            self._last_mag_frd = np.asarray(sample.mag_frd, dtype=float).copy()
            self._last_baro_altitude_m = float(self._params.gps_alt_start + sample.position_world_m[2])

    @staticmethod
    def _step_period_elapsed(elapsed_s: float, dt_s: float, period_s: float) -> tuple[bool, float]:
        """Advance one periodic timer and report whether it fired.

        The remainder is kept so that fractional simulated time carries across
        future updates instead of being quantized away.
        """

        elapsed_s += dt_s
        triggered = False
        while elapsed_s + 1e-12 >= period_s:
            elapsed_s -= period_s
            triggered = True
        return triggered, elapsed_s

    def _consume_period(self, elapsed_attr: str, dt_s: float, period_s: float) -> bool:
        """Consume elapsed simulated time for one named periodic stream."""

        tick, elapsed_s = self._step_period_elapsed(getattr(self, elapsed_attr), dt_s, period_s)
        setattr(self, elapsed_attr, elapsed_s)
        return tick

    def update(self) -> bool:
        """Push any HIL sensor or GPS messages that are due at the current sim time.

        Returns whether a HIL_SENSOR packet was emitted. Genesis uses that
        signal to keep actuator consumption aligned with sensor publication.
        """

        current_time_us = self._clock.current_time_us
        dt_s = max(0.0, (current_time_us - self._last_update_time_us) * 1e-6)
        self._last_update_time_us = current_time_us

        sample = self._read_sensor_sample()
        fields = self._px4_interface.HIL_SENSOR_FIELDS_ACCEL | self._px4_interface.HIL_SENSOR_FIELDS_GYRO

        if self._consume_period("_mag_elapsed_s", dt_s, self._mag_period_s):
            self._last_mag_frd = np.asarray(sample.mag_frd, dtype=float) + np.random.normal(
                0.0,
                self._params.mag_noise_std_gauss,
                size=3,
            )
            fields |= self._px4_interface.HIL_SENSOR_FIELDS_MAG

        if self._consume_period("_baro_elapsed_s", dt_s, self._baro_period_s):
            self._last_baro_altitude_m = float(
                self._params.gps_alt_start
                + sample.position_world_m[2]
                + np.random.normal(0.0, self._params.baro_noise_std_m)
            )
            fields |= self._px4_interface.HIL_SENSOR_FIELDS_BARO

        sensor_sent = False
        if self._consume_period("_hil_sensor_elapsed_s", dt_s, self._hil_sensor_period_s):
            self._last_accel_frd = np.asarray(sample.accel_frd, dtype=float) + np.random.normal(
                0.0,
                self._params.accel_noise_std_mps2,
            )
            self._last_gyro_frd = np.asarray(sample.gyro_frd, dtype=float) + np.random.normal(
                0.0,
                self._params.gyro_noise_std_radps,
                size=3,
            )
            # PX4 accepts partial HIL sensor refreshes, so the bridge can keep
            # slower mag/baro samples latched until their own update periods fire.
            self._px4_interface.send_hil_sensor(
                current_time_us,
                self._last_accel_frd,
                self._last_gyro_frd,
                self._last_mag_frd,
                self._last_baro_altitude_m,
                fields_updated=fields if self._params.dynamic_hil_sensor_fields else None,
            )
            sensor_sent = True

        if self._consume_period("_gps_elapsed_s", dt_s, self._gps_period_s):
            pos_noisy = np.asarray(sample.position_world_m, dtype=float) + np.random.normal(
                0.0,
                self._params.gps_pos_noise_std_m,
                size=3,
            )
            lat = self._params.gps_home_lat_lon[0] + (pos_noisy[0] / 111319.9)
            lon = self._params.gps_home_lat_lon[1] - (
                pos_noisy[1] / (111319.9 * np.cos(np.radians(self._params.gps_home_lat_lon[0])))
            )
            gps_alt = self._params.gps_alt_start + pos_noisy[2]

            vel_w = np.asarray(sample.velocity_world_mps, dtype=float) + np.random.normal(
                0.0,
                self._params.gps_vel_noise_std_mps,
                size=3,
            )
            # Backend kinematics are NWU, while PX4 expects GPS velocity in NED
            # centimeters per second.
            vel_ned = world_nwu_to_ned(vel_w)
            vn = vel_ned[0] * 100.0
            ve = vel_ned[1] * 100.0
            vd = vel_ned[2] * 100.0
            vel = float(np.linalg.norm([vn, ve, vd]))
            cog_rad = np.arctan2(ve, vn)
            cog_deg = (np.degrees(cog_rad) + 360.0) % 360.0

            self._px4_interface.send_hil_gps(
                current_time_us,
                int(lat * 1e7),
                int(lon * 1e7),
                int(gps_alt * 1000),
                int(vel),
                int(vn),
                int(ve),
                int(vd),
                int(cog_deg * 100.0),
            )

        return sensor_sent
