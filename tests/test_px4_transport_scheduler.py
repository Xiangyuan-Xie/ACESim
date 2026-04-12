from __future__ import annotations

import unittest
from types import SimpleNamespace
from typing import TypedDict
from unittest.mock import MagicMock, patch

import numpy as np

from acesim.utils.px4_sensor_scheduler import PX4SensorSample, PX4SensorScheduler
from acesim.utils.px4_transport import PX4ActuatorParams, PX4SensorParams, PX4Transport
from acesim.utils.simulation_clock import SimulationClock


class _FakeMav:
    def __init__(self) -> None:
        self.hil_sensor_calls: list[tuple[object, ...]] = []
        self.hil_gps_calls: list[tuple[object, ...]] = []
        self.vision_calls: list[tuple[object, ...]] = []

    def hil_sensor_send(self, *args: object) -> None:
        self.hil_sensor_calls.append(args)

    def hil_gps_send(self, *args: object) -> None:
        self.hil_gps_calls.append(args)

    def vision_position_estimate_send(self, *args: object) -> None:
        self.vision_calls.append(args)


class _FakeConnection:
    def __init__(self) -> None:
        self.mav = _FakeMav()
        self._queues: dict[str, list[object]] = {
            "HEARTBEAT": [],
            "HIL_ACTUATOR_CONTROLS": [],
        }

    def enqueue(self, msg_type: str, msg: object) -> None:
        self._queues.setdefault(msg_type, []).append(msg)

    def recv_match(self, type: str, blocking: bool = False) -> object | None:
        queue = self._queues.get(type, [])
        if queue:
            return queue.pop(0)
        return None

    def motors_armed(self) -> bool:
        return False

    def close(self) -> None:
        return None


class _FakeTransport:
    HIL_SENSOR_FIELDS_ACCEL = PX4Transport.HIL_SENSOR_FIELDS_ACCEL
    HIL_SENSOR_FIELDS_GYRO = PX4Transport.HIL_SENSOR_FIELDS_GYRO
    HIL_SENSOR_FIELDS_MAG = PX4Transport.HIL_SENSOR_FIELDS_MAG
    HIL_SENSOR_FIELDS_DIFF_PRESS = PX4Transport.HIL_SENSOR_FIELDS_DIFF_PRESS
    HIL_SENSOR_FIELDS_BARO = PX4Transport.HIL_SENSOR_FIELDS_BARO

    class _HilSensorCall(TypedDict):
        timestamp_us: int
        accel_frd: np.ndarray
        gyro_frd: np.ndarray
        mag_frd: np.ndarray
        altitude_m: float
        diff_pressure_hpa: float
        temperature_celsius: float
        fields_updated: int

    def __init__(self) -> None:
        self.hil_sensor_calls: list[_FakeTransport._HilSensorCall] = []
        self.hil_gps_calls: list[dict[str, object]] = []
        self.vision_calls: list[dict[str, object]] = []

    def send_hil_sensor(
        self,
        timestamp_us: int,
        accel_frd: np.ndarray,
        gyro_frd: np.ndarray,
        mag_frd: np.ndarray,
        altitude_m: float,
        diff_pressure_hpa: float = 0.0,
        temperature_celsius: float = 25.0,
        fields_updated: int = 0,
    ) -> None:
        self.hil_sensor_calls.append(
            {
                "timestamp_us": timestamp_us,
                "accel_frd": np.asarray(accel_frd, dtype=float),
                "gyro_frd": np.asarray(gyro_frd, dtype=float),
                "mag_frd": np.asarray(mag_frd, dtype=float),
                "altitude_m": float(altitude_m),
                "diff_pressure_hpa": float(diff_pressure_hpa),
                "temperature_celsius": float(temperature_celsius),
                "fields_updated": int(fields_updated),
            }
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
        self.hil_gps_calls.append(
            {
                "args": (
                    timestamp_us,
                    latitude_e7,
                    longitude_e7,
                    altitude_mm,
                    ground_speed_cm_s,
                    velocity_north_cm_s,
                    velocity_east_cm_s,
                    velocity_down_cm_s,
                    course_over_ground_cdeg,
                    satellites_visible,
                )
            }
        )

    def send_vision_position_estimate(
        self,
        timestamp_us: int,
        position_world_m: np.ndarray,
        attitude_world_quat: np.ndarray,
    ) -> None:
        self.vision_calls.append({"args": (timestamp_us, position_world_m, attitude_world_quat)})


class PX4TransportSchedulerTests(unittest.TestCase):
    def _make_scheduler(
        self,
        *,
        diff_pressure_hpa: float | None,
        temperature_celsius: float = 25.0,
    ) -> tuple[SimulationClock, _FakeTransport, PX4SensorScheduler]:
        clock = SimulationClock(enable_zmq=False)
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="hil",
            hil_sensor_rate_hz=250.0,
            mag_rate_hz=84.0,
            baro_rate_hz=42.0,
            gps_rate_hz=5.0,
        )

        def read_sample() -> PX4SensorSample:
            return PX4SensorSample(
                accel_frd=np.array([0.0, 0.0, -9.81], dtype=float),
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.array([0.2, 0.0, -0.4], dtype=float),
                position_world_m=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
                diff_pressure_hpa=diff_pressure_hpa,
                temperature_celsius=temperature_celsius,
            )

        return clock, transport, PX4SensorScheduler(transport, clock, params, read_sample)

    @patch("acesim.utils.px4_transport.mavutil.mavlink_connection")
    def test_signed_hil_actuator_controls_are_accepted(self, mock_connection: MagicMock) -> None:
        connection = _FakeConnection()
        mock_connection.return_value = connection
        transport = PX4Transport(PX4ActuatorParams())
        transport._is_connected = True
        connection.enqueue(
            "HIL_ACTUATOR_CONTROLS",
            SimpleNamespace(controls=np.array([-1.0, 0.25, 1.5], dtype=float)),
        )

        transport.update_actuator_commands(sim_time_us=10_000, channel_count=3)
        applied = transport.read_applied_actuator_controls(3)

        self.assertIsNotNone(applied)
        np.testing.assert_allclose(applied, np.array([-1.0, 0.25, 1.5], dtype=float))
        transport.close()

    def test_scheduler_sets_diff_pressure_field_when_sample_provides_it(self) -> None:
        clock, transport, scheduler = self._make_scheduler(diff_pressure_hpa=3.2, temperature_celsius=17.5)
        clock.advance_us(4_000)
        sent = scheduler.update()

        self.assertTrue(sent)
        self.assertEqual(len(transport.hil_sensor_calls), 1)
        sample = transport.hil_sensor_calls[0]
        diff_pressure_hpa = sample["diff_pressure_hpa"]
        temperature_celsius = sample["temperature_celsius"]
        fields_updated = sample["fields_updated"]
        self.assertAlmostEqual(diff_pressure_hpa, 3.2)
        self.assertAlmostEqual(temperature_celsius, 17.5)
        self.assertTrue(fields_updated & PX4Transport.HIL_SENSOR_FIELDS_DIFF_PRESS)
        clock.close()

    def test_scheduler_keeps_multirotor_sensor_fields_without_diff_pressure(self) -> None:
        clock, transport, scheduler = self._make_scheduler(diff_pressure_hpa=None)
        clock.advance_us(4_000)
        scheduler.update()

        sample = transport.hil_sensor_calls[0]
        fields_updated = sample["fields_updated"]
        self.assertFalse(fields_updated & PX4Transport.HIL_SENSOR_FIELDS_DIFF_PRESS)
        self.assertTrue(fields_updated & PX4Transport.HIL_SENSOR_FIELDS_ACCEL)
        self.assertTrue(fields_updated & PX4Transport.HIL_SENSOR_FIELDS_GYRO)
        clock.close()


if __name__ == "__main__":
    unittest.main()
