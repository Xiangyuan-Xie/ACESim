from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import TypedDict
from unittest.mock import MagicMock, patch

import numpy as np
import tomli
from scipy.spatial.transform import Rotation

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

    class _VisionCall(TypedDict):
        args: tuple[int, np.ndarray, np.ndarray]

    def __init__(self) -> None:
        self.hil_sensor_calls: list[_FakeTransport._HilSensorCall] = []
        self.hil_gps_calls: list[dict[str, object]] = []
        self.vision_calls: list[_FakeTransport._VisionCall] = []

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
        self.vision_calls.append(
            {
                "args": (
                    timestamp_us,
                    position_world_m,
                    attitude_world_quat,
                )
            }
        )


class PX4TransportSchedulerTests(unittest.TestCase):
    def test_mocap_default_vision_noise_keeps_configured_fusion_defaults(self) -> None:
        params = PX4SensorParams(fusion_mode="mocap")

        self.assertAlmostEqual(params.ekf2_evp_noise, 0.003)
        self.assertAlmostEqual(params.ekf2_eva_noise, 0.01)
        self.assertFalse(hasattr(params, "vision_position_variance_m2"))
        self.assertFalse(hasattr(params, "vision_orientation_variance_rad2"))

    def test_mocap_asset_configs_keep_declared_fusion_parameters(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        config_paths = [
            *sorted((repo_root / "acesim" / "config" / "mujoco").glob("*.toml")),
            repo_root / "acesim" / "config" / "genesis" / "x500_arm2x.toml",
        ]

        for config_path in config_paths:
            config = tomli.loads(config_path.read_text(encoding="utf-8"))
            mocap_config = config.get("params", {}).get("px4_fusion", {}).get("mocap")
            if mocap_config is None:
                continue

            with self.subTest(config=config_path.relative_to(repo_root)):
                self.assertEqual(float(mocap_config["ekf2_evp_noise"]), 0.003)
                self.assertEqual(float(mocap_config["ekf2_eva_noise"]), 0.01)
                self.assertEqual(int(mocap_config["ekf2_mag_type"]), 0)

    def _make_scheduler(
        self,
        *,
        diff_pressure_hpa: float | None,
        temperature_celsius: float = 25.0,
    ) -> tuple[SimulationClock, _FakeTransport, PX4SensorScheduler]:
        clock = SimulationClock()
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

    @patch("acesim.utils.px4_transport.mavutil.mavlink_connection")
    def test_vision_position_estimate_uses_px4_default_covariance_behavior(self, mock_connection: MagicMock) -> None:
        connection = _FakeConnection()
        mock_connection.return_value = connection
        transport = PX4Transport(PX4ActuatorParams())
        transport._is_connected = True

        transport.send_vision_position_estimate(
            12_345,
            np.array([1.0, 2.0, 3.0], dtype=float),
            np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        )

        call = connection.mav.vision_calls[0]
        self.assertEqual(len(call), 7)
        transport.close()

    def test_mocap_scheduler_does_not_override_vision_covariance(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=250.0,
            vision_rate_hz=100.0,
        )

        def read_sample() -> PX4SensorSample:
            return PX4SensorSample(
                accel_frd=np.array([0.0, 0.0, -9.81], dtype=float),
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        clock.advance_us(10_000)
        scheduler.update()

        self.assertEqual(len(transport.vision_calls), 1)
        call = transport.vision_calls[0]["args"]
        self.assertEqual(len(call), 3)
        clock.close()

    def test_mocap_scheduler_filters_single_large_yaw_outlier(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=250.0,
            vision_rate_hz=100.0,
        )
        quaternions = [
            Rotation.from_euler("z", angle_deg, degrees=True).as_quat(scalar_first=True)
            for angle_deg in (0.0, 82.7, 0.0)
        ]

        def read_sample() -> PX4SensorSample:
            quat = quaternions.pop(0) if quaternions else quaternions[-1]
            return PX4SensorSample(
                accel_frd=np.array([0.0, 0.0, -9.81], dtype=float),
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.asarray(quat, dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        clock.advance_us(10_000)
        scheduler.update()
        clock.advance_us(10_000)
        scheduler.update()

        self.assertEqual(len(transport.vision_calls), 1)
        sent_quat = transport.vision_calls[0]["args"][2]
        sent_yaw_deg = Rotation.from_quat(sent_quat, scalar_first=True).as_euler("zyx", degrees=True)[0]
        self.assertAlmostEqual(sent_yaw_deg, 0.0, places=3)
        clock.close()

    def test_mocap_scheduler_allows_continuous_small_yaw_changes(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=250.0,
            vision_rate_hz=100.0,
        )
        quaternions = [
            Rotation.from_euler("z", angle_deg, degrees=True).as_quat(scalar_first=True)
            for angle_deg in (0.0, 5.0, 10.0, 15.0)
        ]

        def read_sample() -> PX4SensorSample:
            quat = (
                quaternions.pop(0)
                if quaternions
                else Rotation.from_euler("z", 15.0, degrees=True).as_quat(scalar_first=True)
            )
            return PX4SensorSample(
                accel_frd=np.array([0.0, 0.0, -9.81], dtype=float),
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.array([0.0, 0.0, 1.0], dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.asarray(quat, dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        for _ in range(3):
            clock.advance_us(10_000)
            scheduler.update()

        sent_yaws = [
            Rotation.from_quat(call["args"][2], scalar_first=True).as_euler("zyx", degrees=True)[0]
            for call in transport.vision_calls
        ]
        np.testing.assert_allclose(sent_yaws, [5.0, 10.0, 15.0], atol=1e-6)
        clock.close()


if __name__ == "__main__":
    unittest.main()
