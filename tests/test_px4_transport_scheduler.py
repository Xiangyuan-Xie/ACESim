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
        self.assertEqual(params.hil_sensor_delay_ms, (0.0, 0.0))
        self.assertEqual(params.vision_delay_ms, (0.0, 0.0))

    def test_px4_delay_ranges_are_parsed_from_asset_params(self) -> None:
        params = PX4SensorParams.from_asset_params(
            {
                "px4_fusion": {
                    "mode": "mocap",
                    "mocap": {"hil_sensor_rate_hz": 200.0, "vision_rate_hz": 100.0},
                    "delay": {
                        "hil_sensor_delay_ms": [0.076, 0.113],
                        "vision_delay_ms": [0.0, 0.0],
                        "actuator_delay_ms": [1.5, 2.5],
                    },
                }
            },
            dynamic_hil_sensor_fields=False,
        )
        actuator_params = PX4ActuatorParams.from_asset_params(
            {
                "px4_fusion": {
                    "delay": {
                        "actuator_delay_ms": [1.5, 2.5],
                    }
                }
            }
        )

        self.assertEqual(params.hil_sensor_delay_ms, (0.076, 0.113))
        self.assertEqual(params.vision_delay_ms, (0.0, 0.0))
        self.assertEqual(actuator_params.actuator_delay_ms, (1.5, 2.5))

    def test_delay_ranges_reject_invalid_bounds(self) -> None:
        with self.assertRaisesRegex(ValueError, "hil_sensor_delay_ms"):
            PX4SensorParams.from_asset_params(
                {"px4_fusion": {"mode": "mocap", "mocap": {}, "delay": {"hil_sensor_delay_ms": [2.0, 1.0]}}},
                dynamic_hil_sensor_fields=False,
            )
        with self.assertRaisesRegex(ValueError, "actuator_delay_ms"):
            PX4ActuatorParams.from_asset_params({"px4_fusion": {"delay": {"actuator_delay_ms": [-1.0, 1.0]}}})

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

    def test_x500_mujoco_configs_align_hil_imu_stream_to_px4_imu_integration_rate(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]

        for asset_name in ("x500", "x500_arm2x"):
            config_path = repo_root / "acesim" / "config" / "mujoco" / f"{asset_name}.toml"
            config = tomli.loads(config_path.read_text(encoding="utf-8"))
            px4_fusion = config["params"]["px4_fusion"]

            with self.subTest(asset=asset_name):
                self.assertEqual(float(px4_fusion["hil"]["hil_sensor_rate_hz"]), 200.0)
                self.assertEqual(float(px4_fusion["mocap"]["hil_sensor_rate_hz"]), 200.0)
                self.assertEqual(float(px4_fusion["mocap"]["vision_rate_hz"]), 100.0)
                self.assertEqual(float(px4_fusion["hil"]["mag_rate_hz"]), 84.0)
                self.assertEqual(float(px4_fusion["hil"]["baro_rate_hz"]), 42.0)
                self.assertEqual(float(px4_fusion["hil"]["gps_rate_hz"]), 5.0)

    def test_non_x500_mujoco_configs_keep_declared_hil_sensor_rates(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        expected_rates = {
            "standard_vtol": 250.0,
            "advanced_plane": 250.0,
            "uuv_bluerov2_heavy": 250.0,
        }

        for asset_name, expected_rate in expected_rates.items():
            config_path = repo_root / "acesim" / "config" / "mujoco" / f"{asset_name}.toml"
            config = tomli.loads(config_path.read_text(encoding="utf-8"))

            with self.subTest(asset=asset_name):
                self.assertEqual(
                    float(config["params"]["px4_fusion"]["hil"]["hil_sensor_rate_hz"]),
                    expected_rate,
                )

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

    def test_scheduler_uses_200hz_hil_sensor_boundary_without_early_publish(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=200.0,
            vision_rate_hz=100.0,
            accel_noise_std_mps2=(0.0, 0.0, 0.0),
            gyro_noise_std_radps=0.0,
        )

        def read_sample() -> PX4SensorSample:
            return PX4SensorSample(
                accel_frd=np.array([1.0, 2.0, 3.0], dtype=float),
                gyro_frd=np.array([0.1, 0.2, 0.3], dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.zeros(3, dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)

        clock.advance_us(4_999)
        self.assertFalse(scheduler.update())
        self.assertEqual(len(transport.hil_sensor_calls), 0)

        clock.advance_us(1)
        self.assertTrue(scheduler.update())
        self.assertEqual(transport.hil_sensor_calls[0]["timestamp_us"], 5_000)
        fields = transport.hil_sensor_calls[0]["fields_updated"]
        self.assertTrue(fields & PX4Transport.HIL_SENSOR_FIELDS_ACCEL)
        self.assertTrue(fields & PX4Transport.HIL_SENSOR_FIELDS_GYRO)
        self.assertFalse(fields & PX4Transport.HIL_SENSOR_FIELDS_MAG)
        self.assertFalse(fields & PX4Transport.HIL_SENSOR_FIELDS_BARO)
        self.assertFalse(transport.hil_gps_calls)
        clock.close()

    def test_scheduler_skips_sensor_sample_read_when_no_stream_is_due(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=200.0,
            vision_rate_hz=100.0,
            accel_noise_std_mps2=(0.0, 0.0, 0.0),
            gyro_noise_std_radps=0.0,
        )
        read_count = 0

        def read_sample() -> PX4SensorSample:
            nonlocal read_count
            read_count += 1
            return PX4SensorSample(
                accel_frd=np.array([1.0, 2.0, 3.0], dtype=float),
                gyro_frd=np.array([0.1, 0.2, 0.3], dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.zeros(3, dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        self.assertEqual(read_count, 1)

        clock.advance_us(1_000)
        self.assertFalse(scheduler.update())

        self.assertEqual(read_count, 1)
        self.assertFalse(transport.hil_sensor_calls)
        self.assertFalse(transport.vision_calls)
        clock.close()

    def test_scheduler_large_step_sends_single_current_hil_sample_without_backlog(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=200.0,
            vision_rate_hz=100.0,
            accel_noise_std_mps2=(0.0, 0.0, 0.0),
            gyro_noise_std_radps=0.0,
        )
        accel_samples = [
            np.array([1.0, 0.0, 0.0], dtype=float),
            np.array([2.0, 0.0, 0.0], dtype=float),
        ]

        def read_sample() -> PX4SensorSample:
            accel = accel_samples.pop(0) if accel_samples else np.array([9.0, 0.0, 0.0], dtype=float)
            return PX4SensorSample(
                accel_frd=accel,
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.zeros(3, dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        clock.advance_us(20_000)
        self.assertTrue(scheduler.update())

        self.assertEqual(len(transport.hil_sensor_calls), 1)
        self.assertEqual(transport.hil_sensor_calls[0]["timestamp_us"], 20_000)
        np.testing.assert_allclose(transport.hil_sensor_calls[0]["accel_frd"], np.array([2.0, 0.0, 0.0]))
        clock.close()

    def test_hil_sensor_delay_releases_sample_with_original_sample_timestamp(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=200.0,
            vision_rate_hz=100.0,
            accel_noise_std_mps2=(0.0, 0.0, 0.0),
            gyro_noise_std_radps=0.0,
            hil_sensor_delay_ms=(2.0, 2.0),
        )
        samples = [
            np.array([1.0, 0.0, 0.0], dtype=float),
            np.array([2.0, 0.0, 0.0], dtype=float),
        ]

        def read_sample() -> PX4SensorSample:
            accel = samples.pop(0) if samples else np.array([9.0, 0.0, 0.0], dtype=float)
            return PX4SensorSample(
                accel_frd=accel,
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.zeros(3, dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)

        clock.advance_us(5_000)
        self.assertFalse(scheduler.update())
        self.assertFalse(transport.hil_sensor_calls)

        clock.advance_us(1_999)
        self.assertFalse(scheduler.update())
        self.assertFalse(transport.hil_sensor_calls)

        clock.advance_us(1)
        self.assertTrue(scheduler.update())
        self.assertEqual(transport.hil_sensor_calls[0]["timestamp_us"], 5_000)
        np.testing.assert_allclose(transport.hil_sensor_calls[0]["accel_frd"], np.array([2.0, 0.0, 0.0]))
        clock.close()

    def test_hil_sensor_delay_keeps_unreleased_samples_when_delay_exceeds_period(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=200.0,
            vision_rate_hz=100.0,
            accel_noise_std_mps2=(0.0, 0.0, 0.0),
            gyro_noise_std_radps=0.0,
            hil_sensor_delay_ms=(7.0, 7.0),
        )
        next_sample = 0

        def read_sample() -> PX4SensorSample:
            nonlocal next_sample
            next_sample += 1
            return PX4SensorSample(
                accel_frd=np.array([float(next_sample), 0.0, 0.0], dtype=float),
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.zeros(3, dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        clock.advance_us(5_000)
        self.assertFalse(scheduler.update())
        clock.advance_us(5_000)
        self.assertFalse(scheduler.update())
        clock.advance_us(2_000)
        self.assertTrue(scheduler.update())

        self.assertEqual(transport.hil_sensor_calls[0]["timestamp_us"], 5_000)
        np.testing.assert_allclose(transport.hil_sensor_calls[0]["accel_frd"], np.array([2.0, 0.0, 0.0]))
        clock.close()

    def test_vision_delay_releases_pose_with_original_sample_timestamp(self) -> None:
        clock = SimulationClock()
        transport = _FakeTransport()
        params = PX4SensorParams(
            fusion_mode="mocap",
            hil_sensor_rate_hz=200.0,
            vision_rate_hz=100.0,
            vision_delay_ms=(3.0, 3.0),
        )

        def read_sample() -> PX4SensorSample:
            return PX4SensorSample(
                accel_frd=np.zeros(3, dtype=float),
                gyro_frd=np.zeros(3, dtype=float),
                mag_frd=np.zeros(3, dtype=float),
                position_world_m=np.array([1.0, 2.0, 3.0], dtype=float),
                velocity_world_mps=np.zeros(3, dtype=float),
                attitude_world_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
            )

        scheduler = PX4SensorScheduler(transport, clock, params, read_sample)
        clock.advance_us(10_000)
        scheduler.update()
        self.assertFalse(transport.vision_calls)

        clock.advance_us(3_000)
        scheduler.update()
        self.assertEqual(transport.vision_calls[0]["args"][0], 10_000)
        clock.close()

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

    @patch("acesim.utils.px4_transport.mavutil.mavlink_connection")
    def test_actuator_update_reports_whether_new_frame_was_received(self, mock_connection: MagicMock) -> None:
        connection = _FakeConnection()
        mock_connection.return_value = connection
        transport = PX4Transport(PX4ActuatorParams())
        transport._is_connected = True

        self.assertFalse(transport.update_actuator_commands(sim_time_us=5_000, channel_count=2))

        connection.enqueue(
            "HIL_ACTUATOR_CONTROLS",
            SimpleNamespace(controls=np.array([0.2, 0.7], dtype=float)),
        )
        self.assertTrue(transport.update_actuator_commands(sim_time_us=6_000, channel_count=2))
        np.testing.assert_allclose(transport.read_applied_actuator_controls(2), np.array([0.2, 0.7], dtype=float))
        transport.close()

    @patch("acesim.utils.px4_transport.mavutil.mavlink_connection")
    def test_actuator_delay_holds_frame_until_release_time(self, mock_connection: MagicMock) -> None:
        connection = _FakeConnection()
        mock_connection.return_value = connection
        transport = PX4Transport(PX4ActuatorParams(actuator_delay_ms=(3.0, 3.0)))
        transport._is_connected = True
        connection.enqueue(
            "HIL_ACTUATOR_CONTROLS",
            SimpleNamespace(controls=np.array([0.2, 0.7], dtype=float)),
        )

        self.assertFalse(transport.update_actuator_commands(sim_time_us=10_000, channel_count=2))
        self.assertIsNone(transport.read_applied_actuator_controls(2))
        self.assertFalse(transport.update_actuator_commands(sim_time_us=12_999, channel_count=2))
        self.assertTrue(transport.update_actuator_commands(sim_time_us=13_000, channel_count=2))
        np.testing.assert_allclose(transport.read_applied_actuator_controls(2), np.array([0.2, 0.7], dtype=float))
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

    @patch("acesim.utils.px4_transport.mavutil.mavlink_connection")
    def test_vision_position_estimate_converts_nwu_flu_attitude_to_ned_frd_euler(
        self, mock_connection: MagicMock
    ) -> None:
        connection = _FakeConnection()
        mock_connection.return_value = connection
        transport = PX4Transport(PX4ActuatorParams())
        transport._is_connected = True
        quat_nwu_flu = Rotation.from_euler("xyz", [10.0, -20.0, 30.0], degrees=True).as_quat(scalar_first=True)

        transport.send_vision_position_estimate(
            12_345,
            np.array([1.0, 2.0, 3.0], dtype=float),
            quat_nwu_flu,
        )

        _, north_m, east_m, down_m, roll, pitch, yaw = connection.mav.vision_calls[0]
        np.testing.assert_allclose([north_m, east_m, down_m], [1.0, -2.0, -3.0])
        expected_euler = Rotation.from_quat(
            [quat_nwu_flu[0], quat_nwu_flu[1], -quat_nwu_flu[2], -quat_nwu_flu[3]],
            scalar_first=True,
        ).as_euler("xyz", degrees=False)
        np.testing.assert_allclose([roll, pitch, yaw], expected_euler, atol=1e-12)
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
