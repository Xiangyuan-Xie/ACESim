from __future__ import annotations

import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Callable, Protocol, cast
from unittest.mock import patch

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv
from acesim.env.mujoco.mc_env import MCEnv
from acesim.env.mujoco.mj_env import MJEnv
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.env.mujoco.uuv_env import UUVEnv
from acesim.env.mujoco.vtol_env import VTOLEnv
from acesim.utils.dynamics import first_order_response_step
from acesim.utils.px4_sensor_scheduler import PX4SensorSample
from acesim.utils.simulation_clock import SimulationClock


class _FakePX4Transport:
    HIL_SENSOR_FIELDS_ACCEL = 0
    HIL_SENSOR_FIELDS_GYRO = 0
    HIL_SENSOR_FIELDS_MAG = 0
    HIL_SENSOR_FIELDS_DIFF_PRESS = 0
    HIL_SENSOR_FIELDS_BARO = 0

    def __init__(self, *args: object, **kwargs: object) -> None:
        self.is_connected = False

    def update_connection_state(self) -> bool:
        return False

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> bool:
        return False

    def read_applied_actuator_controls(self, channel_count: int) -> None:
        return None

    def update_arming_state(self) -> bool:
        return False

    def close(self) -> None:
        return None


class _ConnectedFakePX4Transport(_FakePX4Transport):
    events: list[str] = []

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.is_connected = True
        self.update_times: list[int] = []
        self.has_new_controls = True

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> bool:
        self.update_times.append(int(sim_time_us))
        self.__class__.events.append("actuator")
        return self.has_new_controls


class _ConnectsOnPostStepFakePX4Transport(_FakePX4Transport):
    events: list[str] = []

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.connection_polls = 0

    def update_connection_state(self) -> bool:
        self.connection_polls += 1
        if self.connection_polls == 1:
            self.__class__.events.append("miss")
            return False
        self.is_connected = True
        self.__class__.events.append("connect")
        return True


class _RecordingSensorScheduler:
    instances: list["_RecordingSensorScheduler"] = []

    def __init__(
        self,
        transport: object,
        clock: SimulationClock,
        params: object,
        read_sensor_sample: Callable[[], PX4SensorSample],
    ) -> None:
        self.clock = clock
        self.read_sensor_sample = read_sensor_sample
        self.calls: list[tuple[int, np.ndarray]] = []
        self.__class__.instances.append(self)

    def update(self) -> bool:
        sample = self.read_sensor_sample()
        self.calls.append((int(self.clock.current_time_us), np.asarray(sample.position_world_m, dtype=float)))
        _ConnectedFakePX4Transport.events.append("sensor")
        _ConnectsOnPostStepFakePX4Transport.events.append("sensor")
        return True


class _FakeVisualPublisher:
    def __init__(self, params: object) -> None:
        self.is_enabled = False

    def publish(self, state: object) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeClockPublisher:
    def __init__(self, *args: object, **kwargs: object) -> None:
        return None

    def publish(self, timestamp_us: int) -> None:
        return None

    def close(self) -> None:
        return None


class _MujocoModelLike(Protocol):
    sensor_adr: np.ndarray
    sensor_dim: np.ndarray


class _MujocoDataLike(Protocol):
    sensordata: np.ndarray


class _SupportsSensorSeeding(Protocol):
    _sensor_id_map: dict[str, int]
    _mj_model: _MujocoModelLike
    _mj_data: _MujocoDataLike


def _config_path(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "config" / f"{name}.toml").resolve()


def _write_mujoco_config(root: Path, *, env_type: str, asset_name: str) -> Path:
    config_path = root / "config.toml"
    asset_dir = root / "mujoco"
    asset_dir.mkdir(parents=True, exist_ok=True)
    asset_src = Path(__file__).resolve().parents[1] / "acesim" / "config" / "mujoco" / f"{asset_name}.toml"
    shutil.copy2(asset_src, asset_dir / f"{asset_name}.toml")
    config_path.write_text(
        "\n".join(
            [
                "[basic]",
                'sim_type = "mujoco"',
                f'env_type = "{env_type}"',
                'scene_name = "default"',
                f'asset_name = "{asset_name}"',
                'benchmark = "multirotor"',
                "",
            ]
        ),
        encoding="utf-8",
    )
    return config_path


def _set_sensor(env: _SupportsSensorSeeding, sensor_name: str, values: np.ndarray) -> None:
    sensor_id = env._sensor_id_map[sensor_name]
    adr = env._mj_model.sensor_adr[sensor_id]
    dim = env._mj_model.sensor_dim[sensor_id]
    env._mj_data.sensordata[adr : adr + dim] = np.asarray(values, dtype=float)


@patch("acesim.env.mujoco.px4_mj_env.VehicleVisualStatePublisher", _FakeVisualPublisher)
@patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _FakePX4Transport)
@patch("acesim.env.mujoco.mj_env.ClockPublisher", _FakeClockPublisher)
class MujocoVehicleDynamicsTests(unittest.TestCase):
    def _seed_kinematics(
        self, env: PX4MJEnv, pos: np.ndarray, linvel: np.ndarray, gyro: np.ndarray | None = None
    ) -> None:
        _set_sensor(env, "pos", np.asarray(pos, dtype=float))
        _set_sensor(env, "quat", np.array([1.0, 0.0, 0.0, 0.0], dtype=float))
        _set_sensor(env, "linvel", np.asarray(linvel, dtype=float))
        _set_sensor(env, "gyro", np.zeros(3, dtype=float) if gyro is None else np.asarray(gyro, dtype=float))
        _set_sensor(env, "accel", np.array([0.0, 0.0, 9.81], dtype=float))
        _set_sensor(env, "mag", np.array([2.73e-5, 0.0, -4.54e-5], dtype=float))

    def test_mujoco_run_uses_native_viewer_for_gui_responsiveness(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            with (
                patch("acesim.env.mujoco.mj_env.mujoco.viewer.launch") as launch,
                patch(
                    "acesim.env.mujoco.mj_env.mujoco.viewer.launch_passive",
                    side_effect=AssertionError("passive viewer loop should not be used for GUI"),
                ),
                patch.object(env, "_before_interactive_viewer", wraps=env._before_interactive_viewer) as before,
                patch.object(env, "_after_interactive_viewer", wraps=env._after_interactive_viewer) as after,
            ):
                env.run()

            launch.assert_called_once_with(env._mj_model, env._mj_data)
            before.assert_called_once()
            after.assert_called_once()
            self.assertFalse(env._interactive_viewer_mode)
        finally:
            env.close()

    def test_px4_interactive_viewer_mode_publishes_sensors_from_control_callback(self) -> None:
        _RecordingSensorScheduler.instances.clear()
        _ConnectedFakePX4Transport.events.clear()
        with (
            patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _ConnectedFakePX4Transport),
            patch("acesim.env.mujoco.px4_mj_env.PX4SensorScheduler", _RecordingSensorScheduler),
        ):
            env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            scheduler = _RecordingSensorScheduler.instances[0]
            env._mj_data.time = 0.123

            env._before_interactive_viewer()
            env._control(env._mj_model, env._mj_data)

            self.assertEqual(len(scheduler.calls), 1)
            self.assertEqual(scheduler.calls[0][0], 124000)
            self.assertEqual(_ConnectedFakePX4Transport.events, ["sensor", "actuator"])
        finally:
            env._after_interactive_viewer()
            env.close()

    def test_multicopter_motor_response_maps_controls_to_first_order_rotor_speed(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            controls = np.array([1.0, 0.25, -0.25, 1.2], dtype=float)
            env._handle_applied_actuator_controls(controls)

            expected_controls = np.array([1.0, 0.25, 0.0, 1.0], dtype=float)
            expected_speed_targets = expected_controls * env._params.max_rot_velocity
            np.testing.assert_allclose(env._applied_actuator_controls, expected_controls)
            np.testing.assert_allclose(
                env._desired_rotor_angular_velocity,
                expected_speed_targets,
            )

            env._update_rotor_speed_state(0.01)
            expected_spin_up = first_order_response_step(
                np.zeros(env._rotor_count, dtype=float),
                expected_speed_targets,
                0.01,
                env._params.time_constant_up,
                env._params.time_constant_down,
            )
            np.testing.assert_allclose(env._rotor_angular_velocity, expected_spin_up)

            previous_speed = env._rotor_angular_velocity.copy()
            env._handle_applied_actuator_controls(np.zeros(env._rotor_count, dtype=float))
            env._update_rotor_speed_state(0.01)
            expected_spin_down = first_order_response_step(
                previous_speed,
                np.zeros(env._rotor_count, dtype=float),
                0.01,
                env._params.time_constant_up,
                env._params.time_constant_down,
            )
            np.testing.assert_allclose(env._rotor_angular_velocity, expected_spin_down)
        finally:
            env.close()

    def test_px4_mujoco_sensor_scheduler_runs_after_mj_step_with_post_step_timestamp(self) -> None:
        _RecordingSensorScheduler.instances.clear()
        with (
            patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _ConnectedFakePX4Transport),
            patch("acesim.env.mujoco.px4_mj_env.PX4SensorScheduler", _RecordingSensorScheduler),
        ):
            env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            scheduler = _RecordingSensorScheduler.instances[0]

            env.step()

            self.assertEqual(len(scheduler.calls), 1)
            expected_time_us = int(round(env._mj_data.time * 1_000_000.0))
            self.assertEqual(scheduler.calls[0][0], expected_time_us)
            self.assertEqual(env._simulation_time_us, expected_time_us)
            np.testing.assert_allclose(scheduler.calls[0][1], env._get_sensor_raw("pos"))
        finally:
            env.close()

    def test_px4_mujoco_actuator_polling_stays_inside_control_callback_before_post_step_sensor_publish(
        self,
    ) -> None:
        _RecordingSensorScheduler.instances.clear()
        _ConnectedFakePX4Transport.events.clear()
        with (
            patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _ConnectedFakePX4Transport),
            patch("acesim.env.mujoco.px4_mj_env.PX4SensorScheduler", _RecordingSensorScheduler),
        ):
            env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            env.step()

            transport = cast(_ConnectedFakePX4Transport, env._px4_transport)
            self.assertIsInstance(transport, _ConnectedFakePX4Transport)
            self.assertEqual(transport.update_times, [0])
            self.assertEqual(len(_RecordingSensorScheduler.instances[0].calls), 1)
            self.assertEqual(_ConnectedFakePX4Transport.events, ["actuator", "sensor"])
        finally:
            env.close()

    def test_px4_mujoco_connection_polling_happens_at_most_once_per_step(self) -> None:
        _RecordingSensorScheduler.instances.clear()
        _ConnectsOnPostStepFakePX4Transport.events.clear()
        with (
            patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _ConnectsOnPostStepFakePX4Transport),
            patch("acesim.env.mujoco.px4_mj_env.PX4SensorScheduler", _RecordingSensorScheduler),
        ):
            env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            env.step()

            transport = cast(_ConnectsOnPostStepFakePX4Transport, env._px4_transport)
            self.assertFalse(transport.is_connected)
            self.assertEqual(transport.connection_polls, 1)
            self.assertEqual(len(_RecordingSensorScheduler.instances[0].calls), 0)
            self.assertEqual(_ConnectsOnPostStepFakePX4Transport.events, ["miss"])

            env.step()

            self.assertTrue(transport.is_connected)
            self.assertEqual(transport.connection_polls, 2)
            self.assertEqual(len(_RecordingSensorScheduler.instances[0].calls), 1)
            self.assertEqual(_ConnectsOnPostStepFakePX4Transport.events, ["miss", "connect", "sensor"])
        finally:
            env.close()

    def test_px4_mujoco_visual_rotor_updates_on_every_control_callback(self) -> None:
        with patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _ConnectedFakePX4Transport):
            env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            with patch.object(env, "_update_vehicle_visuals", wraps=env._update_vehicle_visuals) as update_mock:
                env._control(env._mj_model, env._mj_data)
                env._control(env._mj_model, env._mj_data)
                env._control(env._mj_model, env._mj_data)

            self.assertEqual(update_mock.call_count, 3)
        finally:
            env.close()

    def test_visual_rotor_fast_path_matches_rotation_composition(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.1, -0.2, 1.0]), linvel=np.zeros(3))
            base_pos = env._get_sensor_raw("pos")
            base_quat = env._get_sensor_raw("quat")
            rb = Rotation.from_quat(base_quat, scalar_first=True)
            mocap_id = next(mocap_id for mocap_id in env._rotor_mocap_ids if mocap_id >= 0)
            rotor_idx = env._rotor_mocap_ids.index(mocap_id)
            target_speeds = np.zeros(env._rotor_count, dtype=float)
            target_speeds[rotor_idx] = 321.0
            env._advance_visual_rotors(
                mocap_ids=env._rotor_mocap_ids,
                offsets_b=env._rotor_visual_offsets,
                mount_rot=env._rotor_mount_rot,
                rotor_angles=env._rotor_angle,
                visual_speeds=env._visual_rotor_angular_velocity,
                target_speeds=target_speeds,
                spin_directions=env._rotor_direction,
                spin_axes_local=np.array([0.0, 0.0, 1.0], dtype=float),
                smoothing_tc=0.0,
            )

            spin = Rotation.from_rotvec(np.array([0.0, 0.0, env._rotor_angle[rotor_idx]], dtype=float))
            expected_quat = (rb * env._rotor_mount_rot[rotor_idx] * spin).as_quat(scalar_first=True)
            np.testing.assert_allclose(
                env._mj_data.mocap_pos[mocap_id],
                base_pos + rb.apply(env._rotor_visual_offsets[rotor_idx]),
                atol=1e-12,
            )
            np.testing.assert_allclose(env._mj_data.mocap_quat[mocap_id], expected_quat, atol=1e-12)
        finally:
            env.close()

    def test_multicopter_hover_rotors_push_along_world_up_for_level_body(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0

            rotor_positions_w, _, rotor_thrusts, rotor_force_w, rotor_moment_w = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                Rotation.identity(),
                Rotation.identity(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )

            self.assertEqual(rotor_positions_w.shape[0], env._rotor_count)
            self.assertTrue(np.all(rotor_thrusts > 0.0))
            np.testing.assert_allclose(rotor_force_w[:, :2], 0.0, atol=1e-12)
            self.assertTrue(np.all(rotor_force_w[:, 2] > 0.0))
            expected_yaw_sign = -np.sign(env._rotor_direction)
            np.testing.assert_array_equal(np.sign(rotor_moment_w[:, 2]), expected_yaw_sign)
        finally:
            env.close()

    def test_multicopter_axial_velocity_does_not_scale_rotor_thrust(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            base_pos = np.array([0.0, 0.0, 1.0], dtype=float)
            rb = Rotation.identity()

            _, _, baseline_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            baseline_thrusts = baseline_thrusts.copy()
            axial_speed = 12.5
            _, _, up_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.array([0.0, 0.0, axial_speed], dtype=float),
                np.zeros(3, dtype=float),
            )
            _, _, down_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.array([0.0, 0.0, -axial_speed], dtype=float),
                np.zeros(3, dtype=float),
            )

            np.testing.assert_allclose(up_thrusts, baseline_thrusts)
            np.testing.assert_allclose(down_thrusts, baseline_thrusts)
        finally:
            env.close()

    def test_multicopter_linear_speed_command_produces_quadratic_thrust_at_steady_state(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            actuator_output = 0.25
            env._handle_applied_actuator_controls(np.full(env._rotor_count, actuator_output, dtype=float))
            env._update_rotor_speed_state(1.0)

            _, _, rotor_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                Rotation.identity(),
                Rotation.identity(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )

            expected_thrust = actuator_output**2 * env._params.motor_constant * env._params.max_rot_velocity**2
            np.testing.assert_allclose(rotor_thrusts, expected_thrust)
        finally:
            env.close()

    def test_multicopter_lumped_drag_uses_body_frame_mass_normalized_coefficients(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            mass = float(np.sum(env._mj_model.body_mass))
            rb = Rotation.from_euler("z", 90.0, degrees=True)
            rb_inv = rb.inv()
            velocity_w = rb.apply(np.array([2.0, -3.0, 4.0], dtype=float))
            wind_w = rb.apply(np.array([0.5, 1.0, -2.0], dtype=float))
            env._mj_model.opt.wind[:] = wind_w
            force_w = env._compute_lumped_drag_force_w(rb, rb_inv, velocity_w)

            expected_velocity_b = np.array([1.5, -4.0, 6.0], dtype=float)
            expected_force_b = -mass * np.array([0.20, 0.20, 0.00], dtype=float) * expected_velocity_b
            np.testing.assert_allclose(force_w, rb.apply(expected_force_b), atol=1e-12)
            np.testing.assert_allclose(
                env._compute_lumped_drag_force_w(rb, rb_inv, wind_w),
                np.zeros(3, dtype=float),
                atol=1e-12,
            )
        finally:
            env.close()

    def test_px4_mj_wind_velocity_returns_model_option_copy(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            env._mj_model.opt.wind[:] = np.array([1.0, -2.0, 3.0], dtype=float)

            wind_w = env._get_wind_velocity_w()
            wind_w[:] = 0.0

            np.testing.assert_allclose(env._mj_model.opt.wind, np.array([1.0, -2.0, 3.0], dtype=float))
        finally:
            env.close()

    def test_multicopter_apply_lumped_drag_wrench_adds_force_without_direct_moment(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            base_pos = np.array([0.0, 0.0, 1.0], dtype=float)
            velocity_w = np.array([2.0, 0.0, 0.0], dtype=float)
            env._mj_model.opt.wind[:] = np.array([0.5, 0.0, 0.0], dtype=float)
            with patch("acesim.env.mujoco.px4_mj_env.mujoco.mj_applyFT") as apply_ft:
                env._apply_lumped_drag_wrench(base_pos, Rotation.identity(), Rotation.identity(), velocity_w)

            apply_ft.assert_called_once()
            _, _, force_w, moment_w, point_w, body_id, _ = apply_ft.call_args.args
            self.assertLess(float(force_w[0]), 0.0)
            np.testing.assert_allclose(moment_w, np.zeros(3, dtype=float), atol=1e-12)
            np.testing.assert_allclose(point_w, base_pos, atol=1e-12)
            self.assertEqual(body_id, env._base_link_id)
        finally:
            env.close()

    def test_multicopter_lateral_rotor_drag_opposes_lateral_velocity_without_changing_thrust(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            base_pos = np.array([0.0, 0.0, 1.0], dtype=float)
            rb = Rotation.identity()
            lateral_velocity = np.array([3.0, -2.0, 0.0], dtype=float)

            _, _, baseline_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            baseline_thrusts = baseline_thrusts.copy()
            wind_w = np.array([1.0, -0.5, 0.0], dtype=float)
            env._mj_model.opt.wind[:] = wind_w
            relative_lateral_velocity = lateral_velocity - wind_w
            _, _, lateral_thrusts, lateral_force_w, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                lateral_velocity,
                np.zeros(3, dtype=float),
            )

            mu = np.linalg.norm(relative_lateral_velocity[:2]) / (500.0 * env._params.rotor_radius)
            expected_scale = 1.0 + env._rotor_flow_params.advance_c_mu * mu**2
            np.testing.assert_allclose(lateral_thrusts, baseline_thrusts * expected_scale)
            expected_drag = -env._params.rotor_drag_coeff * 500.0 * relative_lateral_velocity
            np.testing.assert_allclose(
                lateral_force_w[:, :2],
                np.tile(expected_drag[:2], (env._rotor_count, 1)),
                atol=1e-12,
            )
            self.assertTrue(np.all(lateral_force_w[:, :2] @ relative_lateral_velocity[:2] < 0.0))
            np.testing.assert_allclose(lateral_force_w[:, 2], lateral_thrusts)
        finally:
            env.close()

    def test_multicopter_rotor_flow_mu_correction_weakly_reduces_lateral_thrust(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            base_pos = np.array([0.0, 0.0, 1.0], dtype=float)
            rb = Rotation.identity()

            _, _, baseline_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            baseline_thrusts = baseline_thrusts.copy()
            _, _, lateral_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.array([12.7, 0.0, 0.0], dtype=float),
                np.zeros(3, dtype=float),
            )

            np.testing.assert_allclose(lateral_thrusts, baseline_thrusts)
            self.assertEqual(env._rotor_flow_params.advance_c_mu, 0.0)
        finally:
            env.close()

    def test_multicopter_ground_effect_only_increases_low_altitude_rotor_thrust(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            rb = Rotation.identity()

            _, _, high_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            high_thrusts = high_thrusts.copy()
            low_base_pos = np.array([0.0, 0.0, 0.10], dtype=float)
            low_rotor_positions_w, _, low_thrusts, _, _ = env._compute_rotor_wrenches(
                low_base_pos,
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )

            height = low_rotor_positions_w[0, 2]
            expected_scale = 1.0 / (1.0 - (env._params.rotor_radius / (4.0 * height)) ** 2)
            np.testing.assert_allclose(low_thrusts, high_thrusts * expected_scale)
            self.assertTrue(np.all(low_thrusts > high_thrusts))
        finally:
            env.close()

    def test_multicopter_ground_effect_uses_static_scene_raycast_above_floor(self) -> None:
        original_merge = MJEnv._merge_scene_robot_xml

        def merge_with_platform(self: MJEnv, scene_path: Path, robot_path: Path) -> str:
            root = ET.fromstring(original_merge(self, scene_path, robot_path))
            worldbody = root.find("worldbody")
            assert worldbody is not None
            ET.SubElement(
                worldbody,
                "geom",
                {
                    "name": "ground_effect_platform",
                    "type": "box",
                    "size": "0.5 0.5 0.02",
                    "pos": "0 0 0.20",
                    "contype": "1",
                    "conaffinity": "1",
                },
            )
            return ET.tostring(root, encoding="unicode")

        with patch.object(MJEnv, "_merge_scene_robot_xml", merge_with_platform):
            env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            rb = Rotation.identity()

            _, _, high_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            high_thrusts = high_thrusts.copy()
            low_rotor_positions_w, _, low_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 0.35], dtype=float),
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )

            platform_top_z = 0.22
            expected_distances = low_rotor_positions_w[:, 2] - platform_top_z
            expected_scales = 1.0 / (1.0 - (env._params.rotor_radius / (4.0 * expected_distances)) ** 2)
            np.testing.assert_allclose(low_thrusts, high_thrusts * expected_scales)
            self.assertTrue(np.all(low_thrusts > high_thrusts))
        finally:
            env.close()

    def test_multicopter_ground_effect_raycast_uses_current_pose_without_frequency_cache(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 0.30]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            rb = Rotation.identity()
            ray_distances = [0.05, 0.20]

            def fake_ray(
                model: object,
                data: object,
                pnt: np.ndarray,
                vec: np.ndarray,
                geomgroup: object,
                flg_static: int,
                bodyexclude: int,
                geomid: np.ndarray,
                cutoff: object,
            ) -> float:
                del model, data, pnt, vec, geomgroup, flg_static, bodyexclude, cutoff
                geomid[0] = 0
                return ray_distances.pop(0) if ray_distances else 0.20

            with patch("acesim.env.mujoco.mc_env.mujoco.mj_ray", side_effect=fake_ray) as ray:
                _, _, first_thrusts, _, _ = env._compute_rotor_wrenches(
                    np.array([0.0, 0.0, 0.30], dtype=float),
                    rb,
                    rb.inv(),
                    np.zeros(3, dtype=float),
                    np.zeros(3, dtype=float),
                )
                first_thrusts = first_thrusts.copy()
                _, _, second_thrusts, _, _ = env._compute_rotor_wrenches(
                    np.array([0.0, 0.0, 0.30], dtype=float),
                    rb,
                    rb.inv(),
                    np.zeros(3, dtype=float),
                    np.zeros(3, dtype=float),
                )

            self.assertGreaterEqual(ray.call_count, 2)
            self.assertGreater(float(first_thrusts[0]), float(second_thrusts[0]))
        finally:
            env.close()

    def test_multicopter_ground_effect_is_zero_when_static_raycast_misses(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            rb = Rotation.identity()

            _, _, high_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            high_thrusts = high_thrusts.copy()
            with patch("acesim.env.mujoco.mc_env.mujoco.mj_ray", return_value=-1.0) as ray:
                _, _, low_thrusts, _, _ = env._compute_rotor_wrenches(
                    np.array([0.0, 0.0, 0.10], dtype=float),
                    rb,
                    rb.inv(),
                    np.zeros(3, dtype=float),
                    np.zeros(3, dtype=float),
                )

            self.assertTrue(ray.called)
            np.testing.assert_allclose(low_thrusts, high_thrusts)
        finally:
            env.close()

    def test_multicopter_ground_effect_is_limited_at_very_low_altitude(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            rb = Rotation.identity()

            _, _, high_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            high_thrusts = high_thrusts.copy()
            _, _, low_thrusts, _, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 0.0], dtype=float),
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )

            np.testing.assert_allclose(low_thrusts, high_thrusts * env._rotor_flow_params.ground_effect_max_scale)
        finally:
            env.close()

    def test_multicopter_projected_body_area_prefers_collision_geoms(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_projected_area_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                link_id = int(env._downwash_body_ids[0])
                area = env._estimate_body_projected_area(
                    link_id,
                    np.array([0.0, 0.0, -1.0], dtype=float),
                )
                selected_geom_ids = env._select_downwash_geom_ids(link_id)

                self.assertGreater(area, 0.0)
                self.assertTrue(selected_geom_ids)
                self.assertTrue(
                    all(
                        env._mj_model.geom_contype[geom_id] != 0 or env._mj_model.geom_conaffinity[geom_id] != 0
                        for geom_id in selected_geom_ids
                    )
                )
            finally:
                env.close()

    def test_multicopter_projected_body_area_reuses_cached_downwash_geometry(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_projected_area_cache_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                link_id = int(env._downwash_body_ids[0])
                self.assertIn(link_id, env._downwash_body_geom_point_offsets)
                with patch.object(env, "_select_downwash_geom_ids", wraps=env._select_downwash_geom_ids) as select_ids:
                    first_area = env._estimate_body_projected_area(link_id, np.array([0.0, 0.0, -1.0], dtype=float))
                    second_area = env._estimate_body_projected_area(link_id, np.array([1.0, 0.0, 0.0], dtype=float))

                self.assertGreater(first_area, 0.0)
                self.assertGreater(second_area, 0.0)
                select_ids.assert_not_called()
            finally:
                env.close()

    def test_multicopter_projected_body_area_uses_cached_convex_projection_hull(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_projected_area_hull_cache_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                link_id = int(env._downwash_body_ids[0])
                self.assertIn(link_id, env._downwash_body_projection_hulls)

                with patch.object(env, "_convex_hull_area_2d", side_effect=AssertionError("slow path used")):
                    area = env._estimate_body_projected_area(
                        link_id,
                        np.array([0.2, -0.4, -1.0], dtype=float),
                    )

                self.assertGreater(area, 0.0)
            finally:
                env.close()

    def test_multicopter_cached_projected_area_matches_slow_projection_for_rotated_body(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_projected_area_cache_match_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                link_id = int(env._downwash_body_ids[0])
                direction_w = np.array([0.3, -0.4, -1.0], dtype=float)
                joint_id = env._mj_model.joint("joint_1").id
                env._mj_data.qpos[env._mj_model.jnt_qposadr[joint_id]] = 0.7
                mujoco.mj_forward(env._mj_model, env._mj_data)

                cached_area = env._estimate_body_projected_area(link_id, direction_w)
                saved_hull = env._downwash_body_projection_hulls.pop(link_id)
                try:
                    slow_area = env._estimate_body_projected_area(link_id, direction_w)
                finally:
                    env._downwash_body_projection_hulls[link_id] = saved_hull

                np.testing.assert_allclose(cached_area, slow_area, rtol=1e-10, atol=1e-12)
            finally:
                env.close()

    def test_multicopter_downwash_does_not_rebuild_projected_area_hulls_each_step(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_hull_cache_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
                env._rotor_angular_velocity[:] = 500.0
                base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
                rotor_positions_w, rotor_axes_w, rotor_thrusts, _, _ = env._compute_rotor_wrenches(
                    base_pos,
                    rb,
                    rb_inv,
                    v_com_w,
                    omega_w,
                )

                with patch.object(env, "_convex_hull_area_2d", side_effect=AssertionError("slow path used")):
                    env._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)
            finally:
                env.close()

    def test_multicopter_downwash_accepts_precomputed_rotor_axes(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_axis_cache_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                body_id = env._downwash_body_ids[0]
                rotor_pos_w = env._mj_data.xpos[env._rotor_body_ids[0]].copy()
                env._mj_data.xipos[body_id] = rotor_pos_w + np.array([0.0, 0.0, -0.20], dtype=float)

                force_w = env._compute_downwash_force_for_body(
                    body_id,
                    np.asarray([rotor_pos_w], dtype=float),
                    np.asarray([3.0], dtype=float),
                    rotor_axes_w=np.asarray([[0.0, 0.0, 1.0]], dtype=float),
                )

                self.assertLess(float(force_w[2]), 0.0)
            finally:
                env.close()

    def test_multicopter_downwash_force_uses_momentum_wake_and_projected_area(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_formula_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                params = env._downwash_params
                body_id = env._downwash_body_ids[0]
                rotor_pos_w = env._mj_data.xpos[env._rotor_body_ids[0]].copy()
                body_pos_w = rotor_pos_w + np.array([0.0, 0.0, -0.20], dtype=float)
                env._mj_data.xipos[body_id] = body_pos_w
                thrust = 3.0

                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch("acesim.env.mujoco.mc_env.mujoco.mj_jacBodyCom"),
                ):
                    force_w = env._compute_downwash_force_for_body(
                        body_id,
                        np.asarray([rotor_pos_w], dtype=float),
                        np.asarray([thrust], dtype=float),
                    )

                disk_area = np.pi * env._params.rotor_radius**2
                wake_speed = params.wake_speed_scale * np.sqrt(2.0 * thrust / (params.air_density * disk_area))
                axial_decay = np.exp(-0.20 / params.axial_decay_m)
                wake_w = wake_speed * axial_decay * np.array([0.0, 0.0, -1.0], dtype=float)
                expected = (
                    0.5
                    * params.air_density
                    * params.drag_coefficient
                    * params.area_scale
                    * 0.02
                    * np.linalg.norm(wake_w)
                    * wake_w
                )
                np.testing.assert_allclose(force_w, expected)
            finally:
                env.close()

    def test_multicopter_downwash_force_reduces_when_body_moves_with_wake(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_velocity_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                body_id = env._downwash_body_ids[0]
                rotor_pos_w = env._mj_data.xpos[env._rotor_body_ids[0]].copy()
                env._mj_data.xipos[body_id] = rotor_pos_w + np.array([0.0, 0.0, -0.20], dtype=float)
                rotor_positions_w = np.asarray([rotor_pos_w], dtype=float)
                rotor_thrusts = np.asarray([3.0], dtype=float)

                def stationary_jacobian(
                    model: object, data: object, jacp: np.ndarray, jacr: np.ndarray, body: int
                ) -> None:
                    return None

                def moving_jacobian(model: object, data: object, jacp: np.ndarray, jacr: np.ndarray, body: int) -> None:
                    jacp[:, 0:3] = np.eye(3)

                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch("acesim.env.mujoco.mc_env.mujoco.mj_jacBodyCom", side_effect=stationary_jacobian),
                ):
                    stationary_force_w = env._compute_downwash_force_for_body(body_id, rotor_positions_w, rotor_thrusts)
                env._mj_data.qvel[0:3] = np.array([0.0, 0.0, -4.0], dtype=float)
                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch("acesim.env.mujoco.mc_env.mujoco.mj_jacBodyCom", side_effect=moving_jacobian),
                ):
                    moving_force_w = env._compute_downwash_force_for_body(body_id, rotor_positions_w, rotor_thrusts)

                self.assertLess(np.linalg.norm(moving_force_w), np.linalg.norm(stationary_force_w))
            finally:
                env.close()

    def test_multicopter_downwash_aggregates_overlapping_wakes_before_body_drag(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_overlap_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                params = env._downwash_params
                body_id = env._downwash_body_ids[0]
                body_pos_w = np.array([0.0, 0.0, 0.0], dtype=float)
                env._mj_data.xipos[body_id] = body_pos_w
                env._mj_model.opt.wind[:] = np.array([2.0, 0.0, 0.0], dtype=float)
                rotor_positions_w = np.array(
                    [
                        [0.0, 0.0, 0.20],
                        [0.02, 0.0, 0.20],
                    ],
                    dtype=float,
                )
                rotor_thrusts = np.array([3.0, 3.0], dtype=float)
                rotor_axes_w = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (2, 1))

                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch("acesim.env.mujoco.mc_env.mujoco.mj_jacBodyCom"),
                ):
                    force_w = env._compute_downwash_force_for_body(
                        body_id,
                        rotor_positions_w,
                        rotor_thrusts,
                        rotor_axes_w=rotor_axes_w,
                    )

                disk_area = np.pi * env._params.rotor_radius**2
                wake_speed = params.wake_speed_scale * np.sqrt(
                    2.0 * rotor_thrusts[0] / (params.air_density * disk_area)
                )
                expected_wake = np.zeros(3, dtype=float)
                for rotor_pos_w in rotor_positions_w:
                    delta_w = body_pos_w - rotor_pos_w
                    axial_distance = -float(delta_w[2])
                    radial_distance = float(np.linalg.norm(delta_w[:2]))
                    wake_radius = env._params.rotor_radius + axial_distance * np.tan(params.wake_spread_angle_rad)
                    profile = max(0.0, 1.0 - (radial_distance / wake_radius) ** 2)
                    axial_decay = np.exp(-axial_distance / params.axial_decay_m)
                    expected_wake += wake_speed * profile * axial_decay * np.array([0.0, 0.0, -1.0], dtype=float)
                v_rel_w = env._mj_model.opt.wind.copy() + expected_wake
                expected = (
                    0.5
                    * params.air_density
                    * params.drag_coefficient
                    * params.area_scale
                    * 0.02
                    * np.linalg.norm(v_rel_w)
                    * v_rel_w
                )

                np.testing.assert_allclose(force_w, expected)
            finally:
                env.close()

    def test_multicopter_downwash_computes_body_com_velocity_once_per_body(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_body_velocity_cache_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                body_id = env._downwash_body_ids[0]
                body_pos_w = np.array([0.0, 0.0, 0.0], dtype=float)
                env._mj_data.xipos[body_id] = body_pos_w
                rotor_positions_w = np.array(
                    [
                        [0.0, 0.0, 0.20],
                        [0.02, 0.0, 0.20],
                    ],
                    dtype=float,
                )
                rotor_thrusts = np.array([3.0, 3.0], dtype=float)
                for rotor_idx, rotor_body_id in enumerate(env._rotor_body_ids[:2]):
                    env._mj_data.xquat[rotor_body_id] = Rotation.identity().as_quat(scalar_first=True)

                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch.object(
                        mujoco,
                        "mj_jacBodyCom",
                        wraps=mujoco.mj_jacBodyCom,
                    ) as velocity,
                ):
                    env._compute_downwash_force_for_body(body_id, rotor_positions_w, rotor_thrusts)

                velocity.assert_called_once()
            finally:
                env.close()

    def test_multicopter_downwash_reuses_body_velocity_jacobian_buffers(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_jacobian_reuse_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                body_id = env._downwash_body_ids[0]
                env._mj_data.xipos[body_id] = np.array([0.0, 0.0, 0.0], dtype=float)
                rotor_positions_w = np.array([[0.0, 0.0, 0.20]], dtype=float)
                rotor_thrusts = np.array([3.0], dtype=float)

                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch.object(mujoco, "mj_jacBodyCom", wraps=mujoco.mj_jacBodyCom) as velocity,
                ):
                    env._compute_downwash_force_for_body(body_id, rotor_positions_w, rotor_thrusts)
                    first_jacp = velocity.call_args.args[2]
                    first_jacr = velocity.call_args.args[3]
                    env._compute_downwash_force_for_body(body_id, rotor_positions_w, rotor_thrusts)
                    second_jacp = velocity.call_args.args[2]
                    second_jacr = velocity.call_args.args[3]

                self.assertIs(first_jacp, second_jacp)
                self.assertIs(first_jacr, second_jacr)
            finally:
                env.close()

    def test_multicopter_convex_hull_area_handles_large_projected_point_clouds(self) -> None:
        angles = np.linspace(0.0, 2.0 * np.pi, 2048, endpoint=False)
        points = np.column_stack((np.cos(angles), np.sin(angles)))

        area = MCEnv._convex_hull_area_2d(points)

        self.assertAlmostEqual(area, np.pi, delta=0.01)

    def test_multicopter_downwash_applies_force_to_configured_scene_bodies(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
                env._rotor_angular_velocity[:] = 500.0
                base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
                rotor_positions_w, rotor_axes_w, rotor_thrusts, rotor_force_w, rotor_moment_w = (
                    env._compute_rotor_wrenches(
                        base_pos,
                        rb,
                        rb_inv,
                        v_com_w,
                        omega_w,
                    )
                )

                with patch("acesim.env.mujoco.mc_env.mujoco.mj_applyFT") as apply_ft:
                    env._apply_rotor_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)
                    env._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)

                downwash_calls = [
                    call for call in apply_ft.call_args_list if int(call.args[5]) in env._downwash_body_ids
                ]
                self.assertTrue(downwash_calls)
                self.assertTrue(any(float(call.args[2][2]) < 0.0 for call in downwash_calls))
                self.assertTrue(all(int(call.args[5]) != env._base_link_id for call in downwash_calls))
            finally:
                env.close()

    def test_multicopter_downwash_body_force_does_not_feed_back_into_rotor_thrust(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_one_way_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
                env._rotor_angular_velocity[:] = 500.0
                base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
                rotor_positions_w, rotor_axes_w, baseline_thrusts, _, _ = env._compute_rotor_wrenches(
                    base_pos,
                    rb,
                    rb_inv,
                    v_com_w,
                    omega_w,
                )
                body_id = env._downwash_body_ids[0]
                env._mj_data.xipos[body_id] = rotor_positions_w[0] + np.array([0.0, 0.0, -0.20], dtype=float)

                with (
                    patch.object(env, "_estimate_body_projected_area", return_value=0.02),
                    patch("acesim.env.mujoco.mc_env.mujoco.mj_jacBodyCom"),
                ):
                    body_force_w = env._compute_downwash_force_for_body(
                        body_id,
                        rotor_positions_w,
                        baseline_thrusts,
                        rotor_axes_w=rotor_axes_w,
                    )
                _, _, thrusts_with_body_in_wake, _, _ = env._compute_rotor_wrenches(
                    base_pos,
                    rb,
                    rb_inv,
                    v_com_w,
                    omega_w,
                )

                self.assertLess(float(body_force_w[2]), 0.0)
                np.testing.assert_allclose(thrusts_with_body_in_wake, baseline_thrusts)
            finally:
                env.close()

    def test_multicopter_downwash_auto_targets_dynamic_bodies_except_exclusions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_auto_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                body_names = {
                    env._mj_model.body(body_id).name
                    for body_id in env._downwash_body_ids
                    if env._mj_model.body(body_id).name
                }

                self.assertFalse(hasattr(env._downwash_params, "affected_bodies"))
                self.assertIn("link_1", body_names)
                self.assertIn("gripper_left", body_names)
                self.assertNotIn("base_link", body_names)
                self.assertNotIn("rotor_1", body_names)
                self.assertNotIn("rotor_1_vis", body_names)
            finally:
                env.close()

    def test_multicopter_downwash_is_zero_when_targets_are_outside_flow_region(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_far_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
                env._rotor_angular_velocity[:] = 500.0
                base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
                rotor_positions_w, rotor_axes_w, rotor_thrusts, _, _ = env._compute_rotor_wrenches(
                    base_pos,
                    rb,
                    rb_inv,
                    v_com_w,
                    omega_w,
                )
                original_positions = {body_id: env._mj_data.xipos[body_id].copy() for body_id in env._downwash_body_ids}
                for body_id in env._downwash_body_ids:
                    env._mj_data.xipos[body_id] = np.array([20.0, 20.0, 20.0], dtype=float)

                with patch("acesim.env.mujoco.mc_env.mujoco.mj_applyFT") as apply_ft:
                    env._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)

                self.assertFalse(apply_ft.called)
                for body_id, original_pos in original_positions.items():
                    env._mj_data.xipos[body_id] = original_pos
            finally:
                env.close()

    def test_multicopter_downwash_skips_projected_area_for_targets_outside_flow_region(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_downwash_early_reject_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500_arm2x")))
            try:
                self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
                env._rotor_angular_velocity[:] = 500.0
                base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
                rotor_positions_w, rotor_axes_w, rotor_thrusts, _, _ = env._compute_rotor_wrenches(
                    base_pos,
                    rb,
                    rb_inv,
                    v_com_w,
                    omega_w,
                )
                for body_id in env._downwash_body_ids:
                    env._mj_data.xipos[body_id] = np.array([20.0, 20.0, 20.0], dtype=float)

                with (
                    patch.object(env, "_estimate_body_projected_area", wraps=env._estimate_body_projected_area) as area,
                    patch.object(mujoco, "mj_jacBodyCom", wraps=mujoco.mj_jacBodyCom) as velocity,
                ):
                    env._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)

                area.assert_not_called()
                velocity.assert_not_called()
            finally:
                env.close()

    def test_multicopter_downwash_is_disabled_when_no_targets_are_configured(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_no_downwash_") as tmpdir:
            env = MCEnv(ConfigLoader(_write_mujoco_config(Path(tmpdir), env_type="mc", asset_name="x500")))
            try:
                self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
                env._rotor_angular_velocity[:] = 500.0
                base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
                rotor_positions_w, rotor_axes_w, rotor_thrusts, _, _ = env._compute_rotor_wrenches(
                    base_pos,
                    rb,
                    rb_inv,
                    v_com_w,
                    omega_w,
                )

                with patch("acesim.env.mujoco.mc_env.mujoco.mj_applyFT") as apply_ft:
                    env._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)

                self.assertFalse(apply_ft.called)
            finally:
                env.close()

    def test_multicopter_uses_actual_rotor_body_axis_when_rotor_is_tilted(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 0.0
            env._rotor_angular_velocity[0] = 500.0
            tilt = Rotation.from_euler("y", 30.0, degrees=True)
            env._mj_data.xmat[env._rotor_body_ids[0]] = tilt.as_matrix().reshape(9)

            _, _, rotor_thrusts, rotor_force_w, _ = env._compute_rotor_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                Rotation.identity(),
                Rotation.identity(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )

            expected_axis_w = tilt.apply(np.array([0.0, 0.0, 1.0], dtype=float))
            np.testing.assert_allclose(rotor_force_w[0], expected_axis_w * rotor_thrusts[0], atol=1e-12)
            self.assertNotAlmostEqual(float(rotor_force_w[0, 0]), 0.0)
            self.assertLess(float(rotor_force_w[0, 2]), float(rotor_thrusts[0]))
        finally:
            env.close()

    def test_multicopter_rotor_axis_uses_mujoco_xmat_hot_path(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            tilt = Rotation.from_euler("y", 20.0, degrees=True)
            env._mj_data.xmat[env._rotor_body_ids[0]] = tilt.as_matrix().reshape(9)

            class _NoFromQuatRotation:
                @staticmethod
                def from_quat(*_args: object, **_kwargs: object) -> object:
                    raise AssertionError("slow path")

            with patch("acesim.env.mujoco.mc_env.Rotation", _NoFromQuatRotation):
                _, rotor_axes_w, _, _, _ = env._compute_rotor_wrenches(
                    np.array([0.0, 0.0, 1.0], dtype=float),
                    Rotation.identity(),
                    Rotation.identity(),
                    np.zeros(3, dtype=float),
                    np.zeros(3, dtype=float),
                )

            expected_axis_w = tilt.apply(np.array([0.0, 0.0, 1.0], dtype=float))
            np.testing.assert_allclose(rotor_axes_w[0], expected_axis_w, atol=1e-12)
        finally:
            env.close()

    def test_advanced_plane_generates_expected_propulsion_and_control_moments(self) -> None:
        env = FWEnv(ConfigLoader(_config_path("advanced_plane")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.2]), linvel=np.array([15.0, 0.0, 0.0]))

            env._handle_applied_actuator_controls(np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], dtype=float))
            env._update_puller_speed(1.0)
            prop_force_b, _ = env._compute_propeller_force(np.array([15.0, 0.0, 0.0], dtype=float))
            self.assertGreater(prop_force_b[0], 0.0)

            env._handle_applied_actuator_controls(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0], dtype=float))
            _, elevator_moment = env._compute_aero_wrench()
            self.assertGreater(abs(elevator_moment[1]), 1e-4)

            env._handle_applied_actuator_controls(
                np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.35, -0.35, 0.0, 0.0], dtype=float)
            )
            _, roll_moment = env._compute_aero_wrench()
            self.assertGreater(abs(roll_moment[0]), 1e-4)

            env._handle_applied_actuator_controls(np.array([0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float))
            _, yaw_moment = env._compute_aero_wrench()
            self.assertGreater(abs(yaw_moment[2]), 1e-4)
            self.assertGreater(env._read_diff_pressure_hpa() or 0.0, 0.0)
        finally:
            env.close()

    def test_standard_vtol_combines_lift_rotors_puller_and_fixed_wing_effects(self) -> None:
        env = VTOLEnv(ConfigLoader(_config_path("standard_vtol")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 0.9]), linvel=np.zeros(3))

            env._handle_applied_actuator_controls(
                np.array([0.25, 1.0, -0.5, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
            )
            expected_lift_controls = np.array([0.25, 1.0, 0.0, 1.0], dtype=float)
            np.testing.assert_allclose(
                env._desired_lift_rotor_angular_velocity,
                expected_lift_controls * env._lift_params.max_rot_velocity,
            )

            env._handle_applied_actuator_controls(np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float))
            env._update_lift_rotor_speed_state(1.0)
            base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
            _, rotor_force_w, _ = env._compute_lift_rotor_wrenches(base_pos, rb, rb_inv, v_com_w, omega_w)
            self.assertGreater(rotor_force_w[:, 2].sum(), 0.0)

            lift_axis_w = Rotation.from_quat(
                env._mj_data.xquat[env._lift_rotor_body_ids[0]].copy(),
                scalar_first=True,
            ).apply(np.array([0.0, 0.0, 1.0], dtype=float))
            self.assertGreater(float(np.dot(rotor_force_w[0], lift_axis_w)), 0.0)

            _, baseline_force_w, _ = env._compute_lift_rotor_wrenches(
                base_pos,
                rb,
                rb_inv,
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            axial_speed = 12.5
            _, up_force_w, _ = env._compute_lift_rotor_wrenches(
                base_pos,
                rb,
                rb_inv,
                lift_axis_w * axial_speed,
                np.zeros(3, dtype=float),
            )
            _, down_force_w, _ = env._compute_lift_rotor_wrenches(
                base_pos,
                rb,
                rb_inv,
                -lift_axis_w * axial_speed,
                np.zeros(3, dtype=float),
            )
            self.assertAlmostEqual(
                float(np.dot(up_force_w[0], lift_axis_w)),
                float(np.dot(baseline_force_w[0], lift_axis_w)),
            )
            self.assertAlmostEqual(
                float(np.dot(down_force_w[0], lift_axis_w)),
                float(np.dot(baseline_force_w[0], lift_axis_w)),
            )

            lift_wind_w = np.array([1.5, -0.5, 0.0], dtype=float)
            lift_velocity_w = np.array([4.5, 1.0, 0.0], dtype=float)
            env._mj_model.opt.wind[:] = lift_wind_w
            _, wind_force_w, _ = env._compute_lift_rotor_wrenches(
                base_pos,
                rb,
                rb_inv,
                lift_velocity_w,
                np.zeros(3, dtype=float),
            )
            expected_lift_drag = (
                -env._lift_params.rotor_drag_coeff
                * env._lift_rotor_angular_velocity[0]
                * (lift_velocity_w - lift_wind_w)
            )
            np.testing.assert_allclose(wind_force_w[0, :2], expected_lift_drag[:2], atol=1e-12)

            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 0.9]), linvel=np.array([18.0, 0.0, 0.0]))
            env._handle_applied_actuator_controls(
                np.array([0.0, 0.0, 0.0, 0.0, 0.8, 0.3, -0.3, 0.35, 0.0], dtype=float)
            )
            _, aero_moment = env._compute_aero_wrench()
            self.assertGreater(np.linalg.norm(aero_moment), 1e-4)
            env._update_puller_speed(1.0)
            body_velocity_flu, _ = env._compute_apparent_body_velocity()
            prop_force_b, _ = env._compute_propeller_force(body_velocity_flu)
            self.assertGreater(prop_force_b[0], 0.0)
        finally:
            env.close()

    def test_uuv_supports_reverse_thrust_buoyancy_and_damping(self) -> None:
        env = UUVEnv(ConfigLoader(_config_path("uuv_bluerov2_heavy")))
        try:
            self._seed_kinematics(
                env,
                pos=np.array([0.0, 0.0, 1.0]),
                linvel=np.array([1.5, 0.0, 0.0]),
                gyro=np.array([0.2, 0.1, 0.05]),
            )

            env._handle_applied_actuator_controls(np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float))
            env._update_rotor_speed_state(1.0)
            rotor_positions_w, rotor_force_w, _ = env._compute_thruster_wrenches(
                np.array([0.0, 0.0, 1.0], dtype=float),
                Rotation.identity(),
                np.zeros(3, dtype=float),
            )
            self.assertEqual(rotor_positions_w.shape[0], 8)
            self.assertLess(np.dot(rotor_force_w[0], env._rotor_axes_b[0]), 0.0)

            hydro_force_b, hydro_torque_b = env._compute_hydrodynamic_wrench(
                np.array([1.5, 0.0, 0.0], dtype=float),
                np.array([0.2, 0.1, 0.05], dtype=float),
            )
            self.assertLess(hydro_force_b[0], 0.0)
            self.assertGreater(np.linalg.norm(hydro_torque_b), 0.0)

            buoyancy_force_b, _ = env._compute_buoyancy_force(Rotation.identity())
            self.assertGreater(buoyancy_force_b[2], 100.0)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
