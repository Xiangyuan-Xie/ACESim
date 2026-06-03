from __future__ import annotations

import unittest
from pathlib import Path
from typing import Protocol
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv
from acesim.env.mujoco.mc_env import MCEnv
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.env.mujoco.uuv_env import UUVEnv
from acesim.env.mujoco.vtol_env import VTOLEnv
from acesim.utils.dynamics import first_order_response_step


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

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        return None

    def read_applied_actuator_controls(self, channel_count: int) -> None:
        return None

    def update_arming_state(self) -> bool:
        return False

    def close(self) -> None:
        return None


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

    def test_multicopter_motor_response_maps_controls_to_first_order_rotor_speed(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            controls = np.array([1.0, 0.5, -0.25, 1.2], dtype=float)
            env._handle_applied_actuator_controls(controls)

            expected_controls = np.array([1.0, 0.5, 0.0, 1.0], dtype=float)
            np.testing.assert_allclose(env._applied_actuator_controls, expected_controls)
            np.testing.assert_allclose(
                env._desired_rotor_angular_velocity,
                expected_controls * env._params.max_rot_velocity,
            )

            env._update_rotor_speed_state(0.01)
            expected_spin_up = first_order_response_step(
                np.zeros(env._rotor_count, dtype=float),
                expected_controls * env._params.max_rot_velocity,
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

    def test_multicopter_hover_rotors_push_along_world_up_for_level_body(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0

            rotor_positions_w, rotor_thrusts, rotor_force_w, rotor_moment_w = env._compute_rotor_wrenches(
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

    def test_multicopter_signed_axial_inflow_reduces_and_boosts_thrust(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 500.0
            base_pos = np.array([0.0, 0.0, 1.0], dtype=float)
            rb = Rotation.identity()

            _, baseline_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            axial_speed = 0.5 * env._params.max_relative_airspeed_mps
            _, up_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.array([0.0, 0.0, axial_speed], dtype=float),
                np.zeros(3, dtype=float),
            )
            _, down_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.array([0.0, 0.0, -axial_speed], dtype=float),
                np.zeros(3, dtype=float),
            )

            np.testing.assert_allclose(up_thrusts, baseline_thrusts * 0.5)
            np.testing.assert_allclose(down_thrusts, baseline_thrusts * 1.25)
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

            _, baseline_thrusts, _, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
            )
            _, lateral_thrusts, lateral_force_w, _ = env._compute_rotor_wrenches(
                base_pos,
                rb,
                rb.inv(),
                lateral_velocity,
                np.zeros(3, dtype=float),
            )

            np.testing.assert_allclose(lateral_thrusts, baseline_thrusts)
            expected_drag = -env._params.rotor_drag_coeff * 500.0 * lateral_velocity
            np.testing.assert_allclose(
                lateral_force_w[:, :2],
                np.tile(expected_drag[:2], (env._rotor_count, 1)),
                atol=1e-12,
            )
            self.assertTrue(np.all(lateral_force_w[:, :2] @ lateral_velocity[:2] < 0.0))
            np.testing.assert_allclose(lateral_force_w[:, 2], baseline_thrusts)
        finally:
            env.close()

    def test_multicopter_uses_actual_rotor_body_axis_when_rotor_is_tilted(self) -> None:
        env = MCEnv(ConfigLoader(_config_path("default")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 1.0]), linvel=np.zeros(3))
            env._rotor_angular_velocity[:] = 0.0
            env._rotor_angular_velocity[0] = 500.0
            tilt = Rotation.from_euler("y", 30.0, degrees=True)
            env._mj_data.xquat[env._rotor_body_ids[0]] = tilt.as_quat(scalar_first=True)

            _, rotor_thrusts, rotor_force_w, _ = env._compute_rotor_wrenches(
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
            axial_speed = 0.5 * env._lift_params.max_relative_airspeed_mps
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
            self.assertLess(float(np.dot(up_force_w[0], lift_axis_w)), float(np.dot(baseline_force_w[0], lift_axis_w)))
            self.assertGreater(
                float(np.dot(down_force_w[0], lift_axis_w)),
                float(np.dot(baseline_force_w[0], lift_axis_w)),
            )

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
