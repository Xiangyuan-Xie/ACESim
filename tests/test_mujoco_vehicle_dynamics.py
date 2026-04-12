from __future__ import annotations

import unittest
from pathlib import Path
from typing import Protocol
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.env.mujoco.uuv_env import UUVEnv
from acesim.env.mujoco.vtol_env import VTOLEnv
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


class _MuJoCoModelLike(Protocol):
    sensor_adr: np.ndarray
    sensor_dim: np.ndarray


class _MuJoCoDataLike(Protocol):
    sensordata: np.ndarray


class _SupportsSensorSeeding(Protocol):
    _sensor_id_map: dict[str, int]
    _mj_model: _MuJoCoModelLike
    _mj_data: _MuJoCoDataLike


def _config_path(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "config" / f"{name}.toml").resolve()


def _set_sensor(env: _SupportsSensorSeeding, sensor_name: str, values: np.ndarray) -> None:
    sensor_id = env._sensor_id_map[sensor_name]
    adr = env._mj_model.sensor_adr[sensor_id]
    dim = env._mj_model.sensor_dim[sensor_id]
    env._mj_data.sensordata[adr : adr + dim] = np.asarray(values, dtype=float)


@patch("acesim.env.mujoco.px4_mj_env.VehicleVisualStatePublisher", _FakeVisualPublisher)
@patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _FakePX4Transport)
@patch("acesim.env.mujoco.mj_env.SimulationClock", lambda: SimulationClock(enable_zmq=False))
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

    def test_plane_generates_expected_propulsion_and_control_moments(self) -> None:
        env = FWEnv(ConfigLoader(_config_path("plane")))
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

    def test_standard_vtol_combines_lift_rotors_and_fixed_wing_effects(self) -> None:
        env = VTOLEnv(ConfigLoader(_config_path("standard_vtol")))
        try:
            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 0.9]), linvel=np.zeros(3))

            env._handle_applied_actuator_controls(np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float))
            env._update_lift_rotor_speed_state(1.0)
            base_pos, _, rb, rb_inv, v_com_w, _, omega_w = env._get_base_kinematics()
            _, rotor_force_w, _ = env._compute_lift_rotor_wrenches(base_pos, rb, rb_inv, v_com_w, omega_w)
            self.assertGreater(rotor_force_w[:, 2].sum(), 0.0)

            self._seed_kinematics(env, pos=np.array([0.0, 0.0, 0.9]), linvel=np.array([18.0, 0.0, 0.0]))
            env._handle_applied_actuator_controls(
                np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.3, -0.3, 0.35, 0.0], dtype=float)
            )
            env._update_puller_speed(1.0)
            prop_force_b, _ = env._compute_propeller_force(np.array([18.0, 0.0, 0.0], dtype=float))
            _, aero_moment = env._compute_aero_wrench()
            self.assertGreater(prop_force_b[0], 0.0)
            self.assertGreater(np.linalg.norm(aero_moment), 1e-4)
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
