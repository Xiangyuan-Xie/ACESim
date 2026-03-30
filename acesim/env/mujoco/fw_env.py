"""MuJoCo fixed-wing environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.utils.px4_sih_aero import AeroSeg


@dataclass
class FWParams:
    """Vehicle parameters for the fixed-wing MuJoCo backend."""

    thrust_motor_constant: float
    thrust_moment_constant: float
    thrust_time_constant_up: float
    thrust_time_constant_down: float
    thrust_max_rot_velocity: float
    thrust_max_relative_airspeed_mps: float
    thrust_rotor_drag_coeff: float
    thrust_rolling_moment_coeff: float
    thrust_rotor_direction: float
    linear_damping: float
    angular_damping: float
    air_density_sea_level: float = 1.225
    idle_visual_speed: float = 120.0
    low_speed_blend_end: float = 180.0
    visual_speed_smoothing_tc: float = 0.02


class FWEnv(PX4MJEnv):
    """MuJoCo fixed-wing backend with SIH-inspired aerodynamics."""

    def __init__(self, config_loader: ConfigLoader):
        asset_params = config_loader.get_asset_params()
        config = asset_params.get("fw", asset_params)
        self._params = FWParams(
            thrust_motor_constant=float(config["thrust_motor_constant"]),
            thrust_moment_constant=float(config["thrust_moment_constant"]),
            thrust_time_constant_up=float(config.get("thrust_time_constant_up", 0.0125)),
            thrust_time_constant_down=float(config.get("thrust_time_constant_down", 0.025)),
            thrust_max_rot_velocity=float(config.get("thrust_max_rot_velocity", 3500.0)),
            thrust_max_relative_airspeed_mps=float(config.get("thrust_max_relative_airspeed_mps", 80.0)),
            thrust_rotor_drag_coeff=float(config.get("thrust_rotor_drag_coeff", 0.0)),
            thrust_rolling_moment_coeff=float(config.get("thrust_rolling_moment_coeff", 0.0)),
            thrust_rotor_direction=float(config.get("thrust_rotor_direction", 1.0)),
            linear_damping=float(config.get("linear_damping", 0.2)),
            angular_damping=float(config.get("angular_damping", 0.05)),
        )
        self._segment_config = config.get("segments", {})
        super().__init__(config_loader)

    def _initialize_vehicle_handles(self) -> None:
        self._puller_rotor_index = int(self._segment_config.get("puller_rotor_index", 4))
        self._puller_body_id = mujoco.mj_name2id(
            self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{self._puller_rotor_index}"
        )
        assert self._puller_body_id >= 0, "Fixed-wing asset must define puller rotor body"
        puller_mocap_ids, puller_offsets, puller_mount_rot = self._resolve_visual_rotor_group(
            [self._puller_rotor_index],
            body_ids=[self._puller_body_id],
        )
        self._puller_mocap_id = puller_mocap_ids[0]
        self._puller_offset = puller_offsets[0].copy()
        self._puller_mount_rot = puller_mount_rot[0]
        self._desired_puller_angular_velocity = 0.0
        self._puller_angular_velocity = 0.0
        self._visual_puller_angular_velocity = 0.0
        self._puller_angle = 0.0
        self._applied_actuator_controls = np.zeros(9, dtype=float)

        self._surface_joint_names = [
            "rudder_joint",
            "left_flap_joint",
            "right_flap_joint",
            "left_elevon_joint",
            "right_elevon_joint",
            "elevator_joint",
        ]
        self._surface_actuator_names = [
            "rudder_ctrl",
            "left_flap_ctrl",
            "right_flap_ctrl",
            "left_elevon_ctrl",
            "right_elevon_ctrl",
            "elevator_ctrl",
        ]
        self._surface_joint_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self._surface_joint_names
        ]
        self._surface_actuator_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self._surface_actuator_names
        ]
        required_joint_names = set(self._required_surface_joint_names())
        required_actuator_names = set(self._required_surface_actuator_names())
        missing_joints = [
            name
            for name, joint_id in zip(self._surface_joint_names, self._surface_joint_ids)
            if joint_id < 0 and name in required_joint_names
        ]
        missing_actuators = [
            name
            for name, actuator_id in zip(self._surface_actuator_names, self._surface_actuator_ids)
            if actuator_id < 0 and name in required_actuator_names
        ]
        assert not missing_joints, f"Fixed-wing asset missing control-surface joints: {', '.join(missing_joints)}"
        assert not missing_actuators, f"Fixed-wing asset missing control actuators: {', '.join(missing_actuators)}"
        self._surface_targets = np.zeros(len(self._surface_joint_names), dtype=float)
        self._build_aero_segments()
        self._last_true_airspeed_mps = 0.0
        self._last_diff_pressure_hpa = 0.0
        self._last_air_density = self._params.air_density_sea_level

    def _required_surface_joint_names(self) -> list[str]:
        return self._surface_joint_names

    def _required_surface_actuator_names(self) -> list[str]:
        return self._surface_actuator_names

    def _build_aero_segments(self) -> None:
        segments = self._segment_config
        wing_span = float(segments.get("wing_span", 0.86))
        wing_mac = float(segments.get("wing_mac", 0.21))
        wing_cf = float(segments.get("wing_flap_chord", wing_mac / 3.0))
        wing_ar = float(segments.get("wing_ar", wing_span / wing_mac))
        self._wing_l = AeroSeg(
            wing_span / 2.0,
            wing_mac,
            float(segments.get("wing_alpha_0_deg", -4.0)),
            np.array(segments.get("wing_left_pos_b", [0.0, -wing_span / 4.0, 0.0]), dtype=float),
            float(segments.get("wing_left_dihedral_deg", 3.0)),
            wing_ar,
            wing_cf,
        )
        self._wing_r = AeroSeg(
            wing_span / 2.0,
            wing_mac,
            float(segments.get("wing_alpha_0_deg", -4.0)),
            np.array(segments.get("wing_right_pos_b", [0.0, wing_span / 4.0, 0.0]), dtype=float),
            float(segments.get("wing_right_dihedral_deg", -3.0)),
            wing_ar,
            wing_cf,
        )
        prop_radius = float(segments.get("prop_radius", 0.1))
        self._tailplane = AeroSeg(
            float(segments.get("tail_span", 0.3)),
            float(segments.get("tail_mac", 0.1)),
            float(segments.get("tail_alpha_0_deg", 0.0)),
            np.array(segments.get("tail_pos_b", [-0.4, 0.0, 0.0]), dtype=float),
            float(segments.get("tail_dihedral_deg", 0.0)),
            float(segments.get("tail_ar", -1.0)),
            float(segments.get("tail_flap_chord", 0.05)),
            prop_radius,
        )
        self._fin = AeroSeg(
            float(segments.get("fin_span", 0.25)),
            float(segments.get("fin_mac", 0.18)),
            float(segments.get("fin_alpha_0_deg", 0.0)),
            np.array(segments.get("fin_pos_b", [-0.45, 0.0, -0.1]), dtype=float),
            float(segments.get("fin_dihedral_deg", -90.0)),
            float(segments.get("fin_ar", -1.0)),
            float(segments.get("fin_flap_chord", 0.12)),
            prop_radius,
        )
        self._fuselage = AeroSeg(
            float(segments.get("fuselage_span", 0.2)),
            float(segments.get("fuselage_mac", 0.8)),
            float(segments.get("fuselage_alpha_0_deg", 0.0)),
            np.array(segments.get("fuselage_pos_b", [0.0, 0.0, 0.0]), dtype=float),
            float(segments.get("fuselage_dihedral_deg", -90.0)),
        )

    def _actuator_channel_count(self) -> int:
        return 9

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        self._applied_actuator_controls = np.asarray(controls, dtype=float)
        self._surface_targets[:] = 0.0
        self._surface_targets[0] = float(np.clip(self._applied_actuator_controls[2], -1.0, 1.0))
        self._surface_targets[1] = float(np.clip(-self._applied_actuator_controls[3], -1.0, 1.0))
        self._surface_targets[2] = float(np.clip(-self._applied_actuator_controls[8], -1.0, 1.0))
        self._surface_targets[3] = float(np.clip(self._applied_actuator_controls[5], -1.0, 1.0))
        self._surface_targets[4] = float(np.clip(self._applied_actuator_controls[6], -1.0, 1.0))
        self._surface_targets[5] = float(np.clip(self._applied_actuator_controls[7], -1.0, 1.0))
        self._desired_puller_angular_velocity = (
            float(np.clip(self._applied_actuator_controls[4], 0.0, 1.0)) * self._params.thrust_max_rot_velocity
        )

    def _apply_surface_targets(self) -> None:
        for target, act_id in zip(self._surface_targets, self._surface_actuator_ids):
            if act_id >= 0:
                self._mj_data.ctrl[act_id] = target

    def _update_puller_speed(self, dt_s: float) -> None:
        diff = self._desired_puller_angular_velocity - self._puller_angular_velocity
        tc = self._params.thrust_time_constant_up if diff > 0.0 else self._params.thrust_time_constant_down
        self._puller_angular_velocity += diff * (1.0 - np.exp(-dt_s / tc))

    def _compute_propeller_force(self, body_velocity_flu: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        omega = self._puller_angular_velocity
        omega_abs = abs(omega)
        thrust = self._params.thrust_motor_constant * omega * omega_abs
        thrust = abs(thrust)
        axial_speed = abs(body_velocity_flu[0])
        scalar = 1.0 - axial_speed / max(self._params.thrust_max_relative_airspeed_mps, 1e-6)
        thrust *= float(np.clip(scalar, 0.0, 1.0))
        force_b = np.array([thrust, 0.0, 0.0], dtype=float)
        torque_b = np.array(
            [
                -self._params.thrust_rotor_direction * thrust * self._params.thrust_moment_constant,
                0.0,
                0.0,
            ],
            dtype=float,
        )
        return force_b, torque_b

    def _compute_apparent_body_velocity(self) -> tuple[np.ndarray, float]:
        _, _, _, rb_inv, v_com_w, _, _ = self._get_base_kinematics()
        v_body = rb_inv.apply(v_com_w)
        altitude_m = float(self._px4_sensor_params.gps_alt_start + self._get_sensor_raw("pos")[2])
        return v_body, altitude_m

    def _update_airspeed_state(self, rho: float, body_velocity_flu: np.ndarray) -> None:
        true_airspeed_mps = max(0.0, float(body_velocity_flu[0]))
        self._last_true_airspeed_mps = true_airspeed_mps
        self._last_air_density = rho
        self._last_diff_pressure_hpa = 0.5 * rho * true_airspeed_mps * true_airspeed_mps * 0.01

    def _compute_aero_wrench(self, throttle_cmd: float) -> tuple[np.ndarray, np.ndarray]:
        body_velocity_flu, altitude_m = self._compute_apparent_body_velocity()
        body_rates_flu = self._get_sensor_raw("gyro")
        left_deflection = self._surface_targets[3] + self._surface_targets[1]
        right_deflection = self._surface_targets[4] + self._surface_targets[2]
        self._wing_l.update_aero(body_velocity_flu, body_rates_flu, altitude_m, left_deflection)
        self._wing_r.update_aero(body_velocity_flu, body_rates_flu, altitude_m, right_deflection)
        thrust_force, _ = self._compute_propeller_force(body_velocity_flu)
        tail_thrust = thrust_force[0] if throttle_cmd > 0.0 else 0.0
        self._tailplane.update_aero(
            body_velocity_flu, body_rates_flu, altitude_m, -self._surface_targets[5], tail_thrust
        )
        self._fin.update_aero(body_velocity_flu, body_rates_flu, altitude_m, self._surface_targets[0], tail_thrust)
        self._fuselage.update_aero(body_velocity_flu, body_rates_flu, altitude_m)
        rho = self._wing_l.rho
        self._update_airspeed_state(rho, body_velocity_flu)

        force_b = (
            self._wing_l.get_fa()
            + self._wing_r.get_fa()
            + self._tailplane.get_fa()
            + self._fin.get_fa()
            + self._fuselage.get_fa()
            - self._params.linear_damping * body_velocity_flu
        )
        moment_b = (
            self._wing_l.get_ma()
            + self._wing_r.get_ma()
            + self._tailplane.get_ma()
            + self._fin.get_ma()
            + self._fuselage.get_ma()
            - self._params.angular_damping * body_rates_flu
        )
        return force_b, moment_b

    def _apply_vehicle_physics(self) -> None:
        self._clear_applied_wrenches()
        self._apply_surface_targets()
        dt_s = self._mj_model.opt.timestep
        self._update_puller_speed(dt_s)
        throttle_cmd = float(np.clip(self._applied_actuator_controls[4], 0.0, 1.0))
        aero_force_b, aero_moment_b = self._compute_aero_wrench(throttle_cmd)
        prop_force_b, prop_torque_b = self._compute_propeller_force(self._compute_apparent_body_velocity()[0])
        self._apply_body_wrench(aero_force_b, aero_moment_b)
        self._apply_body_wrench(prop_force_b, prop_torque_b, self._puller_offset)

    def _compute_visual_prop_speed(self, armed: bool) -> float:
        physical_speed = max(0.0, float(self._puller_angular_velocity))
        actuator_output = float(np.clip(self._applied_actuator_controls[4], 0.0, 1.0))
        if not armed:
            return physical_speed
        if actuator_output <= 0.0:
            return max(physical_speed, self._params.idle_visual_speed)
        blend_end = self._params.low_speed_blend_end
        blend_weight = float(np.clip(1.0 - physical_speed / blend_end, 0.0, 1.0))
        low_speed_target = blend_weight * self._params.idle_visual_speed + (1.0 - blend_weight) * physical_speed
        return max(physical_speed, low_speed_target)

    def _update_vehicle_visuals(self) -> None:
        self._apply_surface_targets()
        if self._puller_mocap_id < 0:
            return
        armed = self._px4_transport.update_arming_state()
        target_speed = self._compute_visual_prop_speed(armed)
        rotor_angles = np.asarray([self._puller_angle], dtype=float)
        visual_speeds = np.asarray([self._visual_puller_angular_velocity], dtype=float)
        self._advance_visual_rotors(
            mocap_ids=[self._puller_mocap_id],
            offsets_b=np.asarray([self._puller_offset], dtype=float),
            mount_rot=[self._puller_mount_rot],
            rotor_angles=rotor_angles,
            visual_speeds=visual_speeds,
            target_speeds=np.asarray([target_speed], dtype=float),
            spin_directions=np.asarray([self._params.thrust_rotor_direction], dtype=float),
            spin_axes_local=np.asarray([[1.0, 0.0, 0.0]], dtype=float),
            smoothing_tc=self._params.visual_speed_smoothing_tc,
        )
        self._puller_angle = float(rotor_angles[0])
        self._visual_puller_angular_velocity = float(visual_speeds[0])

    def _get_visual_rotor_angle(self) -> np.ndarray:
        return np.array([self._puller_angle], dtype=float)

    def _get_visual_rotor_speed(self) -> np.ndarray:
        return np.array([self._visual_puller_angular_velocity], dtype=float)

    def _read_diff_pressure_hpa(self) -> float | None:
        return self._last_diff_pressure_hpa
