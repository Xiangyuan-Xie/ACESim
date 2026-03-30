"""MuJoCo standard VTOL environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv


@dataclass
class VTOLElectricLiftParams:
    """Lift rotor parameters for the standard VTOL backend."""

    rotor_direction: np.ndarray
    motor_constant: float
    moment_constant: float
    rotor_drag_coeff: float
    rolling_moment_coeff: float
    time_constant_up: float
    time_constant_down: float
    max_rot_velocity: float
    max_relative_airspeed_mps: float
    idle_visual_speed: float = 120.0
    low_speed_blend_end: float = 180.0
    visual_speed_smoothing_tc: float = 0.02


class VTOLEnv(FWEnv):
    """MuJoCo standard VTOL backend with lift rotors plus fixed-wing aerodynamics."""

    def __init__(self, config_loader: ConfigLoader):
        asset_params = config_loader.get_asset_params()
        config = asset_params.get("vtol", asset_params)
        self._lift_params = VTOLElectricLiftParams(
            rotor_direction=np.asarray(config["rotor_direction"], dtype=float),
            motor_constant=float(config["motor_constant"]),
            moment_constant=float(config["moment_constant"]),
            rotor_drag_coeff=float(config["rotor_drag_coeff"]),
            rolling_moment_coeff=float(config["rolling_moment_coeff"]),
            time_constant_up=float(config.get("time_constant_up", 0.0125)),
            time_constant_down=float(config.get("time_constant_down", 0.025)),
            max_rot_velocity=float(config.get("max_rot_velocity", 1500.0)),
            max_relative_airspeed_mps=float(config.get("max_relative_airspeed_mps", 50.0)),
        )
        self._lift_rotor_indices = list(config.get("lift_rotor_indices", [0, 1, 2, 3]))
        super().__init__(config_loader)

    def _initialize_vehicle_handles(self) -> None:
        super()._initialize_vehicle_handles()
        self._lift_rotor_body_ids = []
        for rotor_index in self._lift_rotor_indices:
            body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}")
            assert body_id >= 0, f"VTOL asset missing lift rotor body rotor_{rotor_index}"
            self._lift_rotor_body_ids.append(body_id)
        self._lift_rotor_mocap_ids, self._lift_rotor_offsets, self._lift_rotor_mount_rot = (
            self._resolve_visual_rotor_group(
                self._lift_rotor_indices,
                body_ids=self._lift_rotor_body_ids,
            )
        )
        rotor_direction = np.asarray(self._lift_params.rotor_direction, dtype=float)
        if rotor_direction.size != len(self._lift_rotor_indices):
            base = np.array([1.0, -1.0], dtype=float)
            rotor_direction = np.tile(base, int(np.ceil(len(self._lift_rotor_indices) / 2)))[
                : len(self._lift_rotor_indices)
            ]
        self._lift_rotor_direction = rotor_direction
        rotor_count = len(self._lift_rotor_indices)
        self._desired_lift_rotor_angular_velocity = np.zeros(rotor_count, dtype=float)
        self._lift_rotor_angular_velocity = np.zeros(rotor_count, dtype=float)
        self._visual_lift_rotor_angular_velocity = np.zeros(rotor_count, dtype=float)
        self._lift_rotor_angle = np.zeros(rotor_count, dtype=float)

    def _required_surface_joint_names(self) -> list[str]:
        return ["left_elevon_joint", "right_elevon_joint", "elevator_joint"]

    def _required_surface_actuator_names(self) -> list[str]:
        return ["left_elevon_ctrl", "right_elevon_ctrl", "elevator_ctrl"]

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        super()._handle_applied_actuator_controls(controls)
        lift_controls = np.clip(np.asarray(controls[: len(self._lift_rotor_indices)], dtype=float), 0.0, 1.0)
        self._desired_lift_rotor_angular_velocity = lift_controls * self._lift_params.max_rot_velocity

    def _update_lift_rotor_speed_state(self, dt_s: float) -> None:
        for i in range(len(self._lift_rotor_indices)):
            diff = self._desired_lift_rotor_angular_velocity[i] - self._lift_rotor_angular_velocity[i]
            tc = self._lift_params.time_constant_up if diff > 0.0 else self._lift_params.time_constant_down
            self._lift_rotor_angular_velocity[i] += diff * (1.0 - np.exp(-dt_s / tc))

    def _compute_lift_rotor_wrenches(
        self,
        base_pos: np.ndarray,
        rb: Rotation,
        rb_inv: Rotation,
        v_com_w: np.ndarray,
        omega_w: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        rotor_positions_w = np.zeros((len(self._lift_rotor_indices), 3), dtype=float)
        rotor_force_w = np.zeros_like(rotor_positions_w)
        rotor_moment_w = np.zeros_like(rotor_positions_w)
        for i, offset in enumerate(self._lift_rotor_offsets):
            r_off_w = rb.apply(offset)
            rotor_positions_w[i] = base_pos + r_off_w
            v_point_w = v_com_w + np.cross(omega_w, r_off_w)
            v_point_r = rb_inv.apply(v_point_w)
            v_parallel_r = np.array([0.0, 0.0, v_point_r[2]], dtype=float)
            v_perp_r = v_point_r - v_parallel_r
            omega = self._lift_rotor_angular_velocity[i]
            omega_abs = abs(omega)
            thrust = abs(self._lift_params.motor_constant * omega * omega_abs)
            scalar = 1.0 - abs(v_parallel_r[2]) / max(self._lift_params.max_relative_airspeed_mps, 1e-6)
            thrust *= float(np.clip(scalar, 0.0, 1.0))
            direction = self._lift_rotor_direction[i]
            torque_z_r = -direction * thrust * self._lift_params.moment_constant
            f_drag_r = -self._lift_params.rotor_drag_coeff * omega_abs * v_perp_r
            m_rolling_r = -self._lift_params.rolling_moment_coeff * omega_abs * direction * v_perp_r
            rotor_force_w[i] = rb.apply(np.array([0.0, 0.0, thrust], dtype=float) + f_drag_r)
            rotor_moment_w[i] = rb.apply(np.array([0.0, 0.0, torque_z_r], dtype=float) + m_rolling_r)
        return rotor_positions_w, rotor_force_w, rotor_moment_w

    def _apply_vehicle_physics(self) -> None:
        self._clear_applied_wrenches()
        self._apply_surface_targets()
        dt_s = self._mj_model.opt.timestep
        self._update_puller_speed(dt_s)
        self._update_lift_rotor_speed_state(dt_s)

        throttle_cmd = float(np.clip(self._applied_actuator_controls[4], 0.0, 1.0))
        aero_force_b, aero_moment_b = self._compute_aero_wrench(throttle_cmd)
        body_velocity_flu, _ = self._compute_apparent_body_velocity()
        prop_force_b, prop_torque_b = self._compute_propeller_force(body_velocity_flu)
        self._apply_body_wrench(aero_force_b, aero_moment_b)
        self._apply_body_wrench(prop_force_b, prop_torque_b, self._puller_offset)

        base_pos, _, rb, rb_inv, v_com_w, _, omega_w = self._get_base_kinematics()
        rotor_positions_w, rotor_force_w, rotor_moment_w = self._compute_lift_rotor_wrenches(
            base_pos,
            rb,
            rb_inv,
            v_com_w,
            omega_w,
        )
        for i in range(len(self._lift_rotor_indices)):
            mujoco.mj_applyFT(
                self._mj_model,
                self._mj_data,
                rotor_force_w[i],
                rotor_moment_w[i],
                rotor_positions_w[i],
                self._base_link_id,
                self._mj_data.qfrc_applied,
            )

    def _compute_visual_lift_rotor_speed(self, rotor_idx: int, armed: bool) -> float:
        physical_speed = max(0.0, float(self._lift_rotor_angular_velocity[rotor_idx]))
        actuator_output = float(np.clip(self._applied_actuator_controls[rotor_idx], 0.0, 1.0))
        if not armed:
            return physical_speed
        if actuator_output <= 0.0:
            return max(physical_speed, self._lift_params.idle_visual_speed)
        blend_end = self._lift_params.low_speed_blend_end
        blend_weight = float(np.clip(1.0 - physical_speed / blend_end, 0.0, 1.0))
        low_speed_target = blend_weight * self._lift_params.idle_visual_speed + (1.0 - blend_weight) * physical_speed
        return max(physical_speed, low_speed_target)

    def _update_vehicle_visuals(self) -> None:
        super()._update_vehicle_visuals()
        armed = self._px4_transport.update_arming_state()
        target_speeds = np.asarray(
            [self._compute_visual_lift_rotor_speed(i, armed) for i in range(len(self._lift_rotor_indices))],
            dtype=float,
        )
        self._advance_visual_rotors(
            mocap_ids=self._lift_rotor_mocap_ids,
            offsets_b=self._lift_rotor_offsets,
            mount_rot=self._lift_rotor_mount_rot,
            rotor_angles=self._lift_rotor_angle,
            visual_speeds=self._visual_lift_rotor_angular_velocity,
            target_speeds=target_speeds,
            spin_directions=self._lift_rotor_direction,
            spin_axes_local=np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (len(self._lift_rotor_indices), 1)),
            smoothing_tc=self._lift_params.visual_speed_smoothing_tc,
        )

    def _get_visual_rotor_angle(self) -> np.ndarray:
        return np.concatenate([self._lift_rotor_angle.copy(), np.array([self._puller_angle], dtype=float)])

    def _get_visual_rotor_speed(self) -> np.ndarray:
        return np.concatenate(
            [
                self._visual_lift_rotor_angular_velocity.copy(),
                np.array([self._visual_puller_angular_velocity], dtype=float),
            ]
        )
