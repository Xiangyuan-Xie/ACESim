"""MuJoCo standard VTOL environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.fw_env import FWEnv
from acesim.utils.dynamics import LumpedDragParams, first_order_response_step, idle_visual_speed_target


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
        )
        self._lumped_drag_params = LumpedDragParams.from_config(config.get("lumped_drag"))
        self._lift_rotor_indices = list(config.get("lift_rotor_indices", [0, 1, 2, 3]))
        self._puller_rotor_index = int(config.get("puller_rotor_index", 4))
        self._tilt_joint_names = list(config.get("tilt_joint_names", []))
        self._tilt_servo_channel_indices = np.asarray(config.get("tilt_servo_channel_indices", []), dtype=int)
        self._tilt_actuator_names = list(config.get("tilt_actuator_names", []))
        super().__init__(config_loader)

    def _initialize_vehicle_handles(self) -> None:
        super()._initialize_vehicle_handles()
        self._lift_rotor_body_ids = []
        for rotor_index in self._lift_rotor_indices:
            body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}")
            assert body_id >= 0, f"VTOL asset missing lift rotor body rotor_{rotor_index}"
            self._lift_rotor_body_ids.append(body_id)
        (
            self._lift_rotor_mocap_ids,
            self._lift_rotor_offsets,
            self._lift_rotor_visual_offsets,
            self._lift_rotor_mount_rot,
        ) = self._resolve_visual_rotor_group(
            self._lift_rotor_indices,
            body_ids=self._lift_rotor_body_ids,
        )
        self._lift_rotor_site_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, f"rotor_joint_thrust{rotor_index}")
            for rotor_index in self._lift_rotor_indices
        ]
        self._lift_rotor_visual_relative_offsets = np.zeros((len(self._lift_rotor_indices), 3), dtype=float)
        self._lift_rotor_visual_relative_rot: list[Rotation] = []
        for i, rotor_index in enumerate(self._lift_rotor_indices):
            vis_body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}_vis")
            if vis_body_id < 0:
                self._lift_rotor_visual_relative_rot.append(Rotation.identity())
                continue
            body_rot = Rotation.from_quat(self._mj_data.xquat[self._lift_rotor_body_ids[i]].copy(), scalar_first=True)
            vis_rot = Rotation.from_quat(self._mj_data.xquat[vis_body_id].copy(), scalar_first=True)
            self._lift_rotor_visual_relative_offsets[i] = body_rot.inv().apply(
                self._mj_data.xpos[vis_body_id].copy() - self._mj_data.xpos[self._lift_rotor_body_ids[i]].copy()
            )
            self._lift_rotor_visual_relative_rot.append(body_rot.inv() * vis_rot)
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
        self._lift_rotor_positions_w = np.zeros((rotor_count, 3), dtype=float)
        self._lift_rotor_force_w = np.zeros((rotor_count, 3), dtype=float)
        self._lift_rotor_moment_w = np.zeros((rotor_count, 3), dtype=float)
        self._tilt_joint_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            for joint_name in self._tilt_joint_names
        ]
        self._tilt_actuator_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            for actuator_name in self._tilt_actuator_names
        ]
        self._tilt_targets = np.zeros(len(self._tilt_joint_ids), dtype=float)

    def _required_surface_joint_names(self) -> list[str]:
        return ["left_elevon_joint", "right_elevon_joint", "elevator_joint"]

    def _required_surface_actuator_names(self) -> list[str]:
        return ["left_elevon_ctrl", "right_elevon_ctrl", "elevator_ctrl"]

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        super()._handle_applied_actuator_controls(controls)
        lift_controls = np.clip(np.asarray(controls[: len(self._lift_rotor_indices)], dtype=float), 0.0, 1.0)
        self._desired_lift_rotor_angular_velocity = lift_controls * self._lift_params.max_rot_velocity
        for i, (channel_idx, joint_id) in enumerate(zip(self._tilt_servo_channel_indices, self._tilt_joint_ids)):
            if not (0 <= int(channel_idx) < len(controls)) or joint_id < 0:
                continue
            lower, upper = self._mj_model.jnt_range[joint_id]
            self._tilt_targets[i] = float(np.clip(controls[int(channel_idx)], lower, upper))

    def _apply_tilt_targets(self) -> None:
        for target, actuator_id in zip(self._tilt_targets, self._tilt_actuator_ids):
            if actuator_id >= 0:
                self._mj_data.ctrl[actuator_id] = target

    def _update_lift_rotor_speed_state(self, dt_s: float) -> None:
        self._lift_rotor_angular_velocity = first_order_response_step(
            self._lift_rotor_angular_velocity,
            self._desired_lift_rotor_angular_velocity,
            dt_s,
            self._lift_params.time_constant_up,
            self._lift_params.time_constant_down,
        )

    def _compute_lift_rotor_wrenches(
        self,
        base_pos: np.ndarray,
        rb: Rotation,
        rb_inv: Rotation,
        v_com_w: np.ndarray,
        omega_w: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        rotor_positions_w = self._lift_rotor_positions_w
        rotor_force_w = self._lift_rotor_force_w
        rotor_moment_w = self._lift_rotor_moment_w
        rotor_force_w[:] = 0.0
        rotor_moment_w[:] = 0.0
        wind_w = self._get_wind_velocity_w()
        rb_mat = rb.as_matrix()
        rb_inv_mat = rb_inv.as_matrix()
        for i, body_id in enumerate(self._lift_rotor_body_ids):
            rotor_positions_w[i] = (
                self._mj_data.site_xpos[self._lift_rotor_site_ids[i]]
                if self._lift_rotor_site_ids[i] >= 0
                else self._mj_data.xpos[body_id]
            )
            r_off_w = rotor_positions_w[i] - base_pos
            v_point_w = v_com_w + np.cross(omega_w, r_off_w)
            v_air_point_w = v_point_w - wind_w
            v_point_r = rb_inv_mat @ v_air_point_w
            rotor_axis_w = self._mj_data.xmat[body_id].reshape(3, 3)[:, 2]
            rotor_axis_r = rb_inv_mat @ rotor_axis_w
            rotor_axis_r = rotor_axis_r / max(np.linalg.norm(rotor_axis_r), 1e-12)
            v_parallel_r = rotor_axis_r * float(np.dot(v_point_r, rotor_axis_r))
            v_perp_r = v_point_r - v_parallel_r
            omega = self._lift_rotor_angular_velocity[i]
            omega_abs = abs(omega)
            thrust = abs(self._lift_params.motor_constant * omega * omega_abs)
            direction = self._lift_rotor_direction[i]
            torque_axis_r = -direction * thrust * self._lift_params.moment_constant * rotor_axis_r
            f_drag_r = -self._lift_params.rotor_drag_coeff * omega_abs * v_perp_r
            m_rolling_r = -self._lift_params.rolling_moment_coeff * omega_abs * direction * v_perp_r
            rotor_force_w[i] = rb_mat @ (rotor_axis_r * thrust + f_drag_r)
            rotor_moment_w[i] = rb_mat @ (torque_axis_r + m_rolling_r)
        return rotor_positions_w, rotor_force_w, rotor_moment_w

    def _apply_vehicle_physics(self) -> None:
        self._clear_applied_wrenches()
        self._apply_surface_targets()
        self._apply_tilt_targets()
        dt_s = self._mj_model.opt.timestep
        if self._puller_body_id >= 0:
            self._update_puller_speed(dt_s)
        self._update_lift_rotor_speed_state(dt_s)

        aero_force_b, aero_moment_b = self._compute_aero_wrench()
        body_velocity_flu, _ = self._compute_apparent_body_velocity()
        prop_force_b, prop_torque_b = self._compute_propeller_force(body_velocity_flu)
        self._apply_body_wrench(aero_force_b, aero_moment_b)
        if self._puller_body_id >= 0:
            self._apply_body_wrench(prop_force_b, prop_torque_b, self._puller_offset)

        base_pos, _, rb, rb_inv, v_com_w, _, omega_w = self._get_base_kinematics()
        rotor_positions_w, rotor_force_w, rotor_moment_w = self._compute_lift_rotor_wrenches(
            base_pos,
            rb,
            rb_inv,
            v_com_w,
            omega_w,
        )
        self._apply_world_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)
        self._apply_lumped_drag_wrench(base_pos, rb, rb_inv, v_com_w)

    def _compute_visual_lift_rotor_speed(self, rotor_idx: int, armed: bool) -> float:
        physical_speed = max(0.0, float(self._lift_rotor_angular_velocity[rotor_idx]))
        actuator_output = float(np.clip(self._applied_actuator_controls[rotor_idx], 0.0, 1.0))
        return idle_visual_speed_target(
            physical_speed=physical_speed,
            actuator_output=actuator_output,
            armed=armed,
            idle_speed=self._lift_params.idle_visual_speed,
            low_speed_blend_end=self._lift_params.low_speed_blend_end,
        )

    def _update_vehicle_visuals(self) -> None:
        super()._update_vehicle_visuals()
        armed = self._px4_armed_cached
        dt_s = self._mj_model.opt.timestep
        for i, mocap_id in enumerate(self._lift_rotor_mocap_ids):
            if mocap_id < 0:
                continue
            target_speed = self._compute_visual_lift_rotor_speed(i, armed)
            if self._lift_params.visual_speed_smoothing_tc > 0.0:
                delta = float(target_speed - self._visual_lift_rotor_angular_velocity[i])
                self._visual_lift_rotor_angular_velocity[i] += delta * (
                    1.0 - np.exp(-dt_s / self._lift_params.visual_speed_smoothing_tc)
                )
            else:
                self._visual_lift_rotor_angular_velocity[i] = target_speed
            self._lift_rotor_angle[i] += (
                float(self._visual_lift_rotor_angular_velocity[i]) * self._lift_rotor_direction[i] * dt_s
            )
            body_rot = Rotation.from_quat(self._mj_data.xquat[self._lift_rotor_body_ids[i]].copy(), scalar_first=True)
            body_pos = self._mj_data.xpos[self._lift_rotor_body_ids[i]].copy()
            spin = Rotation.from_rotvec(np.array([0.0, 0.0, 1.0], dtype=float) * self._lift_rotor_angle[i])
            self._mj_data.mocap_pos[mocap_id] = body_pos + body_rot.apply(self._lift_rotor_visual_relative_offsets[i])
            self._mj_data.mocap_quat[mocap_id] = (body_rot * self._lift_rotor_visual_relative_rot[i] * spin).as_quat(
                scalar_first=True
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
