"""MuJoCo multicopter environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.utils.dynamics import LumpedDragParams, first_order_response_step, idle_visual_speed_target


@dataclass
class MCParams:
    """Rotor and aerodynamic parameters that directly affect vehicle dynamics."""

    rotor_direction: np.ndarray
    motor_constant: float
    moment_constant: float
    rotor_drag_coeff: float
    rolling_moment_coeff: float
    rotor_radius: float
    time_constant_up: float
    time_constant_down: float
    max_rot_velocity: float
    idle_visual_speed: float = 120.0
    low_speed_blend_end: float = 180.0
    visual_speed_smoothing_tc: float = 0.02


class MCEnv(PX4MJEnv):
    """MuJoCo multicopter backend with PX4 HIL sensor and actuator integration."""

    def __init__(self, config_loader: ConfigLoader):
        config = config_loader.get_asset_params().get("mc", config_loader.get_asset_params())
        self._params = MCParams(
            rotor_direction=np.array(config["rotor_direction"], dtype=float),
            motor_constant=float(config["motor_constant"]),
            moment_constant=float(config["moment_constant"]),
            rotor_drag_coeff=float(config["rotor_drag_coeff"]),
            rolling_moment_coeff=float(config["rolling_moment_coeff"]),
            rotor_radius=float(config["rotor_radius"]),
            time_constant_up=float(config.get("time_constant_up")),
            time_constant_down=float(config.get("time_constant_down")),
            max_rot_velocity=float(config.get("max_rot_velocity")),
        )
        self._lumped_drag_params = LumpedDragParams.from_config(config.get("lumped_drag"))
        super().__init__(config_loader)

    def _initialize_vehicle_handles(self) -> None:
        self._rotor_body_names, self._rotor_body_ids, self._rotor_indices = self._resolve_named_rotor_bodies(
            allow_visual_fallback=True
        )
        assert self._rotor_body_ids, "No rotor bodies found. Expected rotor_<i> or rotor_<i>_vis bodies."
        self._rotor_mocap_ids, self._rotor_offsets, self._rotor_visual_offsets, self._rotor_mount_rot = (
            self._resolve_visual_rotor_group(
                self._rotor_indices,
                body_ids=self._rotor_body_ids,
            )
        )
        rounded_offsets = {tuple(np.round(offset, decimals=6)) for offset in self._rotor_offsets}
        assert len(rounded_offsets) == len(
            self._rotor_offsets
        ), "Rotor offsets must be unique; duplicate rotor visual/body mapping detected."
        self._rotor_count = len(self._rotor_body_ids)
        self._desired_rotor_angular_velocity = np.zeros(self._rotor_count)
        self._rotor_angular_velocity = np.zeros(self._rotor_count)
        self._visual_rotor_angular_velocity = np.zeros(self._rotor_count)
        self._applied_actuator_controls = np.zeros(self._rotor_count)
        self._rotor_angle = np.zeros(self._rotor_count)
        direction = np.asarray(self._params.rotor_direction, dtype=float)
        if direction.size != self._rotor_count:
            base = np.array([1.0, -1.0])
            direction = np.tile(base, int(np.ceil(self._rotor_count / base.size)))[: self._rotor_count]
        self._rotor_direction = direction

    def _actuator_channel_count(self) -> int:
        return self._rotor_count

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        self._applied_actuator_controls = np.clip(np.asarray(controls, dtype=float), 0.0, 1.0)
        self._desired_rotor_angular_velocity = self._applied_actuator_controls * self._params.max_rot_velocity

    def _update_rotor_speed_state(self, dt_s: float) -> None:
        self._rotor_angular_velocity = first_order_response_step(
            self._rotor_angular_velocity,
            self._desired_rotor_angular_velocity,
            dt_s,
            self._params.time_constant_up,
            self._params.time_constant_down,
        )

    def _compute_rotor_wrenches(
        self,
        base_pos: np.ndarray,
        rb: Rotation,
        rb_inv: Rotation,
        v_com_w: np.ndarray,
        omega_w: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        rotor_positions_w = np.zeros((self._rotor_count, 3))
        rotor_thrusts = np.zeros(self._rotor_count)
        rotor_force_w = np.zeros((self._rotor_count, 3))
        rotor_moment_w = np.zeros((self._rotor_count, 3))
        wind_w = self._get_wind_velocity_w()
        for i in range(self._rotor_count):
            r_off_w = rb.apply(self._rotor_offsets[i])
            pos_w = base_pos + r_off_w
            rotor_positions_w[i] = pos_w
            v_point_w = v_com_w + np.cross(omega_w, r_off_w)
            v_air_point_w = v_point_w - wind_w
            v_point_r = rb_inv.apply(v_air_point_w)
            rotor_axis_w = Rotation.from_quat(
                self._mj_data.xquat[self._rotor_body_ids[i]].copy(), scalar_first=True
            ).apply(np.array([0.0, 0.0, 1.0], dtype=float))
            rotor_axis_r = rb_inv.apply(rotor_axis_w)
            rotor_axis_r = rotor_axis_r / max(np.linalg.norm(rotor_axis_r), 1e-12)
            v_axial = float(np.dot(v_point_r, rotor_axis_r))
            v_perp_r = v_point_r - v_axial * rotor_axis_r

            omega = self._rotor_angular_velocity[i]
            omega_abs = abs(omega)
            direction = self._rotor_direction[i]

            thrust = abs(self._params.motor_constant * omega * omega_abs)
            rotor_thrusts[i] = thrust

            torque_axis_r = -direction * thrust * self._params.moment_constant * rotor_axis_r
            f_drag_r = -self._params.rotor_drag_coeff * omega_abs * v_perp_r
            m_rolling_r = -self._params.rolling_moment_coeff * omega_abs * direction * v_perp_r

            rotor_force_w[i] = rb.apply(rotor_axis_r * thrust + f_drag_r)
            rotor_moment_w[i] = rb.apply(torque_axis_r + m_rolling_r)

        return rotor_positions_w, rotor_thrusts, rotor_force_w, rotor_moment_w

    def _apply_rotor_wrenches(
        self,
        rotor_positions_w: np.ndarray,
        rotor_force_w: np.ndarray,
        rotor_moment_w: np.ndarray,
    ) -> None:
        self._clear_applied_wrenches()
        self._apply_world_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)

    def _apply_vehicle_physics(self) -> None:
        dt_s = self._mj_model.opt.timestep
        self._update_rotor_speed_state(dt_s)
        base_pos, _, rb, rb_inv, v_com_w, _, omega_w = self._get_base_kinematics()
        rotor_positions_w, _, rotor_force_w, rotor_moment_w = self._compute_rotor_wrenches(
            base_pos, rb, rb_inv, v_com_w, omega_w
        )
        self._apply_rotor_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)
        self._apply_lumped_drag_wrench(base_pos, rb, rb_inv, v_com_w)

    def _compute_visual_rotor_speed(self, rotor_idx: int, armed: bool) -> float:
        physical_speed = max(0.0, float(self._rotor_angular_velocity[rotor_idx]))
        actuator_output = float(self._applied_actuator_controls[rotor_idx])
        return idle_visual_speed_target(
            physical_speed=physical_speed,
            actuator_output=actuator_output,
            armed=armed,
            idle_speed=self._params.idle_visual_speed,
            low_speed_blend_end=self._params.low_speed_blend_end,
        )

    def _update_vehicle_visuals(self) -> None:
        armed = self._px4_transport.update_arming_state()
        target_visual_speeds = np.asarray(
            [self._compute_visual_rotor_speed(i, armed) for i in range(self._rotor_count)],
            dtype=float,
        )
        self._advance_visual_rotors(
            mocap_ids=self._rotor_mocap_ids,
            offsets_b=self._rotor_visual_offsets,
            mount_rot=self._rotor_mount_rot,
            rotor_angles=self._rotor_angle,
            visual_speeds=self._visual_rotor_angular_velocity,
            target_speeds=target_visual_speeds,
            spin_directions=self._rotor_direction,
            spin_axes_local=np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (self._rotor_count, 1)),
            smoothing_tc=self._params.visual_speed_smoothing_tc,
        )

    def _get_visual_rotor_angle(self) -> np.ndarray:
        return self._rotor_angle.copy()

    def _get_visual_rotor_speed(self) -> np.ndarray:
        return self._visual_rotor_angular_velocity.copy()
