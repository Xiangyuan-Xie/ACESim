"""MuJoCo underwater vehicle environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.utils.dynamics import first_order_response_step, idle_visual_speed_target


@dataclass
class UUVParams:
    """Hydrodynamic and thruster parameters for the underwater backend."""

    added_mass_linear: np.ndarray
    added_mass_angular: np.ndarray
    damping_linear: np.ndarray
    damping_angular: np.ndarray
    buoyancy_origin_b: np.ndarray
    buoyancy_compensation: float
    buoyancy_height_scale_limit: float
    water_surface_z_nwu: float
    rotor_direction: np.ndarray
    motor_constant: np.ndarray
    moment_constant: np.ndarray
    time_constant_up: float
    time_constant_down: float
    max_rot_velocity: float
    rotor_axes_b: np.ndarray
    idle_visual_speed: float = 120.0
    low_speed_blend_end: float = 180.0
    visual_speed_smoothing_tc: float = 0.02


class UUVEnv(PX4MJEnv):
    """MuJoCo underwater backend matching PX4 Gazebo UUV plugin semantics."""

    def __init__(self, config_loader: ConfigLoader):
        asset_params = config_loader.get_asset_params()
        config = asset_params.get("uuv", asset_params)
        self._params = UUVParams(
            added_mass_linear=np.asarray(config["added_mass_linear"], dtype=float),
            added_mass_angular=np.asarray(config["added_mass_angular"], dtype=float),
            damping_linear=np.asarray(config["damping_linear"], dtype=float),
            damping_angular=np.asarray(config["damping_angular"], dtype=float),
            buoyancy_origin_b=np.asarray(config.get("buoyancy_origin_b", [0.0, 0.0, 0.0]), dtype=float),
            buoyancy_compensation=float(config.get("buoyancy_compensation", 1.0)),
            buoyancy_height_scale_limit=float(config.get("buoyancy_height_scale_limit", 0.05)),
            water_surface_z_nwu=float(config.get("water_surface_z_nwu", 10.0)),
            rotor_direction=np.asarray(config["rotor_direction"], dtype=float),
            motor_constant=np.asarray(config["motor_constant"], dtype=float),
            moment_constant=np.asarray(config["moment_constant"], dtype=float),
            time_constant_up=float(config.get("time_constant_up", 0.0125)),
            time_constant_down=float(config.get("time_constant_down", 0.025)),
            max_rot_velocity=float(config.get("max_rot_velocity", 1100.0)),
            rotor_axes_b=np.asarray(config["rotor_axes_b"], dtype=float),
        )
        super().__init__(config_loader)

    def _initialize_vehicle_handles(self) -> None:
        self._rotor_body_names, self._rotor_body_ids, self._rotor_indices = self._resolve_named_rotor_bodies(
            allow_visual_fallback=False
        )
        assert self._rotor_body_ids, "UUV asset must define rotor_<i> bodies"
        self._rotor_mocap_ids, self._rotor_offsets, self._rotor_visual_offsets, self._rotor_mount_rot = (
            self._resolve_visual_rotor_group(
                self._rotor_indices,
                body_ids=self._rotor_body_ids,
            )
        )
        self._rotor_count = len(self._rotor_body_ids)
        assert self._params.rotor_direction.size == self._rotor_count
        assert self._params.motor_constant.size == self._rotor_count
        assert self._params.moment_constant.size == self._rotor_count
        assert self._params.rotor_axes_b.shape == (self._rotor_count, 3)
        self._rotor_axes_b = self._params.rotor_axes_b / np.linalg.norm(
            self._params.rotor_axes_b, axis=1, keepdims=True
        )
        self._desired_rotor_angular_velocity = np.zeros(self._rotor_count, dtype=float)
        self._rotor_angular_velocity = np.zeros(self._rotor_count, dtype=float)
        self._visual_rotor_angular_velocity = np.zeros(self._rotor_count, dtype=float)
        self._applied_actuator_controls = np.zeros(self._rotor_count, dtype=float)
        self._rotor_angle = np.zeros(self._rotor_count, dtype=float)

    def _actuator_channel_count(self) -> int:
        return self._rotor_count

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        self._applied_actuator_controls = np.clip(np.asarray(controls, dtype=float), -1.0, 1.0)
        self._desired_rotor_angular_velocity = self._applied_actuator_controls * self._params.max_rot_velocity

    def _update_rotor_speed_state(self, dt_s: float) -> None:
        self._rotor_angular_velocity = first_order_response_step(
            self._rotor_angular_velocity,
            self._desired_rotor_angular_velocity,
            dt_s,
            self._params.time_constant_up,
            self._params.time_constant_down,
        )

    def _compute_hydrodynamic_wrench(
        self, body_velocity_flu: np.ndarray, body_rates_flu: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        u, v, w = body_velocity_flu
        p, q, r = body_rates_flu
        damping_force = -self._params.damping_linear * body_velocity_flu
        damping_torque = -self._params.damping_angular * body_rates_flu

        x_udot, y_vdot, z_wdot = self._params.added_mass_linear
        k_pdot, m_qdot, n_rdot = self._params.added_mass_angular
        c_ad_fa = np.array(
            [
                [0.0, z_wdot * w, -y_vdot * v],
                [-z_wdot * w, 0.0, x_udot * u],
                [y_vdot * v, -x_udot * u, 0.0],
            ],
            dtype=float,
        )
        c_ad_ta = np.array(
            [
                [0.0, n_rdot * r, -m_qdot * q],
                [-n_rdot * r, 0.0, k_pdot * p],
                [m_qdot * q, -k_pdot * p, 0.0],
            ],
            dtype=float,
        )
        coriolis_force = c_ad_fa @ body_rates_flu
        coriolis_torque = c_ad_fa @ body_velocity_flu + c_ad_ta @ body_rates_flu
        return damping_force + coriolis_force, damping_torque + coriolis_torque

    def _compute_buoyancy_force(self, rb: Rotation) -> tuple[np.ndarray, np.ndarray]:
        mass = float(self._mj_model.body_mass[self._base_link_id])
        buoyancy_force_world = self._params.buoyancy_compensation * mass * np.array([0.0, 0.0, 9.81], dtype=float)
        cob_world = self._get_sensor_raw("pos") + rb.apply(self._params.buoyancy_origin_b)
        scale = abs(
            (cob_world[2] - (self._params.water_surface_z_nwu - self._params.buoyancy_height_scale_limit))
            / (2.0 * self._params.buoyancy_height_scale_limit)
        )
        if cob_world[2] > self._params.water_surface_z_nwu + self._params.buoyancy_height_scale_limit:
            scale = 0.0
        scale = float(np.clip(scale, 0.0, 1.0))
        force_world = buoyancy_force_world * scale
        return rb.inv().apply(force_world), self._params.buoyancy_origin_b.copy()

    def _compute_thruster_wrenches(
        self,
        base_pos: np.ndarray,
        rb: Rotation,
        body_velocity_flu: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        rotor_positions_w = np.zeros((self._rotor_count, 3), dtype=float)
        rotor_force_w = np.zeros_like(rotor_positions_w)
        rotor_moment_w = np.zeros_like(rotor_positions_w)
        for i in range(self._rotor_count):
            omega = self._rotor_angular_velocity[i]
            force_scalar = omega * abs(omega) * self._params.motor_constant[i]
            axis_b = self._rotor_axes_b[i]
            force_b = axis_b * force_scalar
            torque_b = -self._params.rotor_direction[i] * force_scalar * self._params.moment_constant[i] * axis_b
            rotor_positions_w[i] = base_pos + rb.apply(self._rotor_offsets[i])
            rotor_force_w[i] = rb.apply(force_b)
            rotor_moment_w[i] = rb.apply(torque_b)
        return rotor_positions_w, rotor_force_w, rotor_moment_w

    def _apply_vehicle_physics(self) -> None:
        self._clear_applied_wrenches()
        dt_s = self._mj_model.opt.timestep
        self._update_rotor_speed_state(dt_s)
        base_pos, _, rb, rb_inv, v_com_w, body_rates_flu, _ = self._get_base_kinematics()
        body_velocity_flu = rb_inv.apply(v_com_w)
        hydro_force_b, hydro_torque_b = self._compute_hydrodynamic_wrench(body_velocity_flu, body_rates_flu)
        self._apply_body_wrench(hydro_force_b, hydro_torque_b)
        buoyancy_force_b, buoyancy_point_b = self._compute_buoyancy_force(rb)
        self._apply_body_wrench(buoyancy_force_b, np.zeros(3, dtype=float), buoyancy_point_b)
        rotor_positions_w, rotor_force_w, rotor_moment_w = self._compute_thruster_wrenches(
            base_pos, rb, body_velocity_flu
        )
        self._apply_world_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)

    def _compute_visual_rotor_speed(self, rotor_idx: int, armed: bool) -> float:
        physical_speed = abs(float(self._rotor_angular_velocity[rotor_idx]))
        actuator_output = abs(float(self._applied_actuator_controls[rotor_idx]))
        return idle_visual_speed_target(
            physical_speed=physical_speed,
            actuator_output=actuator_output,
            armed=armed,
            idle_speed=self._params.idle_visual_speed,
            low_speed_blend_end=self._params.low_speed_blend_end,
        )

    def _update_vehicle_visuals(self) -> None:
        armed = self._px4_transport.update_arming_state()
        target_speeds = np.asarray(
            [self._compute_visual_rotor_speed(i, armed) for i in range(self._rotor_count)],
            dtype=float,
        )
        spin_direction = np.where(self._rotor_angular_velocity >= 0.0, 1.0, -1.0)
        self._advance_visual_rotors(
            mocap_ids=self._rotor_mocap_ids,
            offsets_b=self._rotor_visual_offsets,
            mount_rot=self._rotor_mount_rot,
            rotor_angles=self._rotor_angle,
            visual_speeds=self._visual_rotor_angular_velocity,
            target_speeds=target_speeds,
            spin_directions=spin_direction,
            spin_axes_local=np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (self._rotor_count, 1)),
            smoothing_tc=self._params.visual_speed_smoothing_tc,
        )

    def _get_visual_rotor_angle(self) -> np.ndarray:
        return self._rotor_angle.copy()

    def _get_visual_rotor_speed(self) -> np.ndarray:
        return self._visual_rotor_angular_velocity.copy()
