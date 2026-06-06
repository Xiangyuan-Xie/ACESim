"""MuJoCo fixed-wing environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.utils.dynamics import first_order_response_step, idle_visual_speed_target


@dataclass
class SurfaceControlSpec:
    joint_name: str
    direction: float
    cd_ctrl: float
    cy_ctrl: float
    cl_ctrl: float
    cell_ctrl: float
    cem_ctrl: float
    cen_ctrl: float


@dataclass
class AdvancedAeroParams:
    alpha0: float
    cl0: float
    cla: float
    clb: float
    cyb: float
    cd0: float
    cem0: float
    cema: float
    cemb: float
    cella: float
    cellb: float
    cena: float
    cenb: float
    cdp: float
    cyp: float
    clp: float
    cellp: float
    cemp: float
    cenp: float
    cdq: float
    cyq: float
    clq: float
    cellq: float
    cemq: float
    cenq: float
    cdr: float
    cyr: float
    clr: float
    cellr: float
    cemr: float
    cenr: float
    alpha_stall: float
    area: float
    aspect_ratio: float
    efficiency: float
    mac: float
    ref_pt: np.ndarray
    sigmoid_m: float
    cd_fp_k1: float
    cd_fp_k2: float
    control_surfaces: list[SurfaceControlSpec]


@dataclass
class LiftSurfaceParams:
    joint_name: str
    cp: np.ndarray
    area: float
    a0: float
    cla: float
    cda: float
    cma: float
    alpha_stall: float
    cla_stall: float
    cda_stall: float
    cma_stall: float
    control_joint_rad_to_cl: float
    cm_delta: float = 0.0


@dataclass
class FWParams:
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
    model: str = "advanced_plane"
    idle_visual_speed: float = 120.0
    low_speed_blend_end: float = 180.0
    visual_speed_smoothing_tc: float = 0.02


class FWEnv(PX4MJEnv):
    """MuJoCo fixed-wing backend with PX4-classic aerodynamic semantics."""

    def __init__(self, config_loader: ConfigLoader):
        asset_params = config_loader.get_asset_params()
        config = asset_params.get("fw", asset_params)
        control_config = config.get("controls", {})
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
            linear_damping=float(config.get("linear_damping", 0.0)),
            angular_damping=float(config.get("angular_damping", 0.0)),
            model=str(config.get("model", "advanced_plane")),
        )
        self._throttle_control_index = int(control_config.get("throttle_channel_index", 4))
        self._surface_control_indices = np.asarray(
            control_config.get("surface_channel_indices", [2, 3, 8, 5, 6, 7]),
            dtype=int,
        )
        self._surface_control_signs = np.asarray(
            control_config.get("surface_channel_signs", [1.0, -1.0, -1.0, 1.0, 1.0, -1.0]),
            dtype=float,
        )
        self._advanced_params = self._load_advanced_params(config.get("advanced"))
        self._lift_surface_params = self._load_lift_surface_params(config.get("lift_surfaces", []))
        super().__init__(config_loader)

    def _load_advanced_params(self, value: object) -> AdvancedAeroParams | None:
        if not isinstance(value, dict):
            return None
        control_surfaces = [
            SurfaceControlSpec(
                joint_name=str(item["joint_name"]),
                direction=float(item.get("direction", 1.0)),
                cd_ctrl=float(item.get("cd_ctrl", 0.0)),
                cy_ctrl=float(item.get("cy_ctrl", 0.0)),
                cl_ctrl=float(item.get("cl_ctrl", 0.0)),
                cell_ctrl=float(item.get("cell_ctrl", 0.0)),
                cem_ctrl=float(item.get("cem_ctrl", 0.0)),
                cen_ctrl=float(item.get("cen_ctrl", 0.0)),
            )
            for item in value.get("control_surfaces", [])
        ]
        return AdvancedAeroParams(
            alpha0=float(value.get("alpha0", 0.0)),
            cl0=float(value.get("cl0", 0.0)),
            cla=float(value.get("cla", 0.0)),
            clb=float(value.get("clb", 0.0)),
            cyb=float(value.get("cyb", 0.0)),
            cd0=float(value.get("cd0", 0.0)),
            cem0=float(value.get("cem0", 0.0)),
            cema=float(value.get("cema", 0.0)),
            cemb=float(value.get("cemb", 0.0)),
            cella=float(value.get("cella", 0.0)),
            cellb=float(value.get("cellb", 0.0)),
            cena=float(value.get("cena", 0.0)),
            cenb=float(value.get("cenb", 0.0)),
            cdp=float(value.get("cdp", 0.0)),
            cyp=float(value.get("cyp", 0.0)),
            clp=float(value.get("clp", 0.0)),
            cellp=float(value.get("cellp", 0.0)),
            cemp=float(value.get("cemp", 0.0)),
            cenp=float(value.get("cenp", 0.0)),
            cdq=float(value.get("cdq", 0.0)),
            cyq=float(value.get("cyq", 0.0)),
            clq=float(value.get("clq", 0.0)),
            cellq=float(value.get("cellq", 0.0)),
            cemq=float(value.get("cemq", 0.0)),
            cenq=float(value.get("cenq", 0.0)),
            cdr=float(value.get("cdr", 0.0)),
            cyr=float(value.get("cyr", 0.0)),
            clr=float(value.get("clr", 0.0)),
            cellr=float(value.get("cellr", 0.0)),
            cemr=float(value.get("cemr", 0.0)),
            cenr=float(value.get("cenr", 0.0)),
            alpha_stall=float(value.get("alpha_stall", 0.3391428111)),
            area=float(value.get("area", 0.34)),
            aspect_ratio=float(value.get("aspect_ratio", 6.5)),
            efficiency=float(value.get("efficiency", 0.97)),
            mac=float(value.get("mac", 0.22)),
            ref_pt=np.asarray(value.get("ref_pt", [0.0, 0.0, 0.0]), dtype=float),
            sigmoid_m=float(value.get("sigmoid_m", 15.0)),
            cd_fp_k1=float(value.get("cd_fp_k1", -3.0)),
            cd_fp_k2=float(value.get("cd_fp_k2", -0.25)),
            control_surfaces=control_surfaces,
        )

    def _load_lift_surface_params(self, values: object) -> list[LiftSurfaceParams]:
        if not isinstance(values, list):
            return []
        return [
            LiftSurfaceParams(
                joint_name=str(item["joint_name"]),
                cp=np.asarray(item["cp"], dtype=float),
                area=float(item["area"]),
                a0=float(item.get("a0", 0.0)),
                cla=float(item.get("cla", 0.0)),
                cda=float(item.get("cda", 0.0)),
                cma=float(item.get("cma", 0.0)),
                alpha_stall=float(item.get("alpha_stall", 0.3391428111)),
                cla_stall=float(item.get("cla_stall", -3.85)),
                cda_stall=float(item.get("cda_stall", -0.9233984055)),
                cma_stall=float(item.get("cma_stall", 0.0)),
                control_joint_rad_to_cl=float(item.get("control_joint_rad_to_cl", 0.0)),
                cm_delta=float(item.get("cm_delta", 0.0)),
            )
            for item in values
        ]

    def _initialize_vehicle_handles(self) -> None:
        self._puller_rotor_index = int(getattr(self, "_puller_rotor_index", 4))
        self._puller_body_id = -1
        self._puller_mocap_id = -1
        self._puller_offset = np.zeros(3, dtype=float)
        self._puller_visual_offset = np.zeros(3, dtype=float)
        self._puller_mount_rot = Rotation.identity()
        self._puller_spin_axis_local = np.array([1.0, 0.0, 0.0], dtype=float)
        if self._puller_rotor_index >= 0:
            self._puller_body_id = mujoco.mj_name2id(
                self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{self._puller_rotor_index}"
            )
            assert self._puller_body_id >= 0, "Fixed-wing asset must define puller rotor body"
            puller_mocap_ids, puller_offsets, puller_visual_offsets, puller_mount_rot = (
                self._resolve_visual_rotor_group(
                    [self._puller_rotor_index],
                    body_ids=[self._puller_body_id],
                )
            )
            self._puller_mocap_id = puller_mocap_ids[0]
            self._puller_offset = puller_offsets[0].copy()
            self._puller_visual_offset = puller_visual_offsets[0].copy()
            self._puller_mount_rot = puller_mount_rot[0]
            # Visual props may have a static mount rotation relative to the rotor
            # body. Spin in the mount-local axis that maps back to body +x so the
            # disc stays normal to the physical puller axis at runtime.
            self._puller_spin_axis_local = self._puller_mount_rot.inv().apply(np.array([1.0, 0.0, 0.0], dtype=float))
            norm = np.linalg.norm(self._puller_spin_axis_local)
            if norm > 1e-9:
                self._puller_spin_axis_local /= norm
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
        self._surface_ctrl_center = np.zeros(len(self._surface_joint_names), dtype=float)
        self._surface_ctrl_halfspan = np.ones(len(self._surface_joint_names), dtype=float)
        for i, actuator_id in enumerate(self._surface_actuator_ids):
            if actuator_id < 0:
                continue
            lower, upper = self._mj_model.actuator_ctrlrange[actuator_id]
            self._surface_ctrl_center[i] = 0.5 * float(lower + upper)
            self._surface_ctrl_halfspan[i] = 0.5 * float(upper - lower)
        self._surface_targets = self._surface_ctrl_center.copy()
        self._joint_target_map = {name: i for i, name in enumerate(self._surface_joint_names)}
        self._last_true_airspeed_mps = 0.0
        self._last_diff_pressure_hpa = 0.0
        self._last_air_density = self._get_medium_density_kg_m3()

    def _required_surface_joint_names(self) -> list[str]:
        return self._surface_joint_names

    def _required_surface_actuator_names(self) -> list[str]:
        return self._surface_actuator_names

    def _actuator_channel_count(self) -> int:
        return 9

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        self._applied_actuator_controls = np.asarray(controls, dtype=float)
        self._surface_targets[:] = self._surface_ctrl_center
        for i, (channel_idx, sign) in enumerate(zip(self._surface_control_indices, self._surface_control_signs)):
            if 0 <= int(channel_idx) < len(self._applied_actuator_controls):
                normalized = float(np.clip(sign * self._applied_actuator_controls[int(channel_idx)], -1.0, 1.0))
                self._surface_targets[i] = self._surface_ctrl_center[i] + normalized * self._surface_ctrl_halfspan[i]
        if 0 <= self._throttle_control_index < len(self._applied_actuator_controls):
            self._desired_puller_angular_velocity = (
                float(np.clip(self._applied_actuator_controls[self._throttle_control_index], 0.0, 1.0))
                * self._params.thrust_max_rot_velocity
            )
        else:
            self._desired_puller_angular_velocity = 0.0

    def _apply_surface_targets(self) -> None:
        for target, act_id in zip(self._surface_targets, self._surface_actuator_ids):
            if act_id >= 0:
                self._mj_data.ctrl[act_id] = target

    def _update_puller_speed(self, dt_s: float) -> None:
        self._puller_angular_velocity = first_order_response_step(
            self._puller_angular_velocity,
            self._desired_puller_angular_velocity,
            dt_s,
            self._params.thrust_time_constant_up,
            self._params.thrust_time_constant_down,
        )

    def _compute_propeller_force(self, body_velocity_flu: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self._puller_body_id < 0:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)
        omega = self._puller_angular_velocity
        omega_abs = abs(omega)
        thrust = abs(self._params.thrust_motor_constant * omega * omega_abs)
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
        v_body = rb_inv.apply(v_com_w - self._get_wind_velocity_w())
        altitude_m = float(self._px4_sensor_params.gps_alt_start + self._get_sensor_raw("pos")[2])
        return v_body, altitude_m

    def _body_axes_world(self, rb: Rotation) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        body_x = rb.apply(np.array([1.0, 0.0, 0.0], dtype=float))
        body_z_down = rb.apply(np.array([0.0, 0.0, -1.0], dtype=float))
        body_y_right = np.cross(body_z_down, body_x)
        body_y_right /= max(np.linalg.norm(body_y_right), 1e-12)
        return body_x, body_y_right, body_z_down

    def _surface_angle(self, joint_name: str) -> float:
        idx = self._joint_target_map.get(joint_name)
        if idx is None or self._surface_joint_ids[idx] < 0:
            return 0.0
        return float(self._mj_data.qpos[self._mj_model.jnt_qposadr[self._surface_joint_ids[idx]]])

    def _surface_angle_deg(self, joint_name: str) -> float:
        return float(np.degrees(self._surface_angle(joint_name)))

    def _update_airspeed_state(self, rho: float, body_velocity_flu: np.ndarray) -> None:
        true_airspeed_mps = max(0.0, float(body_velocity_flu[0]))
        self._last_true_airspeed_mps = true_airspeed_mps
        self._last_air_density = rho
        self._last_diff_pressure_hpa = 0.5 * rho * true_airspeed_mps * true_airspeed_mps * 0.01

    def _compute_advanced_aero_wrench(self) -> tuple[np.ndarray, np.ndarray]:
        assert self._advanced_params is not None
        _, _, rb, rb_inv, v_com_w, gyro_flu, _ = self._get_base_kinematics()
        air_velocity_w = v_com_w - self._get_wind_velocity_w()
        body_velocity_flu = rb_inv.apply(air_velocity_w)
        air_density = self._get_medium_density_kg_m3()
        self._update_airspeed_state(air_density, body_velocity_flu)

        # Keep the advanced fixed-wing model expressed in world/stability axes so
        # the fitted coefficients continue to match PX4's SIH-style semantics.
        body_x, body_y_right, body_z_down = self._body_axes_world(rb)
        air_velocity = air_velocity_w
        vel_in_ld_plane = air_velocity - np.dot(air_velocity, body_y_right) * body_y_right
        speed_in_ld_plane = float(np.linalg.norm(vel_in_ld_plane))
        if speed_in_ld_plane <= 1e-6:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        stability_x = vel_in_ld_plane / speed_in_ld_plane
        stability_y = body_y_right
        stability_z = np.cross(stability_x, stability_y)
        stability_z /= max(np.linalg.norm(stability_z), 1e-12)
        rr, pr, yr = np.array([gyro_flu[0], -gyro_flu[1], -gyro_flu[2]], dtype=float)

        stabx_proj_bodyx = float(np.dot(stability_x, body_x))
        stabx_proj_bodyz = float(np.dot(stability_x, body_z_down))
        alpha = float(np.arctan2(stabx_proj_bodyz, stabx_proj_bodyx))
        beta = float(np.arctan2(np.dot(air_velocity, body_y_right), np.dot(air_velocity, body_x)))
        dyn_pres = 0.5 * air_density * speed_in_ld_plane * speed_in_ld_plane
        half_rho_vel = 0.5 * air_density * speed_in_ld_plane
        span = float(np.sqrt(self._advanced_params.area * self._advanced_params.aspect_ratio))
        mac = self._advanced_params.mac if self._advanced_params.mac > 0.0 else self._advanced_params.area / span

        # Blend the linear small-angle model with the post-stall approximation
        # instead of switching abruptly at ``alpha_stall``.
        exp_pos = np.exp(-self._advanced_params.sigmoid_m * (alpha - self._advanced_params.alpha_stall))
        exp_neg = np.exp(self._advanced_params.sigmoid_m * (alpha + self._advanced_params.alpha_stall))
        sigma = (1.0 + exp_pos + exp_neg) / ((1.0 + exp_pos) * (1.0 + exp_neg))
        alpha_sign = 1.0 if alpha >= 0.0 else -1.0

        cl = (1.0 - sigma) * (self._advanced_params.cl0 + self._advanced_params.cla * alpha) + sigma * (
            2.0 * alpha_sign * np.sin(alpha) * np.sin(alpha) * np.cos(alpha)
        )
        cl += self._advanced_params.clb * beta

        cd_ctrl_tot = 0.0
        cy_ctrl_tot = 0.0
        cl_ctrl_tot = 0.0
        cell_ctrl_tot = 0.0
        cem_ctrl_tot = 0.0
        cen_ctrl_tot = 0.0
        for surface in self._advanced_params.control_surfaces:
            control_angle_deg = self._surface_angle_deg(surface.joint_name)
            control_scale = control_angle_deg * surface.direction
            cd_ctrl_tot += control_scale * surface.cd_ctrl
            cy_ctrl_tot += control_scale * surface.cy_ctrl
            cl_ctrl_tot += control_scale * surface.cl_ctrl
            cell_ctrl_tot += control_scale * surface.cell_ctrl
            cem_ctrl_tot += control_scale * surface.cem_ctrl
            cen_ctrl_tot += control_scale * surface.cen_ctrl

        cl += cl_ctrl_tot
        lift_world = (
            cl * dyn_pres
            + self._advanced_params.clp * (rr * span / 2.0) * half_rho_vel
            + self._advanced_params.clq * (pr * mac / 2.0) * half_rho_vel
            + self._advanced_params.clr * (yr * span / 2.0) * half_rho_vel
        ) * (self._advanced_params.area * (-stability_z))

        cd_fp = 2.0 / (
            1.0
            + np.exp(
                self._advanced_params.cd_fp_k1
                + self._advanced_params.cd_fp_k2
                * max(self._advanced_params.aspect_ratio, 1.0 / self._advanced_params.aspect_ratio)
            )
        )
        cd = (1.0 - sigma) * (
            self._advanced_params.cd0
            + (cl * cl) / (np.pi * self._advanced_params.aspect_ratio * self._advanced_params.efficiency)
        ) + sigma * abs(cd_fp * (0.5 - 0.5 * np.cos(2.0 * alpha)))
        cd += cd_ctrl_tot
        drag_world = (
            cd * dyn_pres
            + self._advanced_params.cdp * (rr * span / 2.0) * half_rho_vel
            + self._advanced_params.cdq * (pr * mac / 2.0) * half_rho_vel
            + self._advanced_params.cdr * (yr * span / 2.0) * half_rho_vel
        ) * (self._advanced_params.area * (-stability_x))

        cy = self._advanced_params.cyb * beta + cy_ctrl_tot
        sideforce_world = (
            cy * dyn_pres
            + self._advanced_params.cyp * (rr * span / 2.0) * half_rho_vel
            + self._advanced_params.cyq * (pr * mac / 2.0) * half_rho_vel
            + self._advanced_params.cyr * (yr * span / 2.0) * half_rho_vel
        ) * (self._advanced_params.area * stability_y)

        if alpha > self._advanced_params.alpha_stall:
            cem = (
                self._advanced_params.cem0
                + (self._advanced_params.cema * self._advanced_params.alpha_stall)
                + 0.0 * (alpha - self._advanced_params.alpha_stall)
            )
        elif alpha < -self._advanced_params.alpha_stall:
            cem = (
                self._advanced_params.cem0
                + (-self._advanced_params.cema * self._advanced_params.alpha_stall)
                + 0.0 * (alpha + self._advanced_params.alpha_stall)
            )
        else:
            cem = self._advanced_params.cem0 + self._advanced_params.cema * alpha
        cem += self._advanced_params.cemb * beta + cem_ctrl_tot

        pm_world = (
            cem * dyn_pres
            + self._advanced_params.cemp * (rr * span / 2.0) * half_rho_vel
            + self._advanced_params.cemq * (pr * mac / 2.0) * half_rho_vel
            + self._advanced_params.cemr * (yr * span / 2.0) * half_rho_vel
        ) * (self._advanced_params.area * mac * body_y_right)

        cell = self._advanced_params.cella * alpha + self._advanced_params.cellb * beta + cell_ctrl_tot
        rm_world = (
            cell * dyn_pres
            + self._advanced_params.cellp * (rr * span / 2.0) * half_rho_vel
            + self._advanced_params.cellq * (pr * mac / 2.0) * half_rho_vel
            + self._advanced_params.cellr * (yr * span / 2.0) * half_rho_vel
        ) * (self._advanced_params.area * span * body_x)

        cen = self._advanced_params.cena * alpha + self._advanced_params.cenb * beta + cen_ctrl_tot
        ym_world = (
            cen * dyn_pres
            + self._advanced_params.cenp * (rr * span / 2.0) * half_rho_vel
            + self._advanced_params.cenq * (pr * mac / 2.0) * half_rho_vel
            + self._advanced_params.cenr * (yr * span / 2.0) * half_rho_vel
        ) * (self._advanced_params.area * span * body_z_down)

        force_world = lift_world + drag_world + sideforce_world
        ref_point_world = rb.apply(self._advanced_params.ref_pt)
        # Aerodynamic coefficients produce moments around the aerodynamic
        # reference point, then the force result is shifted back to base_link.
        moment_world = pm_world + rm_world + ym_world + np.cross(ref_point_world, force_world)
        return rb_inv.apply(force_world), rb_inv.apply(moment_world)

    def _compute_lift_drag_aero_wrench(self) -> tuple[np.ndarray, np.ndarray]:
        _, _, rb, rb_inv, v_com_w, _, omega_w = self._get_base_kinematics()
        wind_w = self._get_wind_velocity_w()
        body_velocity_flu = rb_inv.apply(v_com_w - wind_w)
        air_density = self._get_medium_density_kg_m3()
        self._update_airspeed_state(air_density, body_velocity_flu)

        forward_world = rb.apply(np.array([1.0, 0.0, 0.0], dtype=float))
        upward_world = rb.apply(np.array([0.0, 0.0, 1.0], dtype=float))
        total_force_world = np.zeros(3, dtype=float)
        total_moment_world = np.zeros(3, dtype=float)

        for surface in self._lift_surface_params:
            cp_world = rb.apply(surface.cp)
            vel_world = v_com_w + np.cross(omega_w, cp_world) - wind_w
            speed = float(np.linalg.norm(vel_world))
            if speed <= 0.01:
                continue
            if np.dot(forward_world, vel_world) <= 0.0:
                continue

            vel_unit = vel_world / speed
            spanwise_world = np.cross(forward_world, upward_world)
            spanwise_world /= max(np.linalg.norm(spanwise_world), 1e-12)
            sin_sweep = float(np.clip(np.dot(spanwise_world, vel_unit), -1.0, 1.0))
            sweep = float(np.arcsin(sin_sweep))
            while abs(sweep) > 0.5 * np.pi:
                sweep = sweep - np.pi if sweep > 0.0 else sweep + np.pi
            cos_sweep = float(np.sqrt(max(0.0, 1.0 - np.sin(sweep) ** 2)))

            vel_in_ld_plane = vel_world - np.dot(vel_world, spanwise_world) * spanwise_world
            speed_in_ld_plane = float(np.linalg.norm(vel_in_ld_plane))
            if speed_in_ld_plane <= 1e-9:
                continue
            # Each lifting surface resolves its own local lift/drag plane so a
            # single asset can mix wings, flaps, and control surfaces cleanly.
            drag_direction = -vel_in_ld_plane / speed_in_ld_plane
            lift_direction = np.cross(spanwise_world, vel_in_ld_plane)
            lift_direction /= max(np.linalg.norm(lift_direction), 1e-12)
            cos_alpha = float(np.clip(np.dot(lift_direction, upward_world), -1.0, 1.0))
            alpha = (
                surface.a0 + float(np.arccos(cos_alpha))
                if np.dot(lift_direction, forward_world) >= 0.0
                else surface.a0 - float(np.arccos(cos_alpha))
            )
            while abs(alpha) > 0.5 * np.pi:
                alpha = alpha - np.pi if alpha > 0.0 else alpha + np.pi

            dyn_pres = 0.5 * air_density * speed_in_ld_plane * speed_in_ld_plane
            if alpha > surface.alpha_stall:
                cl = (surface.cla * surface.alpha_stall + surface.cla_stall * (alpha - surface.alpha_stall)) * cos_sweep
                cl = max(0.0, cl)
            elif alpha < -surface.alpha_stall:
                cl = (
                    -surface.cla * surface.alpha_stall + surface.cla_stall * (alpha + surface.alpha_stall)
                ) * cos_sweep
                cl = min(0.0, cl)
            else:
                cl = surface.cla * alpha * cos_sweep
            control_angle = self._surface_angle(surface.joint_name)
            cl += surface.control_joint_rad_to_cl * control_angle
            lift_world = cl * dyn_pres * surface.area * lift_direction

            if alpha > surface.alpha_stall:
                cd = (surface.cda * surface.alpha_stall + surface.cda_stall * (alpha - surface.alpha_stall)) * cos_sweep
            elif alpha < -surface.alpha_stall:
                cd = (
                    -surface.cda * surface.alpha_stall + surface.cda_stall * (alpha + surface.alpha_stall)
                ) * cos_sweep
            else:
                cd = surface.cda * alpha * cos_sweep
            cd = abs(cd)
            drag_world = cd * dyn_pres * surface.area * drag_direction

            if alpha > surface.alpha_stall:
                cm = (surface.cma * surface.alpha_stall + surface.cma_stall * (alpha - surface.alpha_stall)) * cos_sweep
                cm = max(0.0, cm)
            elif alpha < -surface.alpha_stall:
                cm = (
                    -surface.cma * surface.alpha_stall + surface.cma_stall * (alpha + surface.alpha_stall)
                ) * cos_sweep
                cm = min(0.0, cm)
            else:
                cm = surface.cma * alpha * cos_sweep
            cm += surface.cm_delta * control_angle
            moment_world = cm * dyn_pres * surface.area * spanwise_world

            force_world = lift_world + drag_world
            total_force_world += force_world
            total_moment_world += moment_world + np.cross(cp_world, force_world)

        total_force_world += rb.apply(-self._params.linear_damping * body_velocity_flu)
        return rb_inv.apply(total_force_world), rb_inv.apply(total_moment_world)

    def _compute_aero_wrench(self) -> tuple[np.ndarray, np.ndarray]:
        if self._params.model == "advanced_plane":
            return self._compute_advanced_aero_wrench()
        if self._params.model == "standard_vtol":
            return self._compute_lift_drag_aero_wrench()
        raise ValueError(f"Unsupported fixed-wing model: {self._params.model}")

    def _apply_vehicle_physics(self) -> None:
        self._clear_applied_wrenches()
        self._apply_surface_targets()
        dt_s = self._mj_model.opt.timestep
        if self._puller_body_id >= 0:
            self._update_puller_speed(dt_s)
        aero_force_b, aero_moment_b = self._compute_aero_wrench()
        prop_force_b, prop_torque_b = self._compute_propeller_force(self._compute_apparent_body_velocity()[0])
        self._apply_body_wrench(aero_force_b, aero_moment_b)
        if self._puller_body_id >= 0:
            self._apply_body_wrench(prop_force_b, prop_torque_b, self._puller_offset)

    def _compute_visual_prop_speed(self, armed: bool) -> float:
        if self._puller_body_id < 0:
            return 0.0
        physical_speed = max(0.0, float(self._puller_angular_velocity))
        actuator_output = (
            float(np.clip(self._applied_actuator_controls[self._throttle_control_index], 0.0, 1.0))
            if 0 <= self._throttle_control_index < len(self._applied_actuator_controls)
            else 0.0
        )
        return idle_visual_speed_target(
            physical_speed=physical_speed,
            actuator_output=actuator_output,
            armed=armed,
            idle_speed=self._params.idle_visual_speed,
            low_speed_blend_end=self._params.low_speed_blend_end,
        )

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
            offsets_b=np.asarray([self._puller_visual_offset], dtype=float),
            mount_rot=[self._puller_mount_rot],
            rotor_angles=rotor_angles,
            visual_speeds=visual_speeds,
            target_speeds=np.asarray([target_speed], dtype=float),
            spin_directions=np.asarray([self._params.thrust_rotor_direction], dtype=float),
            spin_axes_local=np.asarray([self._puller_spin_axis_local], dtype=float),
            smoothing_tc=self._params.visual_speed_smoothing_tc,
        )
        self._puller_angle = float(rotor_angles[0])
        self._visual_puller_angular_velocity = float(visual_speeds[0])

    def _get_visual_rotor_angle(self) -> np.ndarray:
        if self._puller_body_id < 0:
            return np.zeros(0, dtype=float)
        return np.array([self._puller_angle], dtype=float)

    def _get_visual_rotor_speed(self) -> np.ndarray:
        if self._puller_body_id < 0:
            return np.zeros(0, dtype=float)
        return np.array([self._visual_puller_angular_velocity], dtype=float)

    def _read_diff_pressure_hpa(self) -> float | None:
        return self._last_diff_pressure_hpa
