"""MuJoCo multicopter environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatch

import mujoco
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.utils.dynamics import (
    DownwashParams,
    DownwashProjectionHull,
    LumpedDragParams,
    RotorFlowParams,
    first_order_response_step,
    idle_visual_speed_target,
)


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
        self._rotor_flow_params = RotorFlowParams.from_config(config.get("rotor_flow"))
        self._downwash_params = DownwashParams.from_config(config.get("downwash"))
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
        self._downwash_body_ids = self._resolve_downwash_body_ids()
        self._downwash_body_geom_point_offsets = self._build_downwash_body_geom_point_offsets()
        self._downwash_body_projection_hulls = self._build_downwash_body_projection_hulls()

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

    def _resolve_downwash_body_ids(self) -> list[int]:
        if not self._downwash_params.enabled:
            return []
        excluded = (
            "world",
            "base_link",
            "rotor_*",
            "*_vis",
            *self._downwash_params.exclude_body_patterns,
        )
        body_ids: list[int] = []
        for body_id in range(1, self._mj_model.nbody):
            body_name = mujoco.mj_id2name(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if not body_name or any(fnmatch(body_name, pattern) for pattern in excluded):
                continue
            if self._mj_model.body_mass[body_id] <= 0.0:
                continue
            body_ids.append(body_id)
        return body_ids

    def _compute_rotor_flow_thrust_scale(
        self,
        *,
        pos_w: np.ndarray,
        rotor_axis_w: np.ndarray,
        omega_abs: float,
        v_axial: float,
        v_perp_r: np.ndarray,
    ) -> float:
        params = self._rotor_flow_params
        if not params.enabled:
            return 1.0

        tip_speed = omega_abs * self._params.rotor_radius
        denom = tip_speed + 1e-9
        mu = float(np.linalg.norm(v_perp_r) / denom)
        inflow = float(v_axial / denom)
        scale = 1.0 + params.advance_c_lambda * inflow + params.advance_c_mu * mu**2
        scale = float(np.clip(scale, params.advance_scale_min, params.advance_scale_max))

        if params.ground_effect_enabled:
            downwash_axis_w = -np.asarray(rotor_axis_w, dtype=float)
            axis_norm = float(np.linalg.norm(downwash_axis_w))
            ground_distance = -1.0
            geom_id = np.array([-1], dtype=np.int32)
            if axis_norm > 1e-12:
                downwash_axis_w = downwash_axis_w / axis_norm
                ground_distance = float(
                    mujoco.mj_ray(
                        self._mj_model,
                        self._mj_data,
                        np.asarray(pos_w, dtype=float).copy(),
                        downwash_axis_w.copy(),
                        None,
                        1,
                        # Exclude the vehicle tree so only scene surfaces can produce ground effect.
                        self._base_link_id,
                        geom_id,
                        None,
                    )
                )
            trigger_height = params.ground_effect_height_rotor_diameters * 2.0 * self._params.rotor_radius
            if geom_id[0] >= 0 and 0.0 <= ground_distance < trigger_height:
                max_scale = params.ground_effect_max_scale
                if ground_distance <= self._params.rotor_radius / 4.0:
                    ground_scale = max_scale
                else:
                    ratio = self._params.rotor_radius / (4.0 * ground_distance)
                    ground_scale = max_scale if ratio >= 1.0 else float(np.clip(1.0 / (1.0 - ratio**2), 1.0, max_scale))
                scale *= ground_scale
        return scale

    def _compute_rotor_wrenches(
        self,
        base_pos: np.ndarray,
        rb: Rotation,
        rb_inv: Rotation,
        v_com_w: np.ndarray,
        omega_w: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        rotor_positions_w = np.zeros((self._rotor_count, 3))
        rotor_axes_w = np.zeros((self._rotor_count, 3))
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
            rotor_axis_w = rotor_axis_w / max(np.linalg.norm(rotor_axis_w), 1e-12)
            rotor_axes_w[i] = rotor_axis_w
            rotor_axis_r = rb_inv.apply(rotor_axis_w)
            rotor_axis_r = rotor_axis_r / max(np.linalg.norm(rotor_axis_r), 1e-12)
            v_axial = float(np.dot(v_point_r, rotor_axis_r))
            v_perp_r = v_point_r - v_axial * rotor_axis_r

            omega = self._rotor_angular_velocity[i]
            omega_abs = abs(omega)
            direction = self._rotor_direction[i]

            thrust = abs(self._params.motor_constant * omega * omega_abs)
            thrust *= self._compute_rotor_flow_thrust_scale(
                pos_w=pos_w,
                rotor_axis_w=rotor_axis_w,
                omega_abs=omega_abs,
                v_axial=v_axial,
                v_perp_r=v_perp_r,
            )
            rotor_thrusts[i] = thrust

            torque_axis_r = -direction * thrust * self._params.moment_constant * rotor_axis_r
            f_drag_r = -self._params.rotor_drag_coeff * omega_abs * v_perp_r
            m_rolling_r = -self._params.rolling_moment_coeff * omega_abs * direction * v_perp_r

            rotor_force_w[i] = rb.apply(rotor_axis_r * thrust + f_drag_r)
            rotor_moment_w[i] = rb.apply(torque_axis_r + m_rolling_r)

        return rotor_positions_w, rotor_axes_w, rotor_thrusts, rotor_force_w, rotor_moment_w

    def _apply_rotor_wrenches(
        self,
        rotor_positions_w: np.ndarray,
        rotor_force_w: np.ndarray,
        rotor_moment_w: np.ndarray,
    ) -> None:
        self._clear_applied_wrenches()
        self._apply_world_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)

    def _select_downwash_geom_ids(self, body_id: int) -> list[int]:
        body_geom_ids = [
            geom_id for geom_id in range(self._mj_model.ngeom) if int(self._mj_model.geom_bodyid[geom_id]) == body_id
        ]
        collision_geom_ids = [
            geom_id
            for geom_id in body_geom_ids
            if int(self._mj_model.geom_contype[geom_id]) != 0 or int(self._mj_model.geom_conaffinity[geom_id]) != 0
        ]
        return collision_geom_ids if collision_geom_ids else body_geom_ids

    def _build_downwash_body_geom_point_offsets(self) -> dict[int, list[tuple[int, np.ndarray]]]:
        if not self._downwash_body_ids:
            return {}
        return {
            body_id: [
                (geom_id, self._geom_sample_points_local(geom_id))
                for geom_id in self._select_downwash_geom_ids(body_id)
            ]
            for body_id in self._downwash_body_ids
        }

    def _geom_sample_points_local(self, geom_id: int) -> np.ndarray:
        geom_type = int(self._mj_model.geom_type[geom_id])

        if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            mesh_id = int(self._mj_model.geom_dataid[geom_id])
            if mesh_id < 0:
                return np.zeros((1, 3), dtype=float)
            vert_start = int(self._mj_model.mesh_vertadr[mesh_id])
            vert_count = int(self._mj_model.mesh_vertnum[mesh_id])
            vertices = np.asarray(self._mj_model.mesh_vert[vert_start : vert_start + vert_count], dtype=float)
            if vert_count > 256:
                step = int(np.ceil(vert_count / 256))
                vertices = vertices[::step]
            return vertices.copy()

        sx, sy, sz = self._mj_model.geom_size[geom_id].copy()
        if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            radius = sx
            local_points = np.array(
                [
                    [radius, 0.0, 0.0],
                    [-radius, 0.0, 0.0],
                    [0.0, radius, 0.0],
                    [0.0, -radius, 0.0],
                    [0.0, 0.0, radius],
                    [0.0, 0.0, -radius],
                ],
                dtype=float,
            )
        elif geom_type in (int(mujoco.mjtGeom.mjGEOM_CAPSULE), int(mujoco.mjtGeom.mjGEOM_CYLINDER)):
            radius = sx
            half_length = sy
            local_points = np.array(
                [
                    [radius, 0.0, half_length],
                    [-radius, 0.0, half_length],
                    [0.0, radius, half_length],
                    [0.0, -radius, half_length],
                    [radius, 0.0, -half_length],
                    [-radius, 0.0, -half_length],
                    [0.0, radius, -half_length],
                    [0.0, -radius, -half_length],
                ],
                dtype=float,
            )
        else:
            size = np.maximum(np.array([sx, sy, sz], dtype=float), 1e-9)
            local_points = np.array(
                [[x, y, z] for x in (-size[0], size[0]) for y in (-size[1], size[1]) for z in (-size[2], size[2])],
                dtype=float,
            )
        return local_points

    def _geom_sample_points_w(self, geom_id: int, local_points: np.ndarray | None = None) -> np.ndarray:
        points = self._geom_sample_points_local(geom_id) if local_points is None else local_points
        geom_pos = self._mj_data.geom_xpos[geom_id].copy()
        geom_rot = self._mj_data.geom_xmat[geom_id].reshape(3, 3).copy()
        return geom_pos + points @ geom_rot.T

    def _build_downwash_body_projection_hulls(self) -> dict[int, DownwashProjectionHull]:
        hulls: dict[int, DownwashProjectionHull] = {}
        for body_id in self._downwash_body_ids:
            geom_points = self._downwash_body_geom_point_offsets.get(body_id)
            if not geom_points:
                continue
            body_pos = self._mj_data.xpos[body_id].copy()
            body_rot = self._mj_data.xmat[body_id].reshape(3, 3).copy()
            points_w = np.vstack(
                [self._geom_sample_points_w(geom_id, local_points) for geom_id, local_points in geom_points]
            )
            points_b = (points_w - body_pos) @ body_rot
            if points_b.shape[0] < 4:
                continue
            try:
                hull = ConvexHull(points_b)
            except Exception:
                continue
            triangles = points_b[hull.simplices]
            area_vectors = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            face_areas = 0.5 * np.linalg.norm(area_vectors, axis=1)
            valid = face_areas > 1e-12
            if not np.any(valid):
                continue
            face_normals = area_vectors[valid] / (2.0 * face_areas[valid, None])
            hulls[body_id] = DownwashProjectionHull(
                face_normals_b=face_normals,
                face_areas=face_areas[valid],
            )
        return hulls

    def _estimate_body_projected_area(self, body_id: int, direction_w: np.ndarray) -> float:
        direction = np.asarray(direction_w, dtype=float)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            return 0.0
        normal = direction / norm
        projection_hull = self._downwash_body_projection_hulls.get(body_id)
        if projection_hull is not None:
            body_rot = self._mj_data.xmat[body_id].reshape(3, 3).copy()
            normal_b = normal @ body_rot
            return float(0.5 * np.sum(projection_hull.face_areas * np.abs(projection_hull.face_normals_b @ normal_b)))

        geom_points = self._downwash_body_geom_point_offsets.get(body_id)
        if geom_points is None:
            geom_points = [
                (geom_id, self._geom_sample_points_local(geom_id))
                for geom_id in self._select_downwash_geom_ids(body_id)
            ]
        if not geom_points:
            return 0.0
        points = np.vstack([self._geom_sample_points_w(geom_id, local_points) for geom_id, local_points in geom_points])
        if points.shape[0] < 3:
            return 0.0
        reference = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(reference, normal))) > 0.9:
            reference = np.array([0.0, 1.0, 0.0], dtype=float)
        basis_u = reference - float(np.dot(reference, normal)) * normal
        basis_u /= max(np.linalg.norm(basis_u), 1e-12)
        basis_v = np.cross(normal, basis_u)
        projected = np.column_stack((points @ basis_u, points @ basis_v))
        return self._convex_hull_area_2d(projected)

    @staticmethod
    def _convex_hull_area_2d(points: np.ndarray) -> float:
        projected = np.asarray(points, dtype=float)
        if projected.shape[0] < 3:
            return 0.0
        try:
            return float(ConvexHull(projected).volume)
        except Exception:
            unique_projected = np.unique(projected, axis=0)
            if unique_projected.shape[0] < 3:
                return 0.0
            try:
                return float(ConvexHull(unique_projected).volume)
            except Exception:
                return 0.0

    def _compute_downwash_force_for_body(
        self,
        body_id: int,
        rotor_positions_w: np.ndarray,
        rotor_thrusts: np.ndarray,
        rotor_axes_w: np.ndarray | None = None,
    ) -> np.ndarray:
        params = self._downwash_params
        if not params.enabled:
            return np.zeros(3, dtype=float)
        body_pos_w = self._mj_data.xipos[body_id].copy()
        wind_w = self._get_wind_velocity_w()
        disk_area = np.pi * self._params.rotor_radius**2
        aggregate_wake_w = np.zeros(3, dtype=float)
        for rotor_idx, rotor_pos_w in enumerate(rotor_positions_w):
            thrust = float(rotor_thrusts[rotor_idx])
            if thrust <= 0.0:
                continue
            if rotor_axes_w is None:
                rotor_axis_w = Rotation.from_quat(
                    self._mj_data.xquat[self._rotor_body_ids[rotor_idx]].copy(), scalar_first=True
                ).apply(np.array([0.0, 0.0, 1.0], dtype=float))
                rotor_axis_w = rotor_axis_w / max(np.linalg.norm(rotor_axis_w), 1e-12)
            else:
                rotor_axis_w = rotor_axes_w[rotor_idx]
            downwash_axis_w = -rotor_axis_w
            delta_w = body_pos_w - rotor_pos_w
            axial_distance = float(np.dot(delta_w, downwash_axis_w))
            if axial_distance <= 0.0:
                continue
            radial_w = delta_w - axial_distance * downwash_axis_w
            wake_radius = self._params.rotor_radius + axial_distance * np.tan(params.wake_spread_angle_rad)
            if wake_radius <= 0.0:
                continue
            radial_distance = float(np.linalg.norm(radial_w))
            if radial_distance > wake_radius:
                continue
            profile = max(0.0, 1.0 - (radial_distance / wake_radius) ** 2)
            axial_decay = np.exp(-axial_distance / max(params.axial_decay_m, 1e-9))
            wake_speed = params.wake_speed_scale * np.sqrt(2.0 * thrust / (params.air_density * disk_area))
            wake_w = wake_speed * profile * axial_decay * downwash_axis_w
            aggregate_wake_w += wake_w

        if not np.any(aggregate_wake_w):
            return np.zeros(3, dtype=float)
        jacp = np.zeros((3, self._mj_model.nv), dtype=float)
        jacr = np.zeros((3, self._mj_model.nv), dtype=float)
        mujoco.mj_jacBodyCom(self._mj_model, self._mj_data, jacp, jacr, body_id)
        v_rel_w = wind_w + aggregate_wake_w - jacp @ self._mj_data.qvel
        v_rel_norm = float(np.linalg.norm(v_rel_w))
        if v_rel_norm <= 1e-12:
            return np.zeros(3, dtype=float)
        projected_area = self._estimate_body_projected_area(body_id, v_rel_w)
        if projected_area <= 0.0:
            return np.zeros(3, dtype=float)
        return (
            0.5
            * params.air_density
            * params.drag_coefficient
            * params.area_scale
            * projected_area
            * v_rel_norm
            * v_rel_w
        )

    def _apply_downwash_forces(
        self,
        rotor_positions_w: np.ndarray,
        rotor_thrusts: np.ndarray,
        rotor_axes_w: np.ndarray | None = None,
    ) -> None:
        params = self._downwash_params
        if not params.enabled or not self._downwash_body_ids:
            return

        for body_id in self._downwash_body_ids:
            body_pos_w = self._mj_data.xipos[body_id].copy()
            body_force_w = self._compute_downwash_force_for_body(
                body_id,
                rotor_positions_w,
                rotor_thrusts,
                rotor_axes_w=rotor_axes_w,
            )
            if np.any(body_force_w):
                mujoco.mj_applyFT(
                    self._mj_model,
                    self._mj_data,
                    body_force_w,
                    np.zeros(3, dtype=float),
                    body_pos_w,
                    body_id,
                    self._mj_data.qfrc_applied,
                )

    def _apply_vehicle_physics(self) -> None:
        dt_s = self._mj_model.opt.timestep
        self._update_rotor_speed_state(dt_s)
        base_pos, _, rb, rb_inv, v_com_w, _, omega_w = self._get_base_kinematics()
        rotor_positions_w, rotor_axes_w, rotor_thrusts, rotor_force_w, rotor_moment_w = self._compute_rotor_wrenches(
            base_pos, rb, rb_inv, v_com_w, omega_w
        )
        self._apply_rotor_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)
        self._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)
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
