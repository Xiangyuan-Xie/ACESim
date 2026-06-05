"""MuJoCo multicopter environment with PX4 HIL integration."""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatch

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.px4_mj_env import PX4MJEnv
from acesim.utils.dynamics import (
    AeroSurfaceSamples,
    DownwashParams,
    LumpedDragParams,
    RotorFlowParams,
    RotorInertialTorqueParams,
    first_order_response_step,
)

_GROUND_EFFECT_SUPPORT_GEOM_GROUP = 2
_AERO_SURFACE_COALESCE_GRID_ROTOR_RADIUS_SCALE = 0.5


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
        self._rotor_inertial_torque_params = RotorInertialTorqueParams.from_config(config.get("rotor_inertial_torque"))
        self._rotor_inertial_torque_active = self._rotor_inertial_torque_params.enabled
        if self._rotor_inertial_torque_params.enabled and self._rotor_inertial_torque_params.randomize_enabled:
            random_seed = int(config.get("rotor_inertial_torque", {}).get("random_seed", 0))
            rng = np.random.default_rng(random_seed)
            self._rotor_inertial_torque_active = (
                float(rng.random()) < self._rotor_inertial_torque_params.enabled_probability
            )
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
        self._rotor_angular_acceleration = np.zeros(self._rotor_count)
        self._visual_rotor_angular_velocity = np.zeros(self._rotor_count)
        self._applied_actuator_controls = np.zeros(self._rotor_count)
        self._rotor_angle = np.zeros(self._rotor_count)
        direction = np.asarray(self._params.rotor_direction, dtype=float)
        if direction.size != self._rotor_count:
            base = np.array([1.0, -1.0])
            direction = np.tile(base, int(np.ceil(self._rotor_count / base.size)))[: self._rotor_count]
        self._rotor_direction = direction
        self._rotor_positions_w = np.zeros((self._rotor_count, 3), dtype=float)
        self._rotor_axes_w = np.zeros((self._rotor_count, 3), dtype=float)
        self._rotor_thrusts = np.zeros(self._rotor_count, dtype=float)
        self._rotor_force_w = np.zeros((self._rotor_count, 3), dtype=float)
        self._rotor_moment_w = np.zeros((self._rotor_count, 3), dtype=float)
        self._rotor_axis_r_buffer = np.zeros(3, dtype=float)
        self._v_air_point_w_buffer = np.zeros(3, dtype=float)
        self._target_visual_speeds = np.zeros(self._rotor_count, dtype=float)
        self._rotor_spin_axes_local = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (self._rotor_count, 1))
        self._rotor_mount_quats = np.asarray(
            [rot.as_quat(scalar_first=True) for rot in self._rotor_mount_rot],
            dtype=float,
        )
        self._rotor_ground_ray_geom_id = np.array([-1], dtype=np.int32)
        self._rotor_ground_ray_normal = np.zeros(3, dtype=float)
        self._rotor_ground_ray_origin = np.zeros(3, dtype=float)
        self._rotor_ground_ray_direction = np.zeros(3, dtype=float)
        self._rotor_ground_geomgroup = np.zeros(6, dtype=np.uint8)
        self._rotor_ground_geomgroup[_GROUND_EFFECT_SUPPORT_GEOM_GROUP] = 1
        self._downwash_body_ids = self._resolve_aero_body_ids(
            enabled=self._downwash_params.enabled,
            exclude_patterns=self._downwash_params.exclude_body_patterns,
        )
        self._aero_surface_samples_by_body = self._build_aero_surface_samples_by_body(self._downwash_body_ids)
        self._aero_surface_sample_radius_by_body = {
            body_id: float(np.max(np.linalg.norm(samples.points_b, axis=1)))
            for body_id, samples in self._aero_surface_samples_by_body.items()
            if samples.points_b.size
        }
        self._downwash_jacp = np.zeros((3, self._mj_model.nv), dtype=float)
        self._downwash_jacr = np.zeros((3, self._mj_model.nv), dtype=float)

    def _actuator_channel_count(self) -> int:
        return self._rotor_count

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        self._applied_actuator_controls = np.clip(np.asarray(controls, dtype=float), 0.0, 1.0)
        self._desired_rotor_angular_velocity = self._applied_actuator_controls * self._params.max_rot_velocity

    def _update_rotor_speed_state(self, dt_s: float) -> None:
        previous = self._rotor_angular_velocity.copy()
        self._rotor_angular_velocity = first_order_response_step(
            self._rotor_angular_velocity,
            self._desired_rotor_angular_velocity,
            dt_s,
            self._params.time_constant_up,
            self._params.time_constant_down,
        )
        if dt_s > 0.0:
            self._rotor_angular_acceleration = (self._rotor_angular_velocity - previous) / dt_s
        else:
            self._rotor_angular_acceleration.fill(0.0)

    def _resolve_aero_body_ids(self, *, enabled: bool, exclude_patterns: tuple[str, ...]) -> list[int]:
        if not enabled:
            return []
        excluded = (
            "world",
            "base_link",
            "rotor_*",
            "*_vis",
            *exclude_patterns,
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
            trigger_height = params.ground_effect_height_rotor_diameters * 2.0 * self._params.rotor_radius
            self._rotor_ground_ray_direction[:] = -rotor_axis_w
            axis_norm = float(np.linalg.norm(self._rotor_ground_ray_direction))
            ground_distance = -1.0
            geom_id = -1
            self._rotor_ground_ray_geom_id[0] = -1
            self._rotor_ground_ray_normal[:] = 0.0
            if axis_norm > 1e-12:
                self._rotor_ground_ray_direction[:] = self._rotor_ground_ray_direction / axis_norm
                self._rotor_ground_ray_origin[:] = pos_w
                ground_distance = float(
                    mujoco.mj_ray(
                        self._mj_model,
                        self._mj_data,
                        self._rotor_ground_ray_origin,
                        self._rotor_ground_ray_direction,
                        self._rotor_ground_geomgroup,
                        1,
                        # Exclude the vehicle tree; the group mask selects support surfaces.
                        self._base_link_id,
                        self._rotor_ground_ray_geom_id,
                        self._rotor_ground_ray_normal,
                    )
                )
                geom_id = int(self._rotor_ground_ray_geom_id[0])
            if geom_id >= 0 and 0.0 <= ground_distance < trigger_height:
                hit_normal_w = self._rotor_ground_ray_normal
                normal_norm = float(np.linalg.norm(hit_normal_w))
                if normal_norm <= 1e-12:
                    return scale
                hit_normal_w = hit_normal_w / normal_norm
                if (
                    float(np.dot(hit_normal_w, -self._rotor_ground_ray_direction))
                    <= params.ground_effect_normal_min_dot
                ):
                    return scale
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
        rb: Rotation | np.ndarray,
        rb_inv: Rotation | np.ndarray,
        v_com_w: np.ndarray,
        omega_w: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        rotor_positions_w = self._rotor_positions_w
        rotor_axes_w = self._rotor_axes_w
        rotor_thrusts = self._rotor_thrusts
        rotor_force_w = self._rotor_force_w
        rotor_moment_w = self._rotor_moment_w
        rotor_positions_w.fill(0.0)
        rotor_axes_w.fill(0.0)
        rotor_thrusts.fill(0.0)
        rotor_force_w.fill(0.0)
        rotor_moment_w.fill(0.0)
        wind_w = self._get_wind_velocity_w()
        rb_mat = rb if isinstance(rb, np.ndarray) else rb.as_matrix()
        rb_inv_mat = rb_inv if isinstance(rb_inv, np.ndarray) else rb_inv.as_matrix()
        for i in range(self._rotor_count):
            r_off_w = rb_mat @ self._rotor_offsets[i]
            pos_w = base_pos + r_off_w
            rotor_positions_w[i] = pos_w
            v_air_point_w = self._v_air_point_w_buffer
            v_air_point_w[0] = v_com_w[0] + omega_w[1] * r_off_w[2] - omega_w[2] * r_off_w[1] - wind_w[0]
            v_air_point_w[1] = v_com_w[1] + omega_w[2] * r_off_w[0] - omega_w[0] * r_off_w[2] - wind_w[1]
            v_air_point_w[2] = v_com_w[2] + omega_w[0] * r_off_w[1] - omega_w[1] * r_off_w[0] - wind_w[2]
            v_point_r = rb_inv_mat @ v_air_point_w
            rotor_xmat = self._mj_data.xmat[self._rotor_body_ids[i]].reshape(3, 3)
            rotor_axis_w = rotor_axes_w[i]
            rotor_axis_w[:] = rotor_xmat[:, 2]
            rotor_axis_w = rotor_axis_w / max(np.linalg.norm(rotor_axis_w), 1e-12)
            rotor_axes_w[i] = rotor_axis_w
            rotor_axis_r = self._rotor_axis_r_buffer
            rotor_axis_r[:] = rb_inv_mat @ rotor_axis_w
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

            rotor_force_w[i] = rb_mat @ (rotor_axis_r * thrust + f_drag_r)
            rotor_moment_w[i] = rb_mat @ (torque_axis_r + m_rolling_r)

        return rotor_positions_w, rotor_axes_w, rotor_thrusts, rotor_force_w, rotor_moment_w

    def _apply_rotor_wrenches(
        self,
        rotor_positions_w: np.ndarray,
        rotor_force_w: np.ndarray,
        rotor_moment_w: np.ndarray,
    ) -> None:
        self._clear_applied_wrenches()
        self._apply_world_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)

    def _select_aero_geom_ids(self, body_id: int) -> list[int]:
        body_geom_ids = [
            geom_id for geom_id in range(self._mj_model.ngeom) if int(self._mj_model.geom_bodyid[geom_id]) == body_id
        ]
        collision_geom_ids = [
            geom_id
            for geom_id in body_geom_ids
            if int(self._mj_model.geom_contype[geom_id]) != 0 or int(self._mj_model.geom_conaffinity[geom_id]) != 0
        ]
        return collision_geom_ids if collision_geom_ids else body_geom_ids

    def _build_aero_surface_samples_by_body(self, body_ids: list[int]) -> dict[int, AeroSurfaceSamples]:
        samples_by_body: dict[int, AeroSurfaceSamples] = {}
        for body_id in body_ids:
            point_sets: list[np.ndarray] = []
            normal_sets: list[np.ndarray] = []
            area_sets: list[np.ndarray] = []
            body_pos = self._mj_data.xpos[body_id].copy()
            body_rot = self._mj_data.xmat[body_id].reshape(3, 3).copy()
            for geom_id in self._select_aero_geom_ids(body_id):
                points_g, normals_g, areas = self._geom_surface_samples_local(geom_id)
                if not len(areas):
                    continue
                geom_pos = self._mj_data.geom_xpos[geom_id].copy()
                geom_rot = self._mj_data.geom_xmat[geom_id].reshape(3, 3).copy()
                points_w = geom_pos + points_g @ geom_rot.T
                normals_w = normals_g @ geom_rot.T
                point_sets.append((points_w - body_pos) @ body_rot)
                normal_sets.append(normals_w @ body_rot)
                area_sets.append(areas)
            if point_sets:
                points_b = np.vstack(point_sets)
                normals_b = np.vstack(normal_sets)
                areas = np.concatenate(area_sets)
                sample_grid_m = max(self._params.rotor_radius * _AERO_SURFACE_COALESCE_GRID_ROTOR_RADIUS_SCALE, 1e-3)
                point_keys = np.rint(points_b / sample_grid_m).astype(np.int64)
                normal_keys = np.rint(normals_b / 0.25).astype(np.int64)
                _, inverse = np.unique(np.hstack((point_keys, normal_keys)), axis=0, return_inverse=True)
                if inverse.size and int(np.max(inverse)) + 1 < areas.size:
                    area_sum = np.bincount(inverse, weights=areas)
                    valid_area = area_sum > 1e-12
                    coalesced_points = np.column_stack(
                        (
                            np.bincount(inverse, weights=areas * points_b[:, 0]),
                            np.bincount(inverse, weights=areas * points_b[:, 1]),
                            np.bincount(inverse, weights=areas * points_b[:, 2]),
                        )
                    )
                    coalesced_normals = np.column_stack(
                        (
                            np.bincount(inverse, weights=areas * normals_b[:, 0]),
                            np.bincount(inverse, weights=areas * normals_b[:, 1]),
                            np.bincount(inverse, weights=areas * normals_b[:, 2]),
                        )
                    )
                    coalesced_points = coalesced_points[valid_area] / area_sum[valid_area, None]
                    coalesced_normals = coalesced_normals[valid_area]
                    normal_norms = np.maximum(np.linalg.norm(coalesced_normals, axis=1), 1e-12)
                    points_b = coalesced_points
                    normals_b = coalesced_normals / normal_norms[:, None]
                    areas = area_sum[valid_area]
                samples_by_body[body_id] = AeroSurfaceSamples(
                    points_b=points_b,
                    normals_b=normals_b,
                    areas=areas,
                )
        return samples_by_body

    def _geom_surface_samples_local(self, geom_id: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        geom_type = int(self._mj_model.geom_type[geom_id])

        if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            mesh_id = int(self._mj_model.geom_dataid[geom_id])
            if mesh_id < 0:
                return (
                    np.zeros((0, 3), dtype=float),
                    np.zeros((0, 3), dtype=float),
                    np.zeros(0, dtype=float),
                )
            face_start = int(self._mj_model.mesh_faceadr[mesh_id])
            face_count = int(self._mj_model.mesh_facenum[mesh_id])
            if face_count <= 0:
                return (
                    np.zeros((0, 3), dtype=float),
                    np.zeros((0, 3), dtype=float),
                    np.zeros(0, dtype=float),
                )
            vert_start = int(self._mj_model.mesh_vertadr[mesh_id])
            vertices = np.asarray(self._mj_model.mesh_vert, dtype=float)
            local_faces = np.asarray(self._mj_model.mesh_face[face_start : face_start + face_count], dtype=int)
            triangles = vertices[vert_start + local_faces]
            area_vectors = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            face_areas = 0.5 * np.linalg.norm(area_vectors, axis=1)
            valid = face_areas > 1e-12
            if not np.any(valid):
                return (
                    np.zeros((0, 3), dtype=float),
                    np.zeros((0, 3), dtype=float),
                    np.zeros(0, dtype=float),
                )
            return (
                np.mean(triangles[valid], axis=1),
                area_vectors[valid] / (2.0 * face_areas[valid, None]),
                face_areas[valid],
            )

        sx, sy, sz = self._mj_model.geom_size[geom_id].copy()
        if geom_type == int(mujoco.mjtGeom.mjGEOM_SPHERE):
            radius = sx
            points = np.array(
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
            normals = points / max(radius, 1e-12)
            areas = np.full(6, 4.0 * np.pi * radius**2 / 6.0, dtype=float)
            return points, normals, areas
        elif geom_type in (int(mujoco.mjtGeom.mjGEOM_CAPSULE), int(mujoco.mjtGeom.mjGEOM_CYLINDER)):
            radius = sx
            half_length = sy
            points = np.array(
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
            normals = points.copy()
            normals[:, 2] = 0.0
            normal_norms = np.maximum(np.linalg.norm(normals, axis=1), 1e-12)
            normals = normals / normal_norms[:, None]
            side_area = 2.0 * np.pi * radius * 2.0 * half_length
            areas = np.full(points.shape[0], side_area / points.shape[0], dtype=float)
            return points, normals, areas
        else:
            size = np.maximum(np.array([sx, sy, sz], dtype=float), 1e-9)
            points = np.array(
                [
                    [size[0], 0.0, 0.0],
                    [-size[0], 0.0, 0.0],
                    [0.0, size[1], 0.0],
                    [0.0, -size[1], 0.0],
                    [0.0, 0.0, size[2]],
                    [0.0, 0.0, -size[2]],
                ],
                dtype=float,
            )
            normals = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, -1.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, 0.0, -1.0],
                ],
                dtype=float,
            )
            areas = np.array(
                [
                    4.0 * size[1] * size[2],
                    4.0 * size[1] * size[2],
                    4.0 * size[0] * size[2],
                    4.0 * size[0] * size[2],
                    4.0 * size[0] * size[1],
                    4.0 * size[0] * size[1],
                ],
                dtype=float,
            )
            return points, normals, areas

    def _compute_downwash_wrenches_for_bodies(
        self,
        body_ids: list[int],
        rotor_positions_w: np.ndarray,
        rotor_thrusts: np.ndarray,
        rotor_axes_w: np.ndarray | None = None,
    ) -> dict[int, tuple[np.ndarray, np.ndarray]]:
        params = self._downwash_params
        air_density = self._get_medium_density_kg_m3()
        empty_result = {body_id: (np.zeros(3, dtype=float), np.zeros(3, dtype=float)) for body_id in body_ids}
        if not params.enabled or air_density <= 0.0 or not body_ids:
            return empty_result

        rotor_positions = np.asarray(rotor_positions_w, dtype=float)
        thrusts = np.asarray(rotor_thrusts, dtype=float)
        active_rotors = thrusts > 0.0
        if rotor_positions.size == 0 or not np.any(active_rotors):
            return empty_result

        active_positions = rotor_positions[active_rotors]
        active_thrusts = thrusts[active_rotors]
        if rotor_axes_w is None:
            rotor_axes = np.asarray(
                [
                    self._mj_data.xmat[self._rotor_body_ids[rotor_idx]].reshape(3, 3)[:, 2]
                    for rotor_idx in np.flatnonzero(active_rotors)
                ],
                dtype=float,
            )
        else:
            rotor_axes = np.asarray(rotor_axes_w, dtype=float)[active_rotors]
        axis_norms = np.maximum(np.linalg.norm(rotor_axes, axis=1), 1e-12)
        downwash_axes = -(rotor_axes / axis_norms[:, None])

        sample_points: list[np.ndarray] = []
        sample_normals: list[np.ndarray] = []
        sample_areas: list[np.ndarray] = []
        sample_body_indices: list[np.ndarray] = []
        body_positions: list[np.ndarray] = []
        active_body_ids: list[int] = []
        tan_spread = np.tan(params.wake_spread_angle_rad)
        for body_id in body_ids:
            samples = self._aero_surface_samples_by_body.get(body_id)
            if samples is None:
                continue
            body_pos_w = self._mj_data.xpos[body_id].copy()
            body_com_w = self._mj_data.xipos[body_id].copy()
            sample_radius = self._aero_surface_sample_radius_by_body.get(body_id, 0.0)
            body_delta_w = body_pos_w[None, :] - active_positions
            body_axial_distance = (
                body_delta_w[:, 0] * downwash_axes[:, 0]
                + body_delta_w[:, 1] * downwash_axes[:, 1]
                + body_delta_w[:, 2] * downwash_axes[:, 2]
            )
            body_radial_x = body_delta_w[:, 0] - body_axial_distance * downwash_axes[:, 0]
            body_radial_y = body_delta_w[:, 1] - body_axial_distance * downwash_axes[:, 1]
            body_radial_z = body_delta_w[:, 2] - body_axial_distance * downwash_axes[:, 2]
            body_radial_distance_sq = (
                body_radial_x * body_radial_x + body_radial_y * body_radial_y + body_radial_z * body_radial_z
            )
            max_axial_distance = body_axial_distance + sample_radius
            body_wake_radius = self._params.rotor_radius + np.maximum(0.0, max_axial_distance) * tan_spread
            possible_rotors = (max_axial_distance > 0.0) & (
                body_radial_distance_sq <= (body_wake_radius + sample_radius) ** 2
            )
            if not np.any(possible_rotors):
                continue

            body_index = len(active_body_ids)
            body_rot = self._mj_data.xmat[body_id].reshape(3, 3)
            sample_points.append(body_pos_w + samples.points_b @ body_rot.T)
            sample_normals.append(samples.normals_b @ body_rot.T)
            sample_areas.append(samples.areas)
            sample_body_indices.append(np.full(samples.areas.shape, body_index, dtype=int))
            body_positions.append(body_com_w)
            active_body_ids.append(body_id)

        if not active_body_ids:
            return empty_result

        sample_points_w = np.vstack(sample_points)
        sample_normals_w = np.vstack(sample_normals)
        areas = np.concatenate(sample_areas)
        body_index_by_sample = np.concatenate(sample_body_indices)
        body_positions_w = np.vstack(body_positions)

        delta_w = sample_points_w[:, None, :] - active_positions[None, :, :]
        axial_distance = (
            delta_w[:, :, 0] * downwash_axes[None, :, 0]
            + delta_w[:, :, 1] * downwash_axes[None, :, 1]
            + delta_w[:, :, 2] * downwash_axes[None, :, 2]
        )
        radial_x = delta_w[:, :, 0] - axial_distance * downwash_axes[None, :, 0]
        radial_y = delta_w[:, :, 1] - axial_distance * downwash_axes[None, :, 1]
        radial_z = delta_w[:, :, 2] - axial_distance * downwash_axes[None, :, 2]
        radial_distance_sq = radial_x * radial_x + radial_y * radial_y + radial_z * radial_z
        wake_radius = self._params.rotor_radius + axial_distance * tan_spread
        wake_radius_sq = wake_radius * wake_radius
        active_wake = (axial_distance > 0.0) & (wake_radius > 0.0) & (radial_distance_sq <= wake_radius_sq)
        if not np.any(active_wake):
            return empty_result

        profile = np.zeros_like(axial_distance, dtype=float)
        profile[active_wake] = 1.0 - radial_distance_sq[active_wake] / wake_radius_sq[active_wake]
        axial_decay = np.zeros_like(axial_distance, dtype=float)
        axial_decay[active_wake] = np.exp(-axial_distance[active_wake] / max(params.axial_decay_m, 1e-9))
        disk_area = np.pi * self._params.rotor_radius**2
        wake_speed = params.wake_speed_scale * np.sqrt(2.0 * active_thrusts / (air_density * disk_area))
        wake_weights = profile * axial_decay * wake_speed[None, :]
        wake_by_sample_w = wake_weights @ downwash_axes
        wake_active = (
            wake_by_sample_w[:, 0] * wake_by_sample_w[:, 0]
            + wake_by_sample_w[:, 1] * wake_by_sample_w[:, 1]
            + wake_by_sample_w[:, 2] * wake_by_sample_w[:, 2]
            > 1e-24
        )
        if not np.any(wake_active):
            return empty_result

        wind_w = self._get_wind_velocity_w()
        body_velocity_w = np.zeros((len(active_body_ids), 3), dtype=float)
        body_omega_w = np.zeros((len(active_body_ids), 3), dtype=float)
        jacp = self._downwash_jacp
        jacr = self._downwash_jacr
        for body_index, body_id in enumerate(active_body_ids):
            jacp.fill(0.0)
            jacr.fill(0.0)
            mujoco.mj_jacBodyCom(self._mj_model, self._mj_data, jacp, jacr, body_id)
            body_velocity_w[body_index] = jacp @ self._mj_data.qvel
            body_omega_w[body_index] = jacr @ self._mj_data.qvel

        active_body_index = body_index_by_sample[wake_active]
        r_w = sample_points_w[wake_active] - body_positions_w[active_body_index]
        v_com_w = body_velocity_w[active_body_index]
        omega_body_w = body_omega_w[active_body_index]
        point_velocity_w = np.empty_like(r_w)
        point_velocity_w[:, 0] = v_com_w[:, 0] + omega_body_w[:, 1] * r_w[:, 2] - omega_body_w[:, 2] * r_w[:, 1]
        point_velocity_w[:, 1] = v_com_w[:, 1] + omega_body_w[:, 2] * r_w[:, 0] - omega_body_w[:, 0] * r_w[:, 2]
        point_velocity_w[:, 2] = v_com_w[:, 2] + omega_body_w[:, 0] * r_w[:, 1] - omega_body_w[:, 1] * r_w[:, 0]
        v_rel_w = wind_w + wake_by_sample_w[wake_active] - point_velocity_w
        v_rel_norm_sq = v_rel_w[:, 0] * v_rel_w[:, 0] + v_rel_w[:, 1] * v_rel_w[:, 1] + v_rel_w[:, 2] * v_rel_w[:, 2]
        v_rel_norm = np.sqrt(v_rel_norm_sq)
        valid_rel = v_rel_norm > 1e-12
        if not np.any(valid_rel):
            return empty_result

        active_normals_w = sample_normals_w[wake_active][valid_rel]
        active_areas = areas[wake_active][valid_rel]
        active_r_w = r_w[valid_rel]
        force_body_index = active_body_index[valid_rel]
        active_v_rel_w = v_rel_w[valid_rel]
        active_v_rel_norm = v_rel_norm[valid_rel]
        projected_area = (
            0.5
            * active_areas
            * np.abs(
                (
                    active_normals_w[:, 0] * active_v_rel_w[:, 0]
                    + active_normals_w[:, 1] * active_v_rel_w[:, 1]
                    + active_normals_w[:, 2] * active_v_rel_w[:, 2]
                )
                / active_v_rel_norm
            )
        )
        valid_area = projected_area > 0.0
        if not np.any(valid_area):
            return empty_result

        d_force_w = (
            0.5
            * air_density
            * params.drag_coefficient
            * params.area_scale
            * projected_area[valid_area, None]
            * active_v_rel_norm[valid_area, None]
            * active_v_rel_w[valid_area]
        )
        force_points_w = active_r_w[valid_area]
        d_torque_w = np.empty_like(d_force_w)
        d_torque_w[:, 0] = force_points_w[:, 1] * d_force_w[:, 2] - force_points_w[:, 2] * d_force_w[:, 1]
        d_torque_w[:, 1] = force_points_w[:, 2] * d_force_w[:, 0] - force_points_w[:, 0] * d_force_w[:, 2]
        d_torque_w[:, 2] = force_points_w[:, 0] * d_force_w[:, 1] - force_points_w[:, 1] * d_force_w[:, 0]
        valid_force_body_index = force_body_index[valid_area]
        force_by_body = np.zeros((len(active_body_ids), 3), dtype=float)
        torque_by_body = np.zeros((len(active_body_ids), 3), dtype=float)
        np.add.at(force_by_body, valid_force_body_index, d_force_w)
        np.add.at(torque_by_body, valid_force_body_index, d_torque_w)

        result = dict(empty_result)
        for body_index, body_id in enumerate(active_body_ids):
            result[body_id] = (force_by_body[body_index], torque_by_body[body_index])
        return result

    def _compute_rotor_inertial_torque_w(
        self,
        *,
        rotor_idx: int,
        rotor_axis_w: np.ndarray,
        omega_w: np.ndarray,
    ) -> np.ndarray:
        params = self._rotor_inertial_torque_params
        if not self._rotor_inertial_torque_active or params.inertia_kg_m2 <= 0.0:
            return np.zeros(3, dtype=float)
        axis_w = np.asarray(rotor_axis_w, dtype=float)
        axis_w = axis_w / max(np.linalg.norm(axis_w), 1e-12)
        spin_direction = float(self._rotor_direction[rotor_idx])
        torque_w = np.zeros(3, dtype=float)
        if params.apply_acceleration_torque:
            torque_w += -spin_direction * params.inertia_kg_m2 * self._rotor_angular_acceleration[rotor_idx] * axis_w
        if params.apply_gyro_torque:
            angular_momentum_w = (
                spin_direction * params.inertia_kg_m2 * self._rotor_angular_velocity[rotor_idx] * axis_w
            )
            torque_w += -np.cross(np.asarray(omega_w, dtype=float), angular_momentum_w)
        return torque_w

    def _apply_downwash_forces(
        self,
        rotor_positions_w: np.ndarray,
        rotor_thrusts: np.ndarray,
        rotor_axes_w: np.ndarray | None = None,
    ) -> None:
        params = self._downwash_params
        if not params.enabled or not self._downwash_body_ids:
            return

        downwash_wrenches = self._compute_downwash_wrenches_for_bodies(
            self._downwash_body_ids,
            rotor_positions_w,
            rotor_thrusts,
            rotor_axes_w=rotor_axes_w,
        )
        for body_id, (body_force_w, body_torque_w) in downwash_wrenches.items():
            body_pos_w = self._mj_data.xipos[body_id].copy()
            if np.any(body_force_w) or np.any(body_torque_w):
                mujoco.mj_applyFT(
                    self._mj_model,
                    self._mj_data,
                    body_force_w,
                    body_torque_w,
                    body_pos_w,
                    body_id,
                    self._mj_data.qfrc_applied,
                )

    def _apply_vehicle_physics(self) -> None:
        dt_s = self._mj_model.opt.timestep
        self._update_rotor_speed_state(dt_s)
        base_pos = self._mj_data.xpos[self._base_link_id].copy()
        rb_mat = self._mj_data.xmat[self._base_link_id].reshape(3, 3).copy()
        rb_inv_mat = rb_mat.T
        v_com_w = self._get_sensor_raw("linvel")
        omega_r = self._get_sensor_raw("gyro")
        omega_w = rb_mat @ omega_r
        rotor_positions_w, rotor_axes_w, rotor_thrusts, rotor_force_w, rotor_moment_w = self._compute_rotor_wrenches(
            base_pos, rb_mat, rb_inv_mat, v_com_w, omega_w
        )
        for rotor_idx, rotor_axis_w in enumerate(rotor_axes_w):
            rotor_moment_w[rotor_idx] += self._compute_rotor_inertial_torque_w(
                rotor_idx=rotor_idx,
                rotor_axis_w=rotor_axis_w,
                omega_w=omega_w,
            )
        self._apply_rotor_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)
        self._apply_downwash_forces(rotor_positions_w, rotor_thrusts, rotor_axes_w=rotor_axes_w)
        self._apply_lumped_drag_wrench(base_pos, rb_mat, rb_inv_mat, v_com_w)

    def _update_vehicle_visuals(self) -> None:
        armed = self._px4_transport.update_arming_state()
        if armed:
            physical_speeds = np.maximum(0.0, self._rotor_angular_velocity)
            target_visual_speeds = self._target_visual_speeds
            target_visual_speeds[:] = physical_speeds
            zero_output = self._applied_actuator_controls <= 0.0
            target_visual_speeds[zero_output] = np.maximum(
                target_visual_speeds[zero_output],
                self._params.idle_visual_speed,
            )
            active = ~zero_output
            if self._params.low_speed_blend_end > 0.0 and np.any(active):
                blend_weight = np.clip(
                    1.0 - physical_speeds[active] / self._params.low_speed_blend_end,
                    0.0,
                    1.0,
                )
                low_speed_target = (
                    blend_weight * self._params.idle_visual_speed + (1.0 - blend_weight) * physical_speeds[active]
                )
                target_visual_speeds[active] = np.maximum(physical_speeds[active], low_speed_target)
        else:
            target_visual_speeds = self._target_visual_speeds
            target_visual_speeds[:] = np.maximum(0.0, self._rotor_angular_velocity)
        self._advance_visual_rotors(
            mocap_ids=self._rotor_mocap_ids,
            offsets_b=self._rotor_visual_offsets,
            mount_rot=self._rotor_mount_quats,
            rotor_angles=self._rotor_angle,
            visual_speeds=self._visual_rotor_angular_velocity,
            target_speeds=target_visual_speeds,
            spin_directions=self._rotor_direction,
            spin_axes_local=self._rotor_spin_axes_local,
            smoothing_tc=self._params.visual_speed_smoothing_tc,
            base_pos=self._mj_data.xpos[self._base_link_id],
            base_quat=self._mj_data.xquat[self._base_link_id],
            rb_mat=self._mj_data.xmat[self._base_link_id].reshape(3, 3),
        )

    def _get_visual_rotor_angle(self) -> np.ndarray:
        return self._rotor_angle.copy()

    def _get_visual_rotor_speed(self) -> np.ndarray:
        return self._visual_rotor_angular_velocity.copy()
