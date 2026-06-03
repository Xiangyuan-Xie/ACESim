"""Shared MuJoCo + PX4 HIL environment scaffolding."""

from __future__ import annotations

import re
from typing import Sequence

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.mj_env import MJEnv
from acesim.utils.frame import body_flu_to_frd
from acesim.utils.px4_sensor_scheduler import PX4SensorSample, PX4SensorScheduler
from acesim.utils.px4_transport import PX4ActuatorParams, PX4SensorParams, PX4Transport
from acesim.utils.vehicle_visual_state_publisher import (
    VehicleVisualState,
    VehicleVisualStatePublisher,
    VehicleVisualStreamParams,
)


class PX4MJEnv(MJEnv):
    """Common MuJoCo backend glue for PX4 HIL-driven vehicles."""

    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        try:
            self._asset_params = config_loader.get_asset_params()
            self._px4_sensor_params = PX4SensorParams.from_asset_params(
                self._asset_params,
                dynamic_hil_sensor_fields=False,
            )
            self._visual_stream_params = VehicleVisualStreamParams.from_asset_params(self._asset_params)
            self._px4_actuator_params = PX4ActuatorParams()

            self._px4_transport = PX4Transport(self._px4_actuator_params)
            self._visual_state_publisher = VehicleVisualStatePublisher(self._visual_stream_params)
            self._visual_publish_period_us = int(round(1_000_000.0 / self._visual_stream_params.rate_hz))
            self._next_visual_publish_time_us = 0
            self._initialize_px4_base_handles()
            self._initialize_vehicle_handles()
            self._sensor_scheduler = PX4SensorScheduler(
                self._px4_transport,
                self._sim_clock,
                self._px4_sensor_params,
                self.read_sensor_sample,
            )
            self._update_vehicle_visuals()
        except Exception:
            mujoco.set_mjcb_control(None)
            self._sim_clock.close()
            raise

    def _initialize_px4_base_handles(self) -> None:
        """Resolve shared base-link and sensor handles."""

        self._base_link_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        assert self._base_link_id >= 0, "MuJoCo model must define body 'base_link'"
        self._sensor_id_map = {
            "pos": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "framepos"),
            "quat": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "framequat"),
            "linvel": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "framelinvel"),
            "gyro": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro"),
            "accel": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "accelerometer"),
            "mag": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "magnetometer"),
        }
        missing = [name for name, sensor_id in self._sensor_id_map.items() if sensor_id < 0]
        assert not missing, f"MuJoCo model is missing required sensors: {', '.join(missing)}"

    def _initialize_vehicle_handles(self) -> None:
        """Resolve vehicle-specific joints, bodies, and runtime state."""

    def _resolve_named_rotor_bodies(
        self,
        *,
        allow_visual_fallback: bool,
    ) -> tuple[list[str], list[int], list[int]]:
        """Resolve rotor bodies from shared asset naming conventions.

        This helper intentionally stays in the MuJoCo layer because it depends on
        MJCF body/site discovery rules rather than on backend-independent math.
        ``allow_visual_fallback`` exists for legacy multirotor assets whose
        physical rotors only exist as mocap visual bodies, while UUV assets still
        require physical ``rotor_<i>`` bodies for force application.
        """

        site_indices: list[int] = []
        for site_id in range(self._mj_model.nsite):
            name = mujoco.mj_id2name(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, site_id)
            if not name:
                continue
            match = re.fullmatch(r"rotor_offset_(\d+)", name)
            if match:
                site_indices.append(int(match.group(1)))

        body_indices: list[int] = []
        for body_id in range(self._mj_model.nbody):
            name = mujoco.mj_id2name(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if not name:
                continue
            match = re.fullmatch(r"rotor_(\d+)(_vis)?", name)
            if match:
                body_indices.append(int(match.group(1)))

        rotor_indices = sorted(set(site_indices)) if site_indices else sorted(set(body_indices))
        body_names: list[str] = []
        body_ids: list[int] = []
        valid_indices: list[int] = []
        for rotor_index in rotor_indices:
            body_name = f"rotor_{rotor_index}"
            body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id >= 0:
                body_names.append(body_name)
                body_ids.append(body_id)
                valid_indices.append(rotor_index)
                continue
            if not allow_visual_fallback:
                continue
            visual_name = f"rotor_{rotor_index}_vis"
            visual_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, visual_name)
            if visual_id >= 0:
                body_names.append(visual_name)
                body_ids.append(visual_id)
                valid_indices.append(rotor_index)
        return body_names, body_ids, valid_indices

    def _resolve_visual_rotor_group(
        self,
        rotor_indices: Sequence[int],
        *,
        body_ids: Sequence[int] | None = None,
    ) -> tuple[list[int], np.ndarray, np.ndarray, list[Rotation]]:
        """Resolve physics offsets plus mocap mount poses for rotor visuals."""

        resolved_body_ids = list(body_ids) if body_ids is not None else []
        if not resolved_body_ids:
            for rotor_index in rotor_indices:
                body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}")
                if body_id < 0:
                    raise ValueError(f"Missing rotor body rotor_{rotor_index}")
                resolved_body_ids.append(body_id)

        mocap_ids: list[int] = []
        force_offsets: list[np.ndarray] = []
        visual_offsets: list[np.ndarray] = []
        mount_rot: list[Rotation] = []
        for rotor_index, body_id in zip(rotor_indices, resolved_body_ids):
            vis_body_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{rotor_index}_vis")
            site_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, f"rotor_offset_{rotor_index}")
            force_offset = (
                self._mj_model.site_pos[site_id].copy()
                if site_id >= 0
                else self._resolve_body_offset_from_base(body_id)
            )
            visual_body_id = vis_body_id if vis_body_id >= 0 else body_id
            if vis_body_id >= 0:
                vis_pos = self._mj_model.body_pos[vis_body_id].copy()
                vis_quat = self._mj_model.body_quat[vis_body_id].copy()
                if (
                    np.linalg.norm(vis_pos) <= 1e-9
                    and np.linalg.norm(vis_quat - np.array([1.0, 0.0, 0.0, 0.0], dtype=float)) <= 1e-9
                ):
                    # Legacy multirotor assets keep mocap visual bodies at the world
                    # origin and expect us to reuse the physical rotor mount pose.
                    visual_body_id = body_id
            mocap_ids.append(self._mj_model.body_mocapid[vis_body_id] if vis_body_id >= 0 else -1)
            force_offsets.append(np.asarray(force_offset, dtype=float))
            visual_offsets.append(self._resolve_body_offset_from_base(visual_body_id))
            mount_rot.append(self._resolve_body_rotation_from_base(visual_body_id))
        return (
            mocap_ids,
            np.asarray(force_offsets, dtype=float),
            np.asarray(visual_offsets, dtype=float),
            mount_rot,
        )

    def _advance_visual_rotors(
        self,
        *,
        mocap_ids: Sequence[int],
        offsets_b: np.ndarray,
        mount_rot: Sequence[Rotation],
        rotor_angles: np.ndarray,
        visual_speeds: np.ndarray,
        target_speeds: np.ndarray,
        spin_directions: np.ndarray,
        spin_axes_local: np.ndarray,
        smoothing_tc: float,
    ) -> None:
        """Advance rotor/thruster mocap visuals without spinning physical rigid bodies."""

        if len(mocap_ids) == 0:
            return

        base_pos, _, rb, _, _, _, _ = self._get_base_kinematics()
        dt_s = self._mj_model.opt.timestep
        axes = np.asarray(spin_axes_local, dtype=float)
        if axes.ndim == 1:
            axes = np.tile(axes, (len(mocap_ids), 1))
        norms = np.linalg.norm(axes, axis=1, keepdims=True)
        norms = np.where(norms <= 1e-12, 1.0, norms)
        axes = axes / norms

        for i, mocap_id in enumerate(mocap_ids):
            if mocap_id < 0:
                continue
            if smoothing_tc > 0.0:
                delta = float(target_speeds[i] - visual_speeds[i])
                visual_speeds[i] += delta * (1.0 - np.exp(-dt_s / smoothing_tc))
            else:
                visual_speeds[i] = float(target_speeds[i])
            rotor_angles[i] += float(visual_speeds[i] * spin_directions[i] * dt_s)
            spin = Rotation.from_rotvec(axes[i] * rotor_angles[i])
            self._mj_data.mocap_pos[mocap_id] = base_pos + rb.apply(offsets_b[i])
            self._mj_data.mocap_quat[mocap_id] = (rb * mount_rot[i] * spin).as_quat(scalar_first=True)

    def _get_sensor_raw(self, name: str) -> np.ndarray:
        sensor_id = self._sensor_id_map[name]
        adr = self._mj_model.sensor_adr[sensor_id]
        dim = self._mj_model.sensor_dim[sensor_id]
        return self._mj_data.sensordata[adr : adr + dim].copy()

    def _read_diff_pressure_hpa(self) -> float | None:
        return None

    def _read_sensor_temperature_celsius(self) -> float:
        return 25.0

    def read_sensor_sample(self) -> PX4SensorSample:
        """Read the current canonical PX4 HIL sample."""

        accel_flu = self._get_sensor_raw("accel")
        gyro_flu = self._get_sensor_raw("gyro")
        mag_flu = self._get_sensor_raw("mag") * 10000.0
        return PX4SensorSample(
            accel_frd=body_flu_to_frd(accel_flu),
            gyro_frd=body_flu_to_frd(gyro_flu),
            mag_frd=body_flu_to_frd(mag_flu),
            position_world_m=self._get_sensor_raw("pos"),
            velocity_world_mps=self._get_sensor_raw("linvel"),
            attitude_world_quat=self._get_sensor_raw("quat"),
            diff_pressure_hpa=self._read_diff_pressure_hpa(),
            temperature_celsius=self._read_sensor_temperature_celsius(),
        )

    def _get_visual_rotor_angle(self) -> np.ndarray:
        return np.zeros(0, dtype=float)

    def _get_visual_rotor_speed(self) -> np.ndarray:
        return np.zeros(0, dtype=float)

    def read_visual_state(self) -> VehicleVisualState:
        return VehicleVisualState(
            timestamp_us=self._simulation_time_us,
            position_world_m_nwu=self._get_sensor_raw("pos"),
            attitude_world_quat_scalar_first=self._get_sensor_raw("quat"),
            rotor_angle_rad=self._get_visual_rotor_angle(),
            rotor_visual_speed_radps=self._get_visual_rotor_speed(),
        )

    def _get_base_kinematics(
        self,
    ) -> tuple[np.ndarray, np.ndarray, Rotation, Rotation, np.ndarray, np.ndarray, np.ndarray]:
        """Return base pose, rotation, and velocities."""

        base_pos = self._get_sensor_raw("pos")
        base_quat = self._get_sensor_raw("quat")
        rb = Rotation.from_quat(base_quat, scalar_first=True)
        rb_inv = rb.inv()
        v_com_w = self._get_sensor_raw("linvel")
        omega_r = self._get_sensor_raw("gyro")
        omega_w = rb.apply(omega_r)
        return base_pos, base_quat, rb, rb_inv, v_com_w, omega_r, omega_w

    def _clear_applied_wrenches(self) -> None:
        self._mj_data.xfrc_applied[:] = 0.0
        self._mj_data.qfrc_applied[:] = 0.0

    def _resolve_body_offset_from_base(self, body_id: int) -> np.ndarray:
        """Return a body's current offset from ``base_link`` in body coordinates."""

        base_pos, _, _, rb_inv, _, _, _ = self._get_base_kinematics()
        body_pos = self._mj_data.xpos[body_id].copy()
        return rb_inv.apply(body_pos - base_pos)

    def _resolve_body_rotation_from_base(self, body_id: int) -> Rotation:
        """Return a body's static rotation relative to ``base_link``."""

        if body_id == self._base_link_id:
            return Rotation.identity()

        rotation = Rotation.identity()
        current_id = int(body_id)
        while current_id > 0 and current_id != self._base_link_id:
            local_quat = self._mj_model.body_quat[current_id].copy()
            rotation = Rotation.from_quat(local_quat, scalar_first=True) * rotation
            current_id = int(self._mj_model.body_parentid[current_id])

        if current_id == self._base_link_id:
            return rotation

        # Mocap-only visual bodies live at world scope in the generated MJCF.
        # Fall back to the current world pose to recover their static transform
        # relative to base_link.
        base_quat = self._mj_data.xquat[self._base_link_id].copy()
        body_quat = self._mj_data.xquat[body_id].copy()
        rb_inv = Rotation.from_quat(base_quat, scalar_first=True).inv()
        return rb_inv * Rotation.from_quat(body_quat, scalar_first=True)

    def _apply_body_wrench(
        self,
        force_body_flu: Sequence[float],
        torque_body_flu: Sequence[float],
        point_body_flu: Sequence[float] | None = None,
    ) -> None:
        """Apply one body-frame wrench to the base body."""

        base_pos, _, rb, _, _, _, _ = self._get_base_kinematics()
        point_body = np.zeros(3, dtype=float) if point_body_flu is None else np.asarray(point_body_flu, dtype=float)
        point_world = base_pos + rb.apply(point_body)
        mujoco.mj_applyFT(
            self._mj_model,
            self._mj_data,
            rb.apply(np.asarray(force_body_flu, dtype=float)),
            rb.apply(np.asarray(torque_body_flu, dtype=float)),
            point_world,
            self._base_link_id,
            self._mj_data.qfrc_applied,
        )

    def _apply_world_wrenches(
        self,
        positions_world_m: np.ndarray,
        forces_world_n: np.ndarray,
        moments_world_nm: np.ndarray,
    ) -> None:
        """Apply a batch of world-frame wrenches to ``base_link``.

        Keeping this helper near the MuJoCo callback logic makes the force
        application sites consistent without moving engine-specific API calls into
        ``acesim.utils``.
        """

        for position_world, force_world, moment_world in zip(positions_world_m, forces_world_n, moments_world_nm):
            mujoco.mj_applyFT(
                self._mj_model,
                self._mj_data,
                np.asarray(force_world, dtype=float),
                np.asarray(moment_world, dtype=float),
                np.asarray(position_world, dtype=float),
                self._base_link_id,
                self._mj_data.qfrc_applied,
            )

    def _get_wind_velocity_w(self) -> np.ndarray:
        return self._mj_model.opt.wind.copy()

    def _compute_lumped_drag_force_w(
        self,
        rb: Rotation,
        rb_inv: Rotation,
        v_com_w: np.ndarray,
    ) -> np.ndarray:
        params = getattr(self, "_lumped_drag_params", None)
        if params is None or not params.enabled:
            return np.zeros(3, dtype=float)
        mass = float(np.sum(self._mj_model.body_mass))
        v_air_com_w = np.asarray(v_com_w, dtype=float) - self._get_wind_velocity_w()
        v_air_com_b = rb_inv.apply(v_air_com_w)
        force_b = -mass * params.d * v_air_com_b
        return rb.apply(force_b)

    def _apply_lumped_drag_wrench(
        self, base_pos: np.ndarray, rb: Rotation, rb_inv: Rotation, v_com_w: np.ndarray
    ) -> None:
        force_w = self._compute_lumped_drag_force_w(rb, rb_inv, v_com_w)
        if np.any(force_w):
            self._apply_world_wrenches(
                np.asarray([base_pos], dtype=float),
                np.asarray([force_w], dtype=float),
                np.zeros((1, 3), dtype=float),
            )

    def _actuator_channel_count(self) -> int:
        raise NotImplementedError

    def _handle_applied_actuator_controls(self, controls: np.ndarray) -> None:
        """Map released PX4 controls onto vehicle state."""

    def _update_px4_controls(self) -> None:
        channel_count = self._actuator_channel_count()
        self._px4_transport.update_actuator_commands(self._simulation_time_us, channel_count)
        controls = self._px4_transport.read_applied_actuator_controls(channel_count)
        if controls is not None:
            self._handle_applied_actuator_controls(controls)

    def _apply_vehicle_physics(self) -> None:
        raise NotImplementedError

    def _update_vehicle_visuals(self) -> None:
        """Advance visualization-only state."""

    def _update_custom_control(self) -> None:
        """Hook for subclasses that add extra actuation beyond the vehicle."""

    def _control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._step_count += 1
        self._advance_simulation_time_seconds(model.opt.timestep)
        if not self._px4_transport.is_connected:
            self._px4_transport.update_connection_state()
        else:
            self._sensor_scheduler.update()
            self._update_px4_controls()
            self._apply_vehicle_physics()
        self._update_custom_control()
        self._update_vehicle_visuals()

    def _publish_visual_state_if_due(self) -> None:
        if not self._visual_state_publisher.is_enabled:
            return
        current_time_us = self._simulation_time_us
        while current_time_us >= self._next_visual_publish_time_us:
            self._visual_state_publisher.publish(self.read_visual_state())
            self._next_visual_publish_time_us += self._visual_publish_period_us

    def step(self) -> None:
        super().step()
        self._publish_visual_state_if_due()

    def close(self) -> None:
        self._px4_transport.close()
        self._visual_state_publisher.close()
        super().close()
