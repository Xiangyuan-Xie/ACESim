import platform
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence

import genesis as gs
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.genesis.genesis_env import GenesisEnv
from acesim.utils.px4_interface import PX4Interface


@dataclass
class MultirotorParams:
    rotor_direction: np.ndarray
    motor_constant: float
    moment_constant: float
    rotor_drag_coeff: float
    rolling_moment_coeff: float
    rotor_radius: float


class MultirotorEnv(GenesisEnv):
    IDLE_VISUAL_SPEED = 55.0
    GPS_LAT_START = 39.98329
    GPS_LON_START = 116.34745
    GPS_ALT_START = 50.0
    HIL_SENSOR_RATE_HZ = 250.0
    MAG_RATE_HZ = 100.0
    BARO_RATE_HZ = 50.0
    GPS_RATE_HZ = 30.0

    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        self._px4_interface = PX4Interface()
        self._apply_multirotor_overrides(config_loader.get_asset_params())
        self._runtime_initialized = False
        self._base_link = None
        self._rotor_links: list[Any] = []
        self._rotor_offsets = np.zeros((0, 3))
        self._rotor_count = 0
        self._desired_rotor_angular_velocity = np.zeros(0)
        self._rotor_angular_velocity = np.zeros(0)
        self._rotor_direction = np.zeros(0)
        self._arm_dofs_idx_local = None
        self._configure_update_timing()
        self._initialize_sensor_buffers()
        if platform.system() == "Windows":
            print("[ACESim] Genesis backend initialized on Windows.")
        else:
            print("[ACESim] Genesis backend initialized on Linux.")

    def _load_multirotor_params(self, asset_config: Dict[str, Any]):
        config = asset_config.get("multirotor", asset_config)
        rotor_direction = np.array(config["rotor_direction"], dtype=float)
        return MultirotorParams(
            rotor_direction=rotor_direction,
            motor_constant=float(config["motor_constant"]),
            moment_constant=float(config["moment_constant"]),
            rotor_drag_coeff=float(config["rotor_drag_coeff"]),
            rolling_moment_coeff=float(config["rolling_moment_coeff"]),
            rotor_radius=float(config["rotor_radius"]),
        )

    def _apply_multirotor_overrides(self, asset_params: Dict[str, Any]):
        self._params = self._load_multirotor_params(asset_params)
        if "idle_visual_speed" in asset_params:
            self.IDLE_VISUAL_SPEED = float(asset_params["idle_visual_speed"])
        if "gps_lat_start" in asset_params:
            self.GPS_LAT_START = float(asset_params["gps_lat_start"])
        if "gps_lon_start" in asset_params:
            self.GPS_LON_START = float(asset_params["gps_lon_start"])
        if "gps_alt_start" in asset_params:
            self.GPS_ALT_START = float(asset_params["gps_alt_start"])

    def _to_numpy(self, value, fallback_dim: Optional[int] = None):
        if value is None:
            if fallback_dim is None:
                return np.array([], dtype=float)
            return np.zeros(fallback_dim, dtype=float)
        array = np.asarray(value, dtype=float)
        if array.ndim > 1 and array.shape[0] == 1:
            array = array[0]
        if fallback_dim is not None and array.size == 0:
            return np.zeros(fallback_dim, dtype=float)
        return array

    def _quat_to_rotation(self, quat: np.ndarray):
        q = self._to_numpy(quat, fallback_dim=4).reshape(-1)
        if q.size != 4:
            return Rotation.identity()
        try:
            return Rotation.from_quat(q, scalar_first=True)
        except TypeError:
            return Rotation.from_quat([q[1], q[2], q[3], q[0]])
        except ValueError:
            return Rotation.identity()

    def _body_flu_to_frd(self, vec_flu: np.ndarray):
        return np.array([vec_flu[0], -vec_flu[1], -vec_flu[2]], dtype=float)

    def _world_to_ned(self, vec_world: np.ndarray):
        return np.array([vec_world[0], -vec_world[1], -vec_world[2]], dtype=float)

    def _configure_update_timing(self):
        self._hil_sensor_period_s = 1.0 / self.HIL_SENSOR_RATE_HZ
        self._mag_period_s = 1.0 / self.MAG_RATE_HZ
        self._baro_period_s = 1.0 / self.BARO_RATE_HZ
        self._gps_period_s = 1.0 / self.GPS_RATE_HZ
        self._hil_sensor_elapsed_s = 0.0
        self._mag_elapsed_s = 0.0
        self._baro_elapsed_s = 0.0
        self._gps_elapsed_s = 0.0
        self._hil_sensor_sent = False

    def _initialize_sensor_buffers(self):
        self._last_accel_frd = np.zeros(3)
        self._last_gyro_frd = np.zeros(3)
        self._last_mag_frd = np.zeros(3)
        self._last_baro_altitude_m = self.GPS_ALT_START
        self._prev_base_linvel_w = np.zeros(3)
        self._has_prev_vel = False

    def _resolve_link(self, name: str):
        if self._robot is None:
            return None
        try:
            return self._robot.get_link(name)
        except Exception:
            return None

    def _resolve_joint_dof_indices(self, joint_names: Sequence[str]):
        if self._robot is None:
            return []
        dofs_idx_local = []
        for name in joint_names:
            try:
                joint = self._robot.get_joint(name)
            except Exception:
                continue
            idx = getattr(joint, "dof_idx_local", None)
            if idx is None:
                continue
            dofs_idx_local.append(int(idx))
        return dofs_idx_local

    def _control_dofs_position(self, target: np.ndarray, dofs_idx_local: Sequence[int]):
        if self._robot is None or len(dofs_idx_local) == 0:
            return
        command = np.asarray(target, dtype=float)
        idxs = np.asarray(dofs_idx_local, dtype=int)
        control_fn = getattr(self._robot, "control_dofs_position", None)
        if control_fn is None:
            return
        try:
            control_fn(command, dofs_idx_local=idxs)
            return
        except TypeError:
            pass
        try:
            control_fn(command, idxs)
            return
        except TypeError:
            pass

    def _get_link_pos(self, link):
        if link is None:
            return np.zeros(3)
        getter = getattr(link, "get_pos", None)
        if getter is None:
            return np.zeros(3)
        try:
            return self._to_numpy(getter(), fallback_dim=3)
        except Exception:
            return np.zeros(3)

    def _get_link_quat(self, link):
        if link is None:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        getter = getattr(link, "get_quat", None)
        if getter is None:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        try:
            return self._to_numpy(getter(), fallback_dim=4)
        except Exception:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    def _get_link_vel(self, link):
        if link is None:
            return np.zeros(3)
        getter = getattr(link, "get_vel", None)
        if getter is None:
            return np.zeros(3)
        try:
            return self._to_numpy(getter(), fallback_dim=3)
        except Exception:
            return np.zeros(3)

    def _get_link_ang(self, link):
        if link is None:
            return np.zeros(3)
        getter = getattr(link, "get_ang", None)
        if getter is None:
            return np.zeros(3)
        try:
            return self._to_numpy(getter(), fallback_dim=3)
        except Exception:
            return np.zeros(3)

    def _ensure_scene(self):
        if self._scene is not None:
            return
        self._scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self._dt_s),
            show_viewer=False,
        )
        self._robot = self._scene.add_entity(gs.morphs.MJCF(file=str(self._merged_xml_path)))
        self._scene.build()
        self._scene_show_viewer = False
        self._runtime_initialized = False

    def _initialize_runtime_handles(self):
        self._base_link = self._resolve_link("base_link")
        rotor_links = []
        for idx in range(1, 13):
            link = self._resolve_link(f"rotor_{idx}")
            if link is not None:
                rotor_links.append(link)
        self._rotor_links = rotor_links
        self._rotor_count = len(self._rotor_links)
        self._desired_rotor_angular_velocity = np.zeros(self._rotor_count)
        self._rotor_angular_velocity = np.zeros(self._rotor_count)
        direction = np.asarray(self._params.rotor_direction, dtype=float)
        if direction.size != self._rotor_count:
            base = np.array([1.0, -1.0], dtype=float)
            direction = np.tile(base, int(np.ceil(max(self._rotor_count, 1) / base.size)))[: self._rotor_count]
        self._rotor_direction = direction
        base_pos = self._get_link_pos(self._base_link)
        base_quat = self._get_link_quat(self._base_link)
        rb = self._quat_to_rotation(base_quat)
        rb_inv = rb.inv()
        offsets = []
        for link in self._rotor_links:
            rotor_pos = self._get_link_pos(link)
            offsets.append(rb_inv.apply(rotor_pos - base_pos))
        self._rotor_offsets = np.array(offsets, dtype=float) if offsets else np.zeros((0, 3))
        self._runtime_initialized = True
        self._has_prev_vel = False

    def _ensure_runtime_ready(self):
        self._ensure_scene()
        if not self._runtime_initialized:
            self._initialize_runtime_handles()

    def _get_base_kinematics(self):
        base_pos = self._get_link_pos(self._base_link)
        base_quat = self._get_link_quat(self._base_link)
        rb = self._quat_to_rotation(base_quat)
        v_com_w = self._get_link_vel(self._base_link)
        omega_w = self._get_link_ang(self._base_link)
        return base_pos, rb, v_com_w, omega_w

    def _compute_inertial_observations(self):
        _, rb, v_com_w, omega_w = self._get_base_kinematics()
        if self._has_prev_vel:
            accel_w = (v_com_w - self._prev_base_linvel_w) / max(self._dt_s, 1e-6)
        else:
            accel_w = np.zeros(3)
            self._has_prev_vel = True
        self._prev_base_linvel_w = v_com_w.copy()
        gravity_w = np.array([0.0, 0.0, -9.81], dtype=float)
        proper_accel_w = accel_w - gravity_w
        accel_body_flu = rb.inv().apply(proper_accel_w)
        gyro_body_flu = rb.inv().apply(omega_w)
        accel_frd = self._body_flu_to_frd(accel_body_flu)
        gyro_frd = self._body_flu_to_frd(gyro_body_flu)
        mag_field_w = np.array([0.21523, 0.0, -0.42741], dtype=float)
        mag_body_flu = rb.inv().apply(mag_field_w)
        mag_frd = self._body_flu_to_frd(mag_body_flu)
        return accel_frd, gyro_frd, mag_frd

    def _update_sensors_and_send(self):
        dt = self._dt_s
        self._hil_sensor_sent = False
        self._mag_elapsed_s += dt
        self._baro_elapsed_s += dt
        self._hil_sensor_elapsed_s += dt
        fields = self._px4_interface.HIL_SENSOR_FIELDS_ACCEL | self._px4_interface.HIL_SENSOR_FIELDS_GYRO
        accel_frd, gyro_frd, mag_frd = self._compute_inertial_observations()
        if self._mag_elapsed_s >= self._mag_period_s:
            self._last_mag_frd = mag_frd + np.random.normal(0, 0.003, size=3)
            self._mag_elapsed_s -= self._mag_period_s
            fields |= self._px4_interface.HIL_SENSOR_FIELDS_MAG
        if self._baro_elapsed_s >= self._baro_period_s:
            base_pos = self._get_link_pos(self._base_link)
            self._last_baro_altitude_m = base_pos[2] + self.GPS_ALT_START + np.random.normal(0, 0.25)
            self._baro_elapsed_s -= self._baro_period_s
            fields |= self._px4_interface.HIL_SENSOR_FIELDS_BARO
        if self._hil_sensor_elapsed_s >= self._hil_sensor_period_s:
            self._last_accel_frd = accel_frd + np.random.normal(0, [0.00637, 0.00637, 0.00686])
            self._last_gyro_frd = gyro_frd + np.random.normal(0, 0.0008726646, size=3)
            self._px4_interface.send_hil_sensor(
                self._simulation_time_us,
                self._last_accel_frd,
                self._last_gyro_frd,
                self._last_mag_frd,
                self._last_baro_altitude_m,
                fields_updated=fields,
            )
            self._hil_sensor_elapsed_s -= self._hil_sensor_period_s
            self._hil_sensor_sent = True

    def _get_gps_pos_with_noise(self):
        pos = self._get_link_pos(self._base_link)
        pos_noisy = pos + np.random.normal(0, 0.01, size=3)
        lat = self.GPS_LAT_START + (pos_noisy[0] / 111319.9)
        lon = self.GPS_LON_START - (pos_noisy[1] / (111319.9 * np.cos(np.radians(self.GPS_LAT_START))))
        gps_alt = self.GPS_ALT_START + pos_noisy[2]
        return int(lat * 1e7), int(lon * 1e7), int(gps_alt * 1000)

    def _get_gps_vel_with_noise(self):
        vel_w = self._get_link_vel(self._base_link) + np.random.normal(0, 0.1, size=3)
        vel_ned = self._world_to_ned(vel_w)
        vn = vel_ned[0] * 100.0
        ve = vel_ned[1] * 100.0
        vd = vel_ned[2] * 100.0
        vel = float(np.linalg.norm([vn, ve, vd]))
        cog_rad = np.arctan2(ve, vn)
        cog_deg = (np.degrees(cog_rad) + 360.0) % 360.0
        return int(vel), int(vn), int(ve), int(vd), int(cog_deg * 100.0)

    def _update_gps_and_send(self):
        self._gps_elapsed_s += self._dt_s
        if self._gps_elapsed_s >= self._gps_period_s:
            lat_e7, lon_e7, alt_mm = self._get_gps_pos_with_noise()
            vel_cm_s, vn_cm_s, ve_cm_s, vd_cm_s, cog_cdeg = self._get_gps_vel_with_noise()
            self._px4_interface.send_hil_gps(
                self._simulation_time_us,
                lat_e7,
                lon_e7,
                alt_mm,
                vel_cm_s,
                vn_cm_s,
                ve_cm_s,
                vd_cm_s,
                cog_cdeg,
            )
            self._gps_elapsed_s -= self._gps_period_s

    def _update_px4_controls(self):
        if not self._hil_sensor_sent:
            return
        wait_timeout_s = self._hil_sensor_period_s * 2.0
        controls = self._px4_interface.read_actuator_controls(blocking=True, timeout_s=wait_timeout_s)
        if controls:
            count = min(len(controls), self._rotor_count)
            desired = np.zeros(self._rotor_count)
            desired[:count] = np.array(controls[:count]) * 1000.0
            self._desired_rotor_angular_velocity = np.clip(desired, 0.0, 1000.0)

    def _update_rotor_speed_state(self):
        dt = self._dt_s
        for i in range(self._rotor_count):
            diff = self._desired_rotor_angular_velocity[i] - self._rotor_angular_velocity[i]
            tc = 0.0125 if diff > 0 else 0.025
            self._rotor_angular_velocity[i] += diff * (1.0 - np.exp(-dt / tc))

    def _get_link_index(self, link):
        idx_local = getattr(link, "idx_local", None)
        if idx_local is not None:
            return int(idx_local)
        idx = getattr(link, "idx", None)
        if idx is not None:
            return int(idx)
        return None

    def _apply_link_forces(self, links, forces_w: np.ndarray):
        if self._robot is None or len(links) == 0:
            return
        indices = [self._get_link_index(link) for link in links]
        if any(idx is None for idx in indices):
            return
        apply_fn = getattr(self._robot, "apply_links_external_force", None)
        if apply_fn is not None:
            links_idx = np.array(indices, dtype=int)
            try:
                apply_fn(forces=forces_w, links_idx_local=links_idx)
                return
            except TypeError:
                pass
            try:
                apply_fn(forces_w, links_idx)
                return
            except TypeError:
                pass
        for link, force_w in zip(links, forces_w):
            link_apply = getattr(link, "apply_external_force", None)
            if link_apply is None:
                continue
            try:
                link_apply(force_w)
            except TypeError:
                continue

    def _apply_link_torques(self, links, torques_w: np.ndarray):
        if self._robot is None or len(links) == 0:
            return
        indices = [self._get_link_index(link) for link in links]
        if any(idx is None for idx in indices):
            return
        apply_fn = getattr(self._robot, "apply_links_external_torque", None)
        if apply_fn is not None:
            links_idx = np.array(indices, dtype=int)
            try:
                apply_fn(torques=torques_w, links_idx_local=links_idx)
                return
            except TypeError:
                pass
            try:
                apply_fn(torques_w, links_idx)
                return
            except TypeError:
                pass
        for link, torque_w in zip(links, torques_w):
            link_apply = getattr(link, "apply_external_torque", None)
            if link_apply is None:
                continue
            try:
                link_apply(torque_w)
            except TypeError:
                continue

    def _apply_motor_physics(self):
        if self._rotor_count == 0:
            return
        self._update_rotor_speed_state()
        base_pos, rb, _, _ = self._get_base_kinematics()
        rotor_forces_w = np.zeros((self._rotor_count, 3), dtype=float)
        rotor_torques_w = np.zeros((self._rotor_count, 3), dtype=float)
        for i in range(self._rotor_count):
            omega = self._rotor_angular_velocity[i]
            thrust = self._params.motor_constant * (omega**2)
            rotor_forces_w[i] = rb.apply(np.array([0.0, 0.0, thrust], dtype=float))
            torque_z = self._params.moment_constant * thrust * (-self._rotor_direction[i])
            rotor_torques_w[i] = rb.apply(np.array([0.0, 0.0, torque_z], dtype=float))
            _ = base_pos + rb.apply(self._rotor_offsets[i])
        self._apply_link_forces(self._rotor_links, rotor_forces_w)
        base_torque_w = np.sum(rotor_torques_w, axis=0).reshape(1, 3)
        if self._base_link is not None:
            self._apply_link_torques([self._base_link], base_torque_w)

    def _update_custom_control(self):
        return

    def step(self):
        self._ensure_runtime_ready()
        self._step_count += 1
        self._simulation_time_us += int(self._dt_s * 1e6)
        if not self._px4_interface.is_connected:
            self._px4_interface.update_connection_state()
        else:
            self._update_sensors_and_send()
            self._update_gps_and_send()
            self._update_px4_controls()
            self._apply_motor_physics()
        self._update_custom_control()
        self._scene.step()
