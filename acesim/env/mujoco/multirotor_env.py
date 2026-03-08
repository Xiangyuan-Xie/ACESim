import platform
import re
from dataclasses import dataclass
from typing import Any, Dict, Sequence

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.mujoco_env import MujocoEnv
from acesim.utils.px4_interface import PX4Interface


@dataclass
class MultirotorParams:
    rotor_direction: np.ndarray
    motor_constant: float
    moment_constant: float
    rotor_drag_coeff: float
    rolling_moment_coeff: float


class MultirotorEnv(MujocoEnv):
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

        print("-" * 50)
        if platform.system() == "Windows":
            print("[ACESim] Windows detected. Please execute the following in WSL:")
            print("  1. Bind device: usbipd list / usbipd bind -b <BUSID> / usbipd attach --wsl --busid <BUSID>")
            print("  2. Start backend: ros2 launch px4_sim_ros2 windows.launch.py")
        else:
            print("[ACESim] Linux detected. Please execute the following:")
            print("  1. Start backend: ros2 launch px4_sim_ros2 linux.launch.py")
        print("-" * 50)

        self._initialize_multirotor_handles()
        self._rotor_offsets = self._load_rotor_offsets()
        self._initialize_rotor_state()
        self._configure_update_timing()
        self._initialize_sensor_buffers()

    def _load_multirotor_params(self, asset_config: Dict[str, Any]):
        config = asset_config.get("multirotor", asset_config)
        rotor_direction = np.array(config["rotor_direction"], dtype=float)
        return MultirotorParams(
            rotor_direction=rotor_direction,
            motor_constant=float(config["motor_constant"]),
            moment_constant=float(config["moment_constant"]),
            rotor_drag_coeff=float(config["rotor_drag_coeff"]),
            rolling_moment_coeff=float(config["rolling_moment_coeff"]),
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

    def _initialize_multirotor_handles(self):
        self._base_link_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        rotor_indices = self._find_rotor_indices_from_sites()
        if not rotor_indices:
            rotor_indices = self._find_rotor_indices_from_bodies()
        self._rotor_body_names, self._rotor_body_ids, self._rotor_indices = self._resolve_rotor_bodies(rotor_indices)
        self._rotor_mocap_ids = [
            self._mj_model.body_mocapid[b_id] if b_id >= 0 else -1 for b_id in self._rotor_body_ids
        ]
        self._sensor_id_map = {
            "pos": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "framepos"),
            "quat": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "framequat"),
            "linvel": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "framelinvel"),
            "gyro": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro"),
            "accel": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "accelerometer"),
            "mag": mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "magnetometer"),
        }

    def _initialize_sensor_buffers(self):
        self._last_accel_frd = np.zeros(3)
        self._last_gyro_frd = np.zeros(3)
        self._last_mag_frd = np.zeros(3)
        self._last_baro_altitude_m = self.GPS_ALT_START
        self._last_mag_frd = self._get_mag_with_noise()
        self._last_accel_frd = self._get_accel_with_noise()
        self._last_gyro_frd = self._get_gyro_with_noise()

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

    def _find_rotor_indices_from_sites(self):
        indices = []
        for site_id in range(self._mj_model.nsite):
            name = mujoco.mj_id2name(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, site_id)
            if not name:
                continue
            match = re.fullmatch(r"rotor_offset_(\d+)", name)
            if match:
                indices.append(int(match.group(1)))
        return sorted(set(indices))

    def _find_rotor_indices_from_bodies(self):
        indices = []
        for body_id in range(self._mj_model.nbody):
            name = mujoco.mj_id2name(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if not name:
                continue
            match = re.fullmatch(r"rotor_(\d+)(_vis)?", name)
            if match:
                indices.append(int(match.group(1)))
        return sorted(set(indices))

    def _resolve_rotor_bodies(self, rotor_indices: Sequence[int]):
        body_names = []
        body_ids = []
        valid_indices = []
        for idx in rotor_indices:
            vis_name = f"rotor_{idx}_vis"
            raw_name = f"rotor_{idx}"
            vis_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, vis_name)
            if vis_id >= 0:
                body_names.append(vis_name)
                body_ids.append(vis_id)
                valid_indices.append(idx)
                continue
            raw_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, raw_name)
            if raw_id >= 0:
                body_names.append(raw_name)
                body_ids.append(raw_id)
                valid_indices.append(idx)
        return body_names, body_ids, valid_indices

    def _load_rotor_offsets(self):
        rotor_offsets = []
        base_pos = self._mj_model.body_pos[self._base_link_id]
        for idx, body_id in zip(self._rotor_indices, self._rotor_body_ids):
            site_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, f"rotor_offset_{idx}")
            if site_id >= 0:
                rotor_offsets.append(self._mj_model.site_pos[site_id].copy())
            else:
                rotor_offsets.append(self._mj_model.body_pos[body_id] - base_pos)
        return np.array(rotor_offsets)

    def _initialize_rotor_state(self):
        self._rotor_count = len(self._rotor_body_ids)
        self._desired_rotor_angular_velocity = np.zeros(self._rotor_count)
        self._rotor_angular_velocity = np.zeros(self._rotor_count)
        self._rotor_angle = np.zeros(self._rotor_count)
        direction = np.asarray(self._params.rotor_direction, dtype=float)
        if direction.size != self._rotor_count:
            base = np.array([1.0, -1.0])
            direction = np.tile(base, int(np.ceil(self._rotor_count / base.size)))[: self._rotor_count]
        self._rotor_direction = direction

    def _get_sensor_raw(self, name: str):
        sensor_id = self._sensor_id_map.get(name, -1)
        if sensor_id == -1:
            return np.zeros(3)
        adr = self._mj_model.sensor_adr[sensor_id]
        return self._mj_data.sensordata[adr : adr + self._mj_model.sensor_dim[sensor_id]].copy()

    def _get_accel_with_noise(self):
        sensor_value = self._get_sensor_raw("accel")
        sensor_value += np.random.normal(0, [0.00637, 0.00637, 0.00686])
        return np.array([sensor_value[0], -sensor_value[1], -sensor_value[2]])

    def _get_gyro_with_noise(self):
        sensor_value = self._get_sensor_raw("gyro")
        sensor_value += np.random.normal(0, 0.0008726646, size=3)
        return np.array([sensor_value[0], -sensor_value[1], -sensor_value[2]])

    def _get_mag_with_noise(self):
        sensor_value = self._get_sensor_raw("mag") * 10000.0
        sensor_value += np.random.normal(0, 0.003, size=3)
        return np.array([sensor_value[0], -sensor_value[1], -sensor_value[2]])

    def _update_sensors_and_send(self):
        dt = self._mj_model.opt.timestep
        self._hil_sensor_sent = False
        self._mag_elapsed_s += dt
        self._baro_elapsed_s += dt
        self._hil_sensor_elapsed_s += dt
        if self._mag_elapsed_s >= self._mag_period_s:
            self._last_mag_frd = self._get_mag_with_noise()
            self._mag_elapsed_s -= self._mag_period_s
        if self._baro_elapsed_s >= self._baro_period_s:
            position_sensor = self._get_sensor_raw("pos")
            self._last_baro_altitude_m = position_sensor[2] + self.GPS_ALT_START + np.random.normal(0, 0.25)
            self._baro_elapsed_s -= self._baro_period_s
        if self._hil_sensor_elapsed_s >= self._hil_sensor_period_s:
            self._last_accel_frd = self._get_accel_with_noise()
            self._last_gyro_frd = self._get_gyro_with_noise()
            self._px4_interface.send_hil_sensor(
                self._simulation_time_us,
                self._last_accel_frd,
                self._last_gyro_frd,
                self._last_mag_frd,
                self._last_baro_altitude_m,
            )
            self._hil_sensor_elapsed_s -= self._hil_sensor_period_s
            self._hil_sensor_sent = True

    def _update_gps_and_send(self):
        self._gps_elapsed_s += self._mj_model.opt.timestep
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
        if self._hil_sensor_sent:
            wait_timeout_s = self._hil_sensor_period_s * 2.0
            controls = self._px4_interface.read_actuator_controls(blocking=True, timeout_s=wait_timeout_s)
            if controls:
                count = min(len(controls), self._rotor_count)
                desired = np.zeros(self._rotor_count)
                desired[:count] = np.array(controls[:count]) * 1000
                self._desired_rotor_angular_velocity = np.clip(desired, 0, 1000)

    def _get_gps_pos_with_noise(self):
        pos = self._get_sensor_raw("pos")
        pos_noisy = pos + np.random.normal(0, 0.01, size=3)
        lat = self.GPS_LAT_START + (pos_noisy[0] / 111319.9)
        lon = self.GPS_LON_START - (pos_noisy[1] / (111319.9 * np.cos(np.radians(self.GPS_LAT_START))))
        gps_alt = self.GPS_ALT_START + pos_noisy[2]
        return int(lat * 1e7), int(lon * 1e7), int(gps_alt * 1000)

    def _get_gps_vel_with_noise(self):
        vel_w = self._get_sensor_raw("linvel")
        vel_w = vel_w + np.random.normal(0, 0.1, size=3)
        vn = vel_w[0] * 100.0
        ve = -vel_w[1] * 100.0
        vd = -vel_w[2] * 100.0
        vel = float(np.linalg.norm([vn, ve, vd]))
        cog_rad = np.arctan2(ve, vn)
        cog_deg = (np.degrees(cog_rad) + 360.0) % 360.0
        return int(vel), int(vn), int(ve), int(vd), int(cog_deg * 100.0)

    def _apply_motor_physics(self):
        dt = self._mj_model.opt.timestep
        for i in range(self._rotor_count):
            diff = self._desired_rotor_angular_velocity[i] - self._rotor_angular_velocity[i]
            tc = 0.0125 if diff > 0 else 0.025
            self._rotor_angular_velocity[i] += diff * (1.0 - np.exp(-dt / tc))
        base_pos = self._get_sensor_raw("pos")
        base_quat = self._get_sensor_raw("quat")
        Rb = Rotation.from_quat(base_quat, scalar_first=True)
        v_com_w = self._get_sensor_raw("linvel")
        omega_r = self._get_sensor_raw("gyro")
        omega_w = Rb.apply(omega_r)
        self._mj_data.xfrc_applied[self._base_link_id][:] = 0.0
        self._mj_data.qfrc_applied[:] = 0.0
        for i in range(self._rotor_count):
            r_off_w = Rb.apply(self._rotor_offsets[i])
            v_point_w = v_com_w + np.cross(omega_w, r_off_w)
            v_point_r = Rb.inv().apply(v_point_w)
            v_planar_r = np.array([v_point_r[0], v_point_r[1], 0.0])
            omega = self._rotor_angular_velocity[i]
            direction = self._rotor_direction[i]
            thrust = self._params.motor_constant * (omega**2)
            torque_z_r = self._params.moment_constant * thrust * (-direction)
            f_drag_r = -self._params.rotor_drag_coeff * omega * v_planar_r
            m_rolling_r = -self._params.rolling_moment_coeff * omega * v_planar_r
            f_total_w = Rb.apply(np.array([0.0, 0.0, thrust]) + f_drag_r)
            m_react_w = Rb.apply(np.array([0.0, 0.0, torque_z_r]) + m_rolling_r)
            pos_w = base_pos + r_off_w
            mujoco.mj_applyFT(
                self._mj_model,
                self._mj_data,
                f_total_w,
                m_react_w,
                pos_w,
                self._base_link_id,
                self._mj_data.qfrc_applied,
            )

    def _update_rotor_visuals(self):
        base_pos = self._get_sensor_raw("pos")
        base_quat = self._get_sensor_raw("quat")
        Rb = Rotation.from_quat(base_quat, scalar_first=True)
        armed = self._px4_interface.update_arming_state()
        for i in range(self._rotor_count):
            mocap_id = self._rotor_mocap_ids[i]
            if mocap_id < 0:
                continue
            visual_speed = self._rotor_angular_velocity[i]
            if armed and visual_speed < self.IDLE_VISUAL_SPEED:
                visual_speed = self.IDLE_VISUAL_SPEED
            self._rotor_angle[i] += visual_speed * self._rotor_direction[i] * self._mj_model.opt.timestep
            spin = Rotation.from_rotvec([0.0, 0.0, self._rotor_angle[i]])
            q_total = (Rb * spin).as_quat(scalar_first=True)
            pos_w = base_pos + Rb.apply(self._rotor_offsets[i])
            self._mj_data.mocap_pos[mocap_id] = pos_w
            self._mj_data.mocap_quat[mocap_id] = q_total

    def _update_custom_control(self):
        pass

    def _control(self, model: mujoco.MjModel, data: mujoco.MjData):
        self._step_count += 1
        self._simulation_time_us += int(model.opt.timestep * 1e6)
        if not self._px4_interface.is_connected:
            self._px4_interface.update_connection_state()
        else:
            self._update_sensors_and_send()
            self._update_gps_and_send()
            self._update_px4_controls()
            self._apply_motor_physics()
        self._update_custom_control()
        self._update_rotor_visuals()

    def close(self):
        pass
