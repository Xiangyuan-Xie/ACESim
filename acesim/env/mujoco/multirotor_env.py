"""MuJoCo multirotor environment with PX4 HIL integration."""

import re
from dataclasses import dataclass
from typing import Tuple

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.mujoco_env import MujocoEnv
from acesim.utils.frame import body_flu_to_frd
from acesim.utils.px4_interface import PX4ActuatorParams, PX4Interface, PX4SensorParams
from acesim.utils.px4_sensor_bridge import PX4SensorBridge, PX4SensorSample


@dataclass
class MultirotorParams:
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
    max_relative_airspeed_mps: float


class MultirotorEnv(MujocoEnv):
    """MuJoCo multirotor backend with PX4 HIL sensor and actuator integration."""

    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        asset_params = config_loader.get_asset_params()
        config = asset_params.get("multirotor", asset_params)
        self._params = MultirotorParams(
            rotor_direction=np.array(config["rotor_direction"], dtype=float),
            motor_constant=float(config["motor_constant"]),
            moment_constant=float(config["moment_constant"]),
            rotor_drag_coeff=float(config["rotor_drag_coeff"]),
            rolling_moment_coeff=float(config["rolling_moment_coeff"]),
            rotor_radius=float(config["rotor_radius"]),
            time_constant_up=float(config.get("time_constant_up")),
            time_constant_down=float(config.get("time_constant_down")),
            max_rot_velocity=float(config.get("max_rot_velocity")),
            max_relative_airspeed_mps=float(config.get("max_relative_airspeed_mps")),
        )
        self._px4_sensor_params = PX4SensorParams(
            idle_visual_speed=float(asset_params.get("idle_visual_speed", 55.0)),
            gps_home_lat_lon=(
                float(asset_params.get("gps_lat_start", 39.98329)),
                float(asset_params.get("gps_lon_start", 116.34745)),
            ),
            gps_alt_start=float(asset_params.get("gps_alt_start", 50.0)),
            hil_sensor_rate_hz=float(asset_params.get("hil_sensor_rate_hz", 250.0)),
            mag_rate_hz=float(asset_params.get("mag_rate_hz", 100.0)),
            baro_rate_hz=float(asset_params.get("baro_rate_hz", 50.0)),
            gps_rate_hz=float(asset_params.get("gps_rate_hz", 30.0)),
            dynamic_hil_sensor_fields=False,
        )
        delay_steps_range_raw = asset_params.get("motor_exec_delay_steps_range", (2, 6))
        delay_update_range_raw = asset_params.get("motor_exec_delay_update_steps_range", (4, 8))
        delay_transition_probs_raw = asset_params.get("motor_exec_delay_transition_probs", (0.15, 0.70, 0.15))

        assert len(delay_steps_range_raw) == 2, "motor_exec_delay_steps_range must contain [min, max]"
        assert len(delay_update_range_raw) == 2, "motor_exec_delay_update_steps_range must contain [min, max]"
        assert len(delay_transition_probs_raw) == 3, "motor_exec_delay_transition_probs must contain three values"

        # Explicit tuple construction keeps the runtime config shape checked and
        # gives mypy the fixed tuple lengths required by PX4ActuatorParams.
        delay_steps_range = (int(delay_steps_range_raw[0]), int(delay_steps_range_raw[1]))
        delay_update_range = (int(delay_update_range_raw[0]), int(delay_update_range_raw[1]))
        delay_transition_probs = (
            float(delay_transition_probs_raw[0]),
            float(delay_transition_probs_raw[1]),
            float(delay_transition_probs_raw[2]),
        )
        self._px4_actuator_params = PX4ActuatorParams(
            motor_cmd_rate_hz=float(asset_params.get("motor_cmd_rate_hz", 200.0)),
            motor_exec_delay_steps_range=delay_steps_range,
            motor_exec_delay_update_steps_range=delay_update_range,
            motor_exec_delay_transition_probs=delay_transition_probs,
            motor_exec_delay_drop_prob=float(asset_params.get("motor_exec_delay_drop_prob", 0.03)),
        )

        self._px4_interface = PX4Interface(self._px4_actuator_params)
        self._initialize_multirotor_handles()
        self._sensor_bridge = PX4SensorBridge(
            self._px4_interface,
            self._sim_clock,
            self._px4_sensor_params,
            self.read_sensor_sample,
        )
        self._rotor_offsets = self._load_rotor_offsets()
        self._rotor_count = len(self._rotor_body_ids)
        self._desired_rotor_angular_velocity = np.zeros(self._rotor_count)
        self._rotor_angular_velocity = np.zeros(self._rotor_count)
        self._rotor_angle = np.zeros(self._rotor_count)
        direction = np.asarray(self._params.rotor_direction, dtype=float)
        if direction.size != self._rotor_count:
            base = np.array([1.0, -1.0])
            direction = np.tile(base, int(np.ceil(self._rotor_count / base.size)))[: self._rotor_count]
        self._rotor_direction = direction

    def _initialize_multirotor_handles(self) -> None:
        """Resolve the base body, rotor bodies, mocap visuals, and required sensors."""

        self._base_link_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        assert self._base_link_id >= 0, "MuJoCo model must define body 'base_link'"

        self._rotor_body_names, self._rotor_body_ids, self._rotor_indices = self._resolve_rotor_bodies()
        assert self._rotor_body_ids, "No rotor bodies found. Expected rotor_<i> or rotor_<i>_vis bodies."

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
        missing = [name for name, sensor_id in self._sensor_id_map.items() if sensor_id < 0]
        assert not missing, f"MuJoCo model is missing required sensors: {', '.join(missing)}"

    def _resolve_rotor_bodies(self) -> tuple[list[str], list[int], list[int]]:
        """Find rotor bodies from rotor site names first, then fall back to body names."""

        site_indices = []
        for site_id in range(self._mj_model.nsite):
            name = mujoco.mj_id2name(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, site_id)
            if not name:
                continue
            match = re.fullmatch(r"rotor_offset_(\d+)", name)
            if match:
                site_indices.append(int(match.group(1)))

        body_indices = []
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

    def _load_rotor_offsets(self) -> np.ndarray:
        """Load rotor offsets in the base-link frame for force application and visuals."""

        rotor_offsets = []
        base_pos = self._mj_model.body_pos[self._base_link_id]
        for idx, body_id in zip(self._rotor_indices, self._rotor_body_ids):
            site_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, f"rotor_offset_{idx}")
            if site_id >= 0:
                rotor_offsets.append(self._mj_model.site_pos[site_id].copy())
            else:
                rotor_offsets.append(self._mj_model.body_pos[body_id] - base_pos)
        return np.array(rotor_offsets)

    def _get_sensor_raw(self, name: str) -> np.ndarray:
        sensor_id = self._sensor_id_map.get(name)
        adr = self._mj_model.sensor_adr[sensor_id]
        return self._mj_data.sensordata[adr : adr + self._mj_model.sensor_dim[sensor_id]].copy()

    def read_sensor_sample(self) -> PX4SensorSample:
        """Read one HIL sensor sample.

        MuJoCo exposes `framepos` and `framelinvel` in the simulator world frame
        used across this codebase, which is NWU. The IMU and magnetometer
        sensors are body-frame FLU and are converted here into PX4's FRD
        convention before they reach the bridge.
        """

        accel_flu = self._get_sensor_raw("accel")
        gyro_flu = self._get_sensor_raw("gyro")
        mag_flu = self._get_sensor_raw("mag") * 10000.0
        return PX4SensorSample(
            accel_frd=body_flu_to_frd(accel_flu),
            gyro_frd=body_flu_to_frd(gyro_flu),
            mag_frd=body_flu_to_frd(mag_flu),
            position_world_m=self._get_sensor_raw("pos"),
            velocity_world_mps=self._get_sensor_raw("linvel"),
        )

    def _update_px4_controls(self) -> None:
        """Map released normalized PX4 controls onto rotor speed targets."""

        self._px4_interface.update_actuator_commands(self._simulation_time_us, self._rotor_count)
        controls = self._px4_interface.read_applied_actuator_controls(self._rotor_count)
        if controls is None:
            return
        desired = np.clip(controls * self._params.max_rot_velocity, 0.0, self._params.max_rot_velocity)
        self._desired_rotor_angular_velocity = desired

    def _update_rotor_speed_state(self, dt_s: float) -> None:
        """Advance the first-order motor speed model by one MuJoCo step."""

        for i in range(self._rotor_count):
            diff = self._desired_rotor_angular_velocity[i] - self._rotor_angular_velocity[i]
            tc = self._params.time_constant_up if diff > 0 else self._params.time_constant_down
            self._rotor_angular_velocity[i] += diff * (1.0 - np.exp(-dt_s / tc))

    def _get_base_kinematics(self) -> Tuple[np.ndarray, Rotation, Rotation, np.ndarray, np.ndarray, np.ndarray]:
        """Read base pose and velocities in the simulator world frame."""

        base_pos = self._get_sensor_raw("pos")
        base_quat = self._get_sensor_raw("quat")
        rb = Rotation.from_quat(base_quat, scalar_first=True)
        rb_inv = rb.inv()
        thrust_axis_w = rb.apply(np.array([0.0, 0.0, 1.0]))
        v_com_w = self._get_sensor_raw("linvel")
        omega_r = self._get_sensor_raw("gyro")
        omega_w = rb.apply(omega_r)
        return base_pos, rb, rb_inv, thrust_axis_w, v_com_w, omega_w

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
        for i in range(self._rotor_count):
            r_off_w = rb.apply(self._rotor_offsets[i])
            pos_w = base_pos + r_off_w
            rotor_positions_w[i] = pos_w
            v_point_w = v_com_w + np.cross(omega_w, r_off_w)
            v_point_r = rb_inv.apply(v_point_w)
            v_parallel_r = np.array([0.0, 0.0, v_point_r[2]])
            v_perp_r = v_point_r - v_parallel_r

            omega = self._rotor_angular_velocity[i]
            omega_abs = abs(omega)
            direction = self._rotor_direction[i]

            # Match the Gazebo/PX4 rotor model that keeps thrust positive and
            # uses the spin direction only in the reaction torque term.
            thrust = self._params.motor_constant * omega * omega_abs
            thrust = abs(thrust)

            # The axial inflow term attenuates thrust when the rotor sees a
            # large relative velocity along its thrust axis.
            scalar = 1.0 - abs(v_parallel_r[2]) / max(self._params.max_relative_airspeed_mps, 1e-6)
            scalar = float(np.clip(scalar, 0.0, 1.0))
            thrust *= scalar

            rotor_thrusts[i] = thrust

            torque_z_r = -direction * thrust * self._params.moment_constant

            # Rotor drag and rolling moment depend on the airspeed component
            # orthogonal to the thrust axis.
            f_drag_r = -self._params.rotor_drag_coeff * omega_abs * v_perp_r
            m_rolling_r = -self._params.rolling_moment_coeff * omega_abs * direction * v_perp_r

            rotor_force_w[i] = rb.apply(np.array([0.0, 0.0, thrust]) + f_drag_r)
            rotor_moment_w[i] = rb.apply(np.array([0.0, 0.0, torque_z_r]) + m_rolling_r)

        return rotor_positions_w, rotor_thrusts, rotor_force_w, rotor_moment_w

    def _apply_rotor_wrenches(
        self,
        rotor_positions_w: np.ndarray,
        rotor_force_w: np.ndarray,
        rotor_moment_w: np.ndarray,
    ) -> None:
        """Accumulate rotor forces and torques onto the base body."""

        self._mj_data.xfrc_applied[:] = 0.0
        self._mj_data.qfrc_applied[:] = 0.0
        for i in range(self._rotor_count):
            mujoco.mj_applyFT(
                self._mj_model,
                self._mj_data,
                rotor_force_w[i],
                rotor_moment_w[i],
                rotor_positions_w[i],
                self._base_link_id,
                self._mj_data.qfrc_applied,
            )

    def _apply_motor_physics(self) -> None:
        """Update rotor state and apply the resulting aerodynamic wrench."""

        dt_s = self._mj_model.opt.timestep
        self._update_rotor_speed_state(dt_s)
        base_pos, rb, rb_inv, _, v_com_w, omega_w = self._get_base_kinematics()
        rotor_positions_w, _, rotor_force_w, rotor_moment_w = self._compute_rotor_wrenches(
            base_pos, rb, rb_inv, v_com_w, omega_w
        )
        self._apply_rotor_wrenches(rotor_positions_w, rotor_force_w, rotor_moment_w)

    def _update_rotor_visuals(self) -> None:
        """Spin mocap-only rotor visuals using the simulated motor speed."""

        base_pos = self._get_sensor_raw("pos")
        base_quat = self._get_sensor_raw("quat")
        rb = Rotation.from_quat(base_quat, scalar_first=True)
        armed = self._px4_interface.update_arming_state()
        for i in range(self._rotor_count):
            mocap_id = self._rotor_mocap_ids[i]
            if mocap_id < 0:
                continue
            visual_speed = self._rotor_angular_velocity[i]
            if armed and visual_speed < self._px4_sensor_params.idle_visual_speed:
                visual_speed = self._px4_sensor_params.idle_visual_speed
            self._rotor_angle[i] += visual_speed * self._rotor_direction[i] * self._mj_model.opt.timestep
            spin = Rotation.from_rotvec([0.0, 0.0, self._rotor_angle[i]])
            q_total = (rb * spin).as_quat(scalar_first=True)
            pos_w = base_pos + rb.apply(self._rotor_offsets[i])
            self._mj_data.mocap_pos[mocap_id] = pos_w
            self._mj_data.mocap_quat[mocap_id] = q_total

    def _update_custom_control(self) -> None:
        """Hook for subclasses that add extra actuation beyond the vehicle."""

    def _control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """MuJoCo control callback executed once per backend step."""

        self._step_count += 1
        self._advance_simulation_time_seconds(model.opt.timestep)
        if not self._px4_interface.is_connected:
            self._px4_interface.update_connection_state()
        else:
            self._sensor_bridge.update()
            self._update_px4_controls()
            self._apply_motor_physics()
        self._update_custom_control()
        self._update_rotor_visuals()

    def close(self) -> None:
        """Release PX4 resources and then delegate base-backend cleanup."""

        self._px4_interface.close()
        super().close()
