"""Genesis multicopter environment with PX4 HIL integration."""

import platform
from dataclasses import dataclass
from typing import Any, Optional, Sequence

import genesis as gs
import numpy as np
from scipy.spatial.transform import Rotation

from acesim.config.config_loader import ConfigLoader
from acesim.env.genesis.genesis_env import GenesisEnv
from acesim.utils.frame import body_flu_to_frd
from acesim.utils.px4_sensor_scheduler import PX4SensorSample, PX4SensorScheduler
from acesim.utils.px4_transport import PX4ActuatorParams, PX4SensorParams, PX4Transport


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


class MCEnv(GenesisEnv):
    """Genesis multicopter backend with lazy runtime setup and PX4 HIL wiring."""

    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        asset_params = config_loader.get_asset_params()
        config = asset_params.get("mc", asset_params)
        self._params = MCParams(
            rotor_direction=np.array(config["rotor_direction"], dtype=float),
            motor_constant=float(config["motor_constant"]),
            moment_constant=float(config["moment_constant"]),
            rotor_drag_coeff=float(config["rotor_drag_coeff"]),
            rolling_moment_coeff=float(config["rolling_moment_coeff"]),
            rotor_radius=float(config["rotor_radius"]),
            time_constant_up=float(config.get("time_constant_up", 0.0125)),
            time_constant_down=float(config.get("time_constant_down", 0.025)),
            max_rot_velocity=float(config.get("max_rot_velocity", 1000.0)),
        )
        self._px4_sensor_params = PX4SensorParams.from_asset_params(
            asset_params,
            dynamic_hil_sensor_fields=True,
        )
        self._px4_actuator_params = PX4ActuatorParams()

        self._px4_transport = None
        self._sensor_scheduler = None
        self._runtime_initialized = False
        self._base_link = None
        self._rotor_links: list[Any] = []
        self._rotor_offsets = np.zeros((0, 3))
        self._rotor_count = 0
        self._desired_rotor_angular_velocity = np.zeros(0)
        self._rotor_angular_velocity = np.zeros(0)
        self._rotor_direction = np.zeros(0)
        self._arm_dofs_idx_local = None
        self._prev_base_linvel_w = np.zeros(3, dtype=float)
        self._has_prev_vel = False
        if platform.system() == "Windows":
            print("[ACESim] Genesis backend initialized on Windows.")
        else:
            print("[ACESim] Genesis backend initialized on Linux.")

    def _to_numpy(self, value, fallback_dim: Optional[int] = None):
        """Normalize Genesis getter outputs into flat NumPy arrays."""

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
        """Convert backend quaternions into SciPy rotations across API variants."""

        q = self._to_numpy(quat, fallback_dim=4).reshape(-1)
        if q.size != 4:
            return Rotation.identity()
        try:
            return Rotation.from_quat(q, scalar_first=True)
        except TypeError:
            return Rotation.from_quat([q[1], q[2], q[3], q[0]])
        except ValueError:
            return Rotation.identity()

    def _resolve_link(self, name: str):
        """Resolve one Genesis link by name, returning ``None`` if unavailable."""

        if self._robot is None:
            return None
        try:
            return self._robot.get_link(name)
        except Exception:
            return None

    def _resolve_joint_dof_indices(self, joint_names: Sequence[str]):
        """Resolve local DOF indices for the named Genesis joints."""

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
        """Drive Genesis DOFs through whichever control signature the backend exposes."""

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
        """Read one Genesis link position in the simulator world frame."""

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
        """Read one Genesis link quaternion, defaulting to identity on failure."""

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
        """Read one Genesis link linear velocity in the simulator world frame."""

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
        """Read one Genesis link angular velocity in the simulator world frame."""

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
        """Resolve Genesis links and derive rotor offsets once the scene exists."""

        self._base_link = self._resolve_link("base_link")
        if self._base_link is None:
            raise ValueError("Genesis robot must provide link 'base_link'")

        rotor_links = []
        for idx in range(1, 13):
            link = self._resolve_link(f"rotor_{idx}")
            if link is not None:
                rotor_links.append(link)

        self._rotor_links = rotor_links
        self._rotor_count = len(self._rotor_links)
        if self._rotor_count == 0:
            raise ValueError("No rotor links found. Expected links named rotor_<i>.")
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

    def _ensure_runtime_ready(self):
        """Create the scene and runtime-only handles on first use."""

        self._ensure_scene()
        if not self._runtime_initialized:
            self._initialize_runtime_handles()

    def _ensure_px4_transport(self):
        """Create PX4 transport objects after the runtime state becomes usable."""

        if self._px4_transport is None:
            self._px4_transport = PX4Transport(self._px4_actuator_params)
            self._sensor_scheduler = PX4SensorScheduler(
                self._px4_transport,
                self._sim_clock,
                self._px4_sensor_params,
                self.read_sensor_sample,
                self.reset_sensor_state,
            )

    def _get_base_kinematics(self):
        """Read base pose, orientation, linear velocity, and angular velocity in world NWU."""

        base_pos = self._get_link_pos(self._base_link)
        base_quat = self._get_link_quat(self._base_link)
        rb = self._quat_to_rotation(base_quat)
        v_com_w = self._get_link_vel(self._base_link)
        omega_w = self._get_link_ang(self._base_link)
        return base_pos, rb, v_com_w, omega_w

    def reset_sensor_state(self) -> None:
        """Clear the previous-velocity cache used for finite-difference acceleration."""

        self._prev_base_linvel_w = np.zeros(3, dtype=float)
        self._has_prev_vel = False

    def read_sensor_sample(self) -> PX4SensorSample:
        """Synthesize one HIL sensor sample from Genesis runtime kinematics.

        Genesis exposes base-link pose and velocity in the simulator world frame,
        which is NWU in this codebase. Linear acceleration is estimated from the
        previous velocity sample because the backend does not provide a direct
        body-frame accelerometer measurement.
        """

        _, rb, v_com_w, omega_w = self._get_base_kinematics()
        if self._has_prev_vel:
            accel_w = (v_com_w - self._prev_base_linvel_w) / max(self._dt_s, 1e-6)
        else:
            accel_w = np.zeros(3, dtype=float)
            self._has_prev_vel = True
        self._prev_base_linvel_w = v_com_w.copy()

        gravity_w = np.array([0.0, 0.0, -9.81], dtype=float)
        # HIL accelerometers report proper acceleration, so gravity is removed
        # before rotating the sample into the body frame.
        proper_accel_w = accel_w - gravity_w
        rb_inv = rb.inv()
        mag_field_w = np.array([0.21523, 0.0, -0.42741], dtype=float)
        return PX4SensorSample(
            accel_frd=body_flu_to_frd(rb_inv.apply(proper_accel_w)),
            gyro_frd=body_flu_to_frd(rb_inv.apply(omega_w)),
            mag_frd=body_flu_to_frd(rb_inv.apply(mag_field_w)),
            position_world_m=self._get_link_pos(self._base_link),
            velocity_world_mps=v_com_w,
            attitude_world_quat=self._get_link_quat(self._base_link),
        )

    def _update_px4_controls(self, sensor_sent: bool):
        """Map released normalized PX4 controls onto rotor speed targets."""

        if not sensor_sent:
            return
        px4 = self._px4_transport
        assert px4 is not None
        px4.update_actuator_commands(self._simulation_time_us, self._rotor_count)
        controls = px4.read_applied_actuator_controls(self._rotor_count)
        if controls is None:
            return
        desired = np.clip(controls * self._params.max_rot_velocity, 0.0, self._params.max_rot_velocity)
        self._desired_rotor_angular_velocity = desired

    def _update_rotor_speed_state(self):
        """Advance the first-order motor speed model by one Genesis step."""

        dt = self._dt_s
        for i in range(self._rotor_count):
            diff = self._desired_rotor_angular_velocity[i] - self._rotor_angular_velocity[i]
            tc = self._params.time_constant_up if diff > 0 else self._params.time_constant_down
            self._rotor_angular_velocity[i] += diff * (1.0 - np.exp(-dt / tc))

    def _get_link_index(self, link):
        """Return the best-effort Genesis link index used by force APIs."""

        idx_local = getattr(link, "idx_local", None)
        if idx_local is not None:
            return int(idx_local)
        idx = getattr(link, "idx", None)
        if idx is not None:
            return int(idx)
        return None

    def _apply_link_forces(self, links, forces_w: np.ndarray):
        """Apply world-frame forces using the vectorized API when available."""

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
        """Apply world-frame torques using the vectorized API when available."""

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

    def _compute_rotor_wrenches(
        self, base_pos: np.ndarray, rb: Rotation, rb_inv: Rotation, v_com_w: np.ndarray, omega_w: np.ndarray
    ):
        """Compute rotor forces and torques for the current Genesis state."""

        rotor_positions_w = np.zeros((self._rotor_count, 3), dtype=float)
        rotor_forces_w = np.zeros((self._rotor_count, 3), dtype=float)
        rotor_torques_w = np.zeros((self._rotor_count, 3), dtype=float)
        for i in range(self._rotor_count):
            r_off_w = rb.apply(self._rotor_offsets[i])
            pos_w = base_pos + r_off_w
            rotor_positions_w[i] = pos_w
            v_point_w = v_com_w + np.cross(omega_w, r_off_w)
            v_point_r = rb_inv.apply(v_point_w)
            v_planar_r = np.array([v_point_r[0], v_point_r[1], 0.0], dtype=float)
            omega = self._rotor_angular_velocity[i]
            direction = self._rotor_direction[i]

            # Genesis does not expose the same signed motor model path as the
            # MuJoCo backend, so the spin direction only enters the torque term.
            thrust = self._params.motor_constant * (omega**2)
            f_drag_r = -self._params.rotor_drag_coeff * omega * v_planar_r
            rotor_forces_w[i] = rb.apply(np.array([0.0, 0.0, thrust], dtype=float) + f_drag_r)
            torque_z_r = self._params.moment_constant * thrust * (-direction)
            m_rolling_r = -self._params.rolling_moment_coeff * omega * v_planar_r
            rotor_torques_w[i] = rb.apply(np.array([0.0, 0.0, torque_z_r], dtype=float) + m_rolling_r)
        return rotor_positions_w, rotor_forces_w, rotor_torques_w

    def _apply_motor_physics(self):
        """Update rotor state and apply the resulting aerodynamic wrench."""

        if self._rotor_count == 0:
            return
        self._update_rotor_speed_state()
        base_pos, rb, v_com_w, omega_w = self._get_base_kinematics()
        rb_inv = rb.inv()
        _, rotor_forces_w, rotor_torques_w = self._compute_rotor_wrenches(base_pos, rb, rb_inv, v_com_w, omega_w)
        self._apply_link_forces(self._rotor_links, rotor_forces_w)
        base_torque_w = np.sum(rotor_torques_w, axis=0).reshape(1, 3)
        if self._base_link is not None:
            self._apply_link_torques([self._base_link], base_torque_w)

    def _update_custom_control(self):
        """Hook for subclasses that add extra actuation beyond the vehicle."""

        return

    def step(self):
        """Advance the Genesis backend, PX4 transport, and vehicle physics by one step."""

        self._ensure_runtime_ready()
        self._ensure_px4_transport()
        self._step_count += 1
        self._advance_simulation_time_seconds(self._dt_s)
        if not self._px4_transport.is_connected:
            self._px4_transport.update_connection_state()
        else:
            sensor_sent = self._sensor_scheduler.update()
            self._update_px4_controls(sensor_sent)
            self._apply_motor_physics()
        self._update_custom_control()
        self._scene.step()

    def close(self):
        """Release PX4 resources and then delegate base-backend cleanup."""

        if self._px4_transport is not None:
            self._px4_transport.close()
        super().close()
