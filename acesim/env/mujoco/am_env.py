"""MuJoCo aerial manipulator environment with the manipulator control stack."""

import json
import math
import os
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import mujoco
import zmq
from acetele.core.make_robot import make_robot

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.mc_env import MCEnv
from acesim.utils.arm_servo_scheduler import ArmControlSample, ArmServoScheduler, ArmStateSample
from acesim.utils.arm_state_publisher import ArmStatePublisher
from acesim.utils.delay import parse_delay_range_ms
from acesim.utils.math import calculate_coupled_gripper_positions
from acesim.utils.sim_streams import ArmCommandStreamParams, ArmCommandStreamSubscriber


@dataclass
class AMParams:
    """Timing parameters that affect the manipulator control loop."""

    arm_control_rate_hz: float
    arm_state_publish_rate_hz: float
    arm_motion_max_velocity: list[float]
    joint_state_delay_ms: tuple[float, float]


@dataclass
class _ArmMotionCommand:
    """One simulation-time joint-space arm motion command."""

    command_id: str
    start_time_us: int
    duration_s: float
    start_pose: list[float]
    target_pose: list[float]
    completed: bool = False


_DEFAULT_ARM_MOTION_MAX_VELOCITY = [1.0] * 7
_QUINTIC_MAX_VELOCITY_SCALE = 1.875


class AMEnv(MCEnv):
    """MuJoCo aerial manipulator environment with an attached arm control agent."""

    def __init__(self, config_loader: ConfigLoader):
        self._arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
        ]
        self._arm_actuated_joint_names = [
            *self._arm_joint_names,
            "joint_gripper_left",
            "joint_gripper_right",
        ]
        self._arm_command_only = self._parse_arm_command_only()
        self._fixed_arm_pose = self._parse_fixed_arm_pose()
        self._arm_command_socket: zmq.Socket | None = None
        self._arm_command_stream_subscriber: ArmCommandStreamSubscriber | None = None
        self._active_arm_motion: _ArmMotionCommand | None = None
        self._held_arm_pose: list[float] | None = None
        super().__init__(config_loader)
        asset_params = config_loader.get_asset_params()
        config = asset_params
        self._arm_command_stream_params = ArmCommandStreamParams.from_asset_params(asset_params)
        if self._arm_command_stream_params.enabled and os.environ.get("ACESIM_ARM_COMMAND_ENDPOINT"):
            raise ValueError("arm_command_stream cannot be enabled with ACESIM_ARM_COMMAND_ENDPOINT")
        arm_config = config.get("arm", {})
        if not isinstance(arm_config, Mapping):
            raise ValueError("arm must be a table")
        arm_delay_config = arm_config.get("delay", {})
        if not isinstance(arm_delay_config, Mapping):
            raise ValueError("arm.delay must be a table")
        self._arm_params = AMParams(
            arm_control_rate_hz=float(config.get("arm_control_rate_hz", 50.0)),
            arm_state_publish_rate_hz=float(config.get("arm_state_publish_rate_hz", 250.0)),
            arm_motion_max_velocity=self._parse_arm_motion_limit(
                config.get("arm_motion_max_velocity", _DEFAULT_ARM_MOTION_MAX_VELOCITY),
                name="arm_motion_max_velocity",
            ),
            joint_state_delay_ms=parse_delay_range_ms(
                arm_delay_config.get("joint_state_delay_ms", (0.0, 0.0)),
                "joint_state_delay_ms",
            ),
        )
        self._robot = (
            None
            if self._fixed_arm_pose is not None or self._arm_command_only or self._arm_command_stream_params.enabled
            else make_robot()
        )
        self._arm_actuator_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self._arm_actuated_joint_names
        ]
        self._arm_joint_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self._arm_actuated_joint_names
        ]
        self._arm_state_publisher = ArmStatePublisher()
        self._initialize_arm_command_socket()
        if self._arm_command_stream_params.enabled:
            self._arm_command_stream_subscriber = ArmCommandStreamSubscriber(self._arm_command_stream_params)
        self._arm_servo_scheduler = ArmServoScheduler(
            clock=self._sim_clock,
            publisher=self._arm_state_publisher,
            control_rate_hz=self._arm_params.arm_control_rate_hz,
            state_publish_rate_hz=self._arm_params.arm_state_publish_rate_hz,
            read_control_target=self._read_arm_control_target,
            apply_control=self._apply_arm_control,
            read_state=self._read_arm_joint_state,
            joint_state_delay_ms=self._arm_params.joint_state_delay_ms,
        )
        self._reset_to_home()
        if self._fixed_arm_pose is not None:
            self._apply_fixed_arm_pose()
        elif self._arm_command_only:
            self._held_arm_pose = self._expand_arm_pose_with_coupled_gripper(
                self._current_arm_pose()[:5],
                field_name="current arm pose",
            )
        elif self._arm_command_stream_subscriber is not None:
            self._held_arm_pose = self._expand_arm_pose_with_coupled_gripper(
                self._current_arm_pose()[:5],
                field_name="current arm pose",
            )
        elif self._robot is not None:
            self._sync_robot_to_current_arm_pose()
        self._arm_servo_scheduler.reset()

    def _parse_arm_command_only(self) -> bool:
        """Return whether this env is driven only by benchmark arm commands."""

        raw_value = os.environ.get("ACESIM_ARM_COMMAND_ONLY")
        if raw_value is None:
            return False

        value = raw_value.strip().lower()
        if value in ("", "0", "false", "no", "off"):
            return False
        if value in ("1", "true", "yes", "on"):
            if not os.environ.get("ACESIM_ARM_COMMAND_ENDPOINT"):
                raise ValueError("ACESIM_ARM_COMMAND_ONLY requires ACESIM_ARM_COMMAND_ENDPOINT")
            return True
        raise ValueError("ACESIM_ARM_COMMAND_ONLY must be a boolean value")

    def _parse_fixed_arm_pose(self) -> list[float] | None:
        """Parse the static benchmark arm target, if requested."""

        raw_pose = os.environ.get("ACESIM_FIXED_ARM_POSE")
        if raw_pose is None:
            return None

        try:
            pose = json.loads(raw_pose)
        except json.JSONDecodeError as exc:
            raise ValueError("ACESIM_FIXED_ARM_POSE must be a JSON list of exactly 7 finite floats") from exc

        return self._parse_arm_pose_payload(pose, field_name="ACESIM_FIXED_ARM_POSE")

    def _parse_arm_motion_limit(self, raw_value: object, *, name: str) -> list[float]:
        """Parse per-joint positive limits in command units/s or command units/s^2."""

        joint_count = len(self._arm_actuated_joint_names)
        if isinstance(raw_value, bool):
            raise ValueError(f"{name} must be a positive finite float or a list of {joint_count} values")
        if isinstance(raw_value, (int, float)):
            values = [float(raw_value)] * joint_count
        elif isinstance(raw_value, list) and len(raw_value) == joint_count:
            values = []
            for index, value in enumerate(raw_value):
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"{name} value at index {index} must be a positive finite float")
                values.append(float(value))
        else:
            raise ValueError(f"{name} must be a positive finite float or a list of {joint_count} values")

        for index, value in enumerate(values):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} value at index {index} must be a positive finite float")
        return values

    def _initialize_arm_command_socket(self) -> None:
        """Enable the optional benchmark arm-command REP socket."""

        endpoint = os.environ.get("ACESIM_ARM_COMMAND_ENDPOINT")
        if not endpoint:
            return

        socket = zmq.Context.instance().socket(zmq.REP)
        socket.setsockopt(zmq.LINGER, 0)
        socket.bind(endpoint)
        self._arm_command_socket = socket

    def _poll_arm_command_socket(self) -> None:
        """Accept benchmark arm commands without blocking the MuJoCo callback."""

        if self._arm_command_socket is None:
            return

        while True:
            try:
                payload = self._arm_command_socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                return

            try:
                command = self._parse_arm_motion_command(payload)
            except Exception as exc:
                self._arm_command_socket.send_json({"ok": False, "error": str(exc)})
                continue

            self._active_arm_motion = command
            self._held_arm_pose = None
            self._arm_command_socket.send_json(
                {
                    "ok": True,
                    "command_id": command.command_id,
                    "accepted_timestamp_us": command.start_time_us,
                    "duration_s": command.duration_s,
                }
            )

    def _parse_arm_motion_command(self, payload: object) -> _ArmMotionCommand:
        if not isinstance(payload, dict):
            raise ValueError("arm command must be a JSON object")
        if payload.get("type") != "move_joint_pose":
            raise ValueError("arm command type must be 'move_joint_pose'")

        target_pose = self._parse_arm_pose_payload(payload.get("pose"), field_name="pose")
        duration_s = float(payload.get("duration_s", 0.0))
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError("duration_s must be a positive finite float")
        command_id = str(payload.get("command_id", "arm_motion"))
        start_pose = self._expand_arm_pose_with_coupled_gripper(
            self._current_arm_pose()[:5], field_name="current arm pose"
        )
        return _ArmMotionCommand(
            command_id=command_id,
            start_time_us=self._sim_clock.current_time_us,
            duration_s=self._resolve_arm_motion_duration(duration_s, start_pose, target_pose),
            start_pose=start_pose,
            target_pose=target_pose,
        )

    def _parse_arm_pose_payload(self, payload: object, *, field_name: str) -> list[float]:
        if not isinstance(payload, list) or len(payload) not in (
            len(self._arm_joint_names),
            len(self._arm_actuated_joint_names),
        ):
            raise ValueError(
                f"{field_name} must be a JSON list of exactly {len(self._arm_joint_names)} or "
                f"{len(self._arm_actuated_joint_names)} finite floats"
            )

        pose: list[float] = []
        for index, value in enumerate(payload):
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field_name} value at index {index} must be a finite float")
            target = float(value)
            if not math.isfinite(target):
                raise ValueError(f"{field_name} value at index {index} must be finite")
            pose.append(target)

        return self._expand_arm_pose_with_coupled_gripper(pose, field_name=field_name)

    def _expand_arm_pose_with_coupled_gripper(self, pose: list[float], *, field_name: str) -> list[float]:
        if len(pose) == len(self._arm_joint_names):
            return pose + list(calculate_coupled_gripper_positions(pose[4]))

        expected_left, expected_right = calculate_coupled_gripper_positions(pose[4])
        if not (
            math.isclose(pose[5], expected_left, abs_tol=1e-6) and math.isclose(pose[6], expected_right, abs_tol=1e-6)
        ):
            raise ValueError(
                f"{field_name} has uncoupled gripper values; expected coupled gripper "
                f"[{expected_left:.6g}, {expected_right:.6g}] from joint_5={pose[4]:.6g}"
            )
        return pose[:5] + [expected_left, expected_right]

    def _resolve_arm_motion_duration(
        self,
        requested_duration_s: float,
        start_pose: list[float],
        target_pose: list[float],
    ) -> float:
        duration_s = requested_duration_s
        for index, (start, target) in enumerate(zip(start_pose, target_pose)):
            delta = abs(target - start)
            if delta <= 0.0:
                continue
            max_velocity = self._arm_params.arm_motion_max_velocity[index]
            duration_s = max(duration_s, _QUINTIC_MAX_VELOCITY_SCALE * delta / max_velocity)
        return duration_s

    def _quintic_time_scaling(self, alpha: float) -> float:
        alpha = min(1.0, max(0.0, alpha))
        return alpha * alpha * alpha * (10.0 + alpha * (-15.0 + 6.0 * alpha))

    def _current_arm_pose(self) -> list[float]:
        pose: list[float] = []
        for joint_id in self._arm_joint_ids:
            if joint_id < 0:
                pose.append(0.0)
                continue
            pose.append(float(self._mj_data.qpos[self._mj_model.jnt_qposadr[joint_id]]))
        return pose

    def _sync_robot_to_current_arm_pose(self) -> None:
        """Align ACETele's internal command state with the MuJoCo home keyframe."""

        if self._robot is None or not hasattr(self._robot, "set_position"):
            return
        self._robot.set_position(self._current_arm_pose()[:5])

    def _sample_active_arm_motion(self) -> list[float] | None:
        command = self._active_arm_motion
        if command is None:
            return self._held_arm_pose

        elapsed_s = max(0.0, (self._sim_clock.current_time_us - command.start_time_us) * 1e-6)
        alpha = self._quintic_time_scaling(elapsed_s / command.duration_s)
        arm_pose = [
            start + (target - start) * alpha for start, target in zip(command.start_pose[:5], command.target_pose[:5])
        ]
        pose = self._expand_arm_pose_with_coupled_gripper(arm_pose, field_name="interpolated arm pose")
        if elapsed_s >= command.duration_s:
            command.completed = True
            self._held_arm_pose = list(command.target_pose)
            self._active_arm_motion = None
            return self._held_arm_pose
        return pose

    def _apply_fixed_arm_pose(self) -> None:
        """Initialize arm qpos, qvel, and actuator targets to the benchmark pose."""

        if self._fixed_arm_pose is None:
            return

        for i, target in enumerate(self._fixed_arm_pose):
            joint_id = self._arm_joint_ids[i]
            if joint_id >= 0:
                qpos_adr = self._mj_model.jnt_qposadr[joint_id]
                qvel_adr = self._mj_model.jnt_dofadr[joint_id]
                self._mj_data.qpos[qpos_adr] = target
                self._mj_data.qvel[qvel_adr] = 0.0

            act_id = self._arm_actuator_ids[i]
            if act_id >= 0:
                self._mj_data.ctrl[act_id] = target

        mujoco.mj_forward(self._mj_model, self._mj_data)

    def _apply_fixed_arm_actuator_targets(self) -> None:
        """Keep fixed-pose benchmark arms under actuator control without teleporting joints."""

        if self._fixed_arm_pose is None:
            return

        for i, target in enumerate(self._fixed_arm_pose):
            act_id = self._arm_actuator_ids[i]
            if act_id >= 0:
                self._mj_data.ctrl[act_id] = target

    def _resolve_home_keyframe(self):
        """Return the preferred home keyframe, favoring scene-specific overrides."""

        for name in ("scene_home", "home"):
            key_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_KEY, name)
            if key_id >= 0:
                return key_id, name
        return -1, ""

    def _load_asset_home_qpos(self):
        """Read the asset-level home qpos used when the scene keyframe is not sufficient."""

        asset_name = self._config_loader.get_asset_name()
        asset_xml = (Path(__file__).parent / "asset" / asset_name / f"{asset_name}.xml").resolve()
        root = ET.parse(asset_xml).getroot()
        keyframe = root.find("keyframe")
        if keyframe is None:
            return []
        for key in keyframe.findall("key"):
            if key.get("name") == "home":
                qpos_str = key.get("qpos", "")
                return [float(v) for v in qpos_str.split()] if qpos_str else []
        return []

    def _reset_with_home_pose(self, key_id: int):
        """Reset to the scene and then reapply the robot-only home pose if needed."""

        mujoco.mj_resetData(self._mj_model, self._mj_data)
        base_joint_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
        if base_joint_id < 0:
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)
            return
        base_qpos_adr = self._mj_model.jnt_qposadr[base_joint_id]
        robot_qpos_len = self._mj_model.nq - base_qpos_adr
        key_qpos = self._load_asset_home_qpos()
        if key_qpos:
            copy_len = min(robot_qpos_len, len(key_qpos))
            self._mj_data.qpos[base_qpos_adr : base_qpos_adr + copy_len] = key_qpos[:copy_len]
        self._mj_data.qvel[:] = 0.0
        mujoco.mj_forward(self._mj_model, self._mj_data)

    def _sync_arm_ctrl_to_current_qpos(self):
        """Initialize arm actuator targets from the current joint configuration."""

        home_pose = []
        for i, act_id in enumerate(self._arm_actuator_ids):
            j_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, self._arm_actuated_joint_names[i])
            if j_id >= 0 and act_id >= 0:
                self._mj_data.ctrl[act_id] = self._mj_data.qpos[self._mj_model.jnt_qposadr[j_id]]
                home_pose.append(self._mj_data.ctrl[act_id])
        return home_pose

    def _reset_to_home(self):
        """Reset the environment and synchronize the arm controller's targets."""

        key_id, key_name = self._resolve_home_keyframe()
        if key_id < 0:
            print("No 'home' keyframe found. Using default.")
            return

        if key_name == "home":
            self._reset_with_home_pose(key_id)
        else:
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)

        home_pose = self._sync_arm_ctrl_to_current_qpos()
        print(f"Loading '{key_name}' keyframe: [{', '.join([f'{v:.3f}' for v in home_pose])}]")

    def _read_arm_control_target(self) -> ArmControlSample | None:
        """Read the next arm control target from the robot controller."""

        if self._fixed_arm_pose is not None:
            return ArmControlSample(joint_positions=self._fixed_arm_pose)

        if self._arm_command_stream_subscriber is not None:
            command = self._arm_command_stream_subscriber.read_latest()
            if command is not None:
                self._held_arm_pose = list(command["positions"])
            if self._held_arm_pose is not None:
                return ArmControlSample(joint_positions=self._held_arm_pose)
            return None

        benchmark_pose = self._sample_active_arm_motion()
        if benchmark_pose is not None:
            return ArmControlSample(joint_positions=benchmark_pose)

        if self._robot is None:
            raise RuntimeError("AMEnv has no arm controller; send an arm command or disable ACESIM_ARM_COMMAND_ONLY")

        joint_pos, _, _ = self._robot.act()
        return ArmControlSample(
            joint_positions=self._expand_arm_pose_with_coupled_gripper(list(joint_pos), field_name="robot.act()")
        )

    def _apply_arm_control(self, control_sample: ArmControlSample) -> None:
        """Apply one scheduled arm control sample to MuJoCo actuators."""

        for i, act_id in enumerate(self._arm_actuator_ids):
            if act_id >= 0 and i < len(control_sample.joint_positions):
                self._mj_data.ctrl[act_id] = control_sample.joint_positions[i]

    def _read_arm_joint_state(self) -> ArmStateSample:
        """Return the current MuJoCo arm state for the exported arm and gripper joints."""

        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []
        for joint_id in self._arm_joint_ids:
            if joint_id < 0:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
                continue

            qpos_adr = self._mj_model.jnt_qposadr[joint_id]
            qvel_adr = self._mj_model.jnt_dofadr[joint_id]
            positions.append(float(self._mj_data.qpos[qpos_adr]))
            velocities.append(float(self._mj_data.qvel[qvel_adr]))
            efforts.append(float(self._mj_data.qfrc_actuator[qvel_adr]))

        return ArmStateSample(positions=positions, velocities=velocities, efforts=efforts)

    def _update_custom_control(self):
        """Extend the multicopter control hook with manipulator actuation."""

        self._poll_arm_command_socket()
        self._arm_servo_scheduler.update()

    def step(self):
        """Advance one step while keeping benchmark fixed-pose arm targets applied."""

        if self._fixed_arm_pose is not None:
            self._apply_fixed_arm_actuator_targets()
        super().step()

    def close(self):
        """Release the arm agent before delegating backend cleanup."""

        if self._robot is not None:
            self._robot.close()
        if self._arm_command_socket is not None:
            self._arm_command_socket.close(linger=0)
            self._arm_command_socket = None
        if self._arm_command_stream_subscriber is not None:
            self._arm_command_stream_subscriber.close()
            self._arm_command_stream_subscriber = None
        self._arm_state_publisher.close()
        super().close()
