from __future__ import annotations

import json
import math
import os
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from importlib import import_module
from pathlib import Path
from typing import Any, Protocol, cast
from unittest.mock import patch

import mujoco

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.mj_env import MJEnv
from acesim.utils.math import calculate_coupled_gripper_positions


class _FakePX4Transport:
    created_count = 0

    def __init__(self, *args: object, **kwargs: object) -> None:
        type(self).created_count += 1
        self.is_connected = False

    def update_connection_state(self) -> bool:
        return False

    def update_actuator_commands(self, sim_time_us: int, channel_count: int) -> None:
        return None

    def read_applied_actuator_controls(self, channel_count: int):
        return None

    def update_arming_state(self) -> bool:
        return False

    def close(self) -> None:
        return None


class _FakeVisualPublisher:
    def __init__(self, params: object) -> None:
        self.is_enabled = False

    def publish(self, state: object) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeArmStatePublisher:
    def __init__(self, *args: object, **kwargs: object) -> None:
        return None

    def publish(
        self,
        timestamp_us: int,
        positions: object,
        velocities: object,
        efforts: object,
    ) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeClockPublisher:
    def __init__(self, *args: object, **kwargs: object) -> None:
        return None

    def publish(self, timestamp_us: int) -> None:
        return None

    def close(self) -> None:
        return None


class _FakeRobotAgent:
    last_instance: "_FakeRobotAgent | None" = None

    def __init__(self) -> None:
        type(self).last_instance = self
        self.positions: list[list[float]] = []

    def act(self) -> tuple[list[float], None, None]:
        return ([0.0] * 7, None, None)

    def set_position(self, positions: list[float]) -> None:
        self.positions.append(list(positions))

    def close(self) -> None:
        return None


class _MergeOnlyMJEnv(MJEnv):
    _config_loader: ConfigLoader

    def __init__(self, config_loader: ConfigLoader):
        self._config_loader = config_loader


class _SupportsHeadlessEnv(Protocol):
    _config_loader: ConfigLoader
    _rotor_count: int
    _step_count: int
    _arm_actuator_ids: list[int]
    _arm_joint_ids: list[int]
    _arm_params: Any
    _held_arm_pose: list[float]
    _mj_data: Any
    _mj_model: Any
    _robot: _FakeRobotAgent | None
    _sim_clock: Any

    def step(self) -> None: ...

    def close(self) -> None: ...

    def _current_arm_pose(self) -> list[float]: ...

    def _poll_arm_command_socket(self) -> None: ...

    def _read_arm_control_target(self) -> Any: ...


def _config_path(name: str) -> Path:
    return (Path(__file__).resolve().parents[1] / "acesim" / "config" / f"{name}.toml").resolve()


@patch("acesim.env.mujoco.am_env.make_robot", lambda: _FakeRobotAgent())
@patch("acesim.env.mujoco.am_env.ArmStatePublisher", _FakeArmStatePublisher)
@patch("acesim.env.mujoco.px4_mj_env.VehicleVisualStatePublisher", _FakeVisualPublisher)
@patch("acesim.env.mujoco.px4_mj_env.PX4Transport", _FakePX4Transport)
@patch("acesim.env.mujoco.mj_env.ClockPublisher", _FakeClockPublisher)
class MujocoHeadlessStartupTests(unittest.TestCase):
    def _instantiate_and_step(self, config_file: Path) -> _SupportsHeadlessEnv:
        loader = ConfigLoader(config_file)
        module_name, class_name = loader.get_sim_info()
        env_cls = getattr(import_module(module_name), class_name)
        env = env_cls(loader)
        try:
            for _ in range(3):
                env.step()
            return env
        except Exception:
            env.close()
            raise

    def _instantiate(self, config_file: Path) -> _SupportsHeadlessEnv:
        loader = ConfigLoader(config_file)
        module_name, class_name = loader.get_sim_info()
        env_cls = getattr(import_module(module_name), class_name)
        return env_cls(loader)

    def _send_arm_motion_command(
        self,
        env: _SupportsHeadlessEnv,
        endpoint: str,
        target_pose: list[float],
        *,
        duration_s: float,
        command_id: str = "test",
    ) -> dict[str, object]:
        import zmq

        client = zmq.Context.instance().socket(zmq.REQ)
        client.setsockopt(zmq.LINGER, 0)
        client.setsockopt(zmq.RCVTIMEO, 1000)
        client.connect(endpoint)
        try:
            client.send_json(
                {
                    "type": "move_joint_pose",
                    "command_id": command_id,
                    "pose": target_pose,
                    "duration_s": duration_s,
                }
            )
            env._poll_arm_command_socket()
            return client.recv_json()
        finally:
            client.close(linger=0)

    def _write_mc_config(self, root: Path, *, asset_name: str) -> Path:
        config_path = root / f"{asset_name}.toml"
        asset_src = Path(__file__).resolve().parents[1] / "acesim" / "config" / "mujoco" / f"{asset_name}.toml"
        asset_dst_dir = root / "mujoco"
        asset_dst_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(asset_src, asset_dst_dir / f"{asset_name}.toml")
        config_path.write_text(
            "\n".join(
                [
                    "[basic]",
                    'sim_type = "mujoco"',
                    'env_type = "mc"',
                    'scene_name = "default"',
                    f'asset_name = "{asset_name}"',
                    'benchmark = "multirotor"',
                    "",
                ]
            ),
            encoding="utf-8",
        )
        return config_path

    def test_default_config_starts_headless(self) -> None:
        expected_loader = ConfigLoader(_config_path("default"))
        env = self._instantiate_and_step(_config_path("default"))
        try:
            for _ in range(8):
                env.step()
            self.assertEqual(env._config_loader.get_asset_name(), expected_loader.get_asset_name())
            self.assertEqual(env._config_loader.get_env_type(), expected_loader.get_env_type())
            self.assertGreaterEqual(env._step_count, 1)
        finally:
            env.close()

    def test_all_mujoco_configs_start_headless(self) -> None:
        config_cases = [
            _config_path("default"),
            _config_path("advanced_plane"),
            _config_path("standard_vtol"),
            _config_path("uuv_bluerov2_heavy"),
        ]
        synthetic_assets = ["iris", "x500", "typhoon_h480"]

        with tempfile.TemporaryDirectory(prefix="acesim_mujoco_headless_") as tmpdir:
            temp_root = Path(tmpdir)
            for asset_name in synthetic_assets:
                config_cases.append(self._write_mc_config(temp_root, asset_name=asset_name))

            for config_file in config_cases:
                with self.subTest(config=config_file.name):
                    env = self._instantiate_and_step(config_file)
                    try:
                        self.assertGreater(env._step_count, 0)
                    finally:
                        env.close()

    def test_default_mc_env_handles_split_visual_rotor_offsets(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_mc_split_offsets_") as tmpdir:
            config_path = self._write_mc_config(Path(tmpdir), asset_name="x500")
            env = self._instantiate_and_step(config_path)
            try:
                self.assertEqual(env._rotor_count, 4)
            finally:
                env.close()

    def test_fixed_arm_pose_avoids_robot_agent_and_sets_mujoco_targets(self) -> None:
        fixed_arm_pose = [0.11, -0.22, 0.33, -0.44, 0.55]
        fixed_pose = fixed_arm_pose + list(calculate_coupled_gripper_positions(fixed_arm_pose[4]))

        def fail_make_robot() -> _FakeRobotAgent:
            raise AssertionError("make_robot should not be called in fixed-pose mode")

        with (
            patch.dict(os.environ, {"ACESIM_FIXED_ARM_POSE": json.dumps(fixed_pose)}),
            patch("acesim.env.mujoco.am_env.make_robot", fail_make_robot),
        ):
            env = self._instantiate(_config_path("default"))

        try:
            sample = env._read_arm_control_target()
            self.assertEqual(sample.joint_positions, fixed_pose)
            for expected, joint_id, actuator_id in zip(fixed_pose, env._arm_joint_ids, env._arm_actuator_ids):
                self.assertGreaterEqual(joint_id, 0)
                qpos_adr = env._mj_model.jnt_qposadr[joint_id]
                qvel_adr = env._mj_model.jnt_dofadr[joint_id]
                self.assertAlmostEqual(float(env._mj_data.qpos[qpos_adr]), expected)
                self.assertAlmostEqual(float(env._mj_data.qvel[qvel_adr]), 0.0)
                self.assertGreaterEqual(actuator_id, 0)
                self.assertAlmostEqual(float(env._mj_data.ctrl[actuator_id]), expected)
        finally:
            env.close()

    def test_fixed_arm_pose_accepts_five_joint_pose_and_couples_gripper(self) -> None:
        fixed_arm_pose = [0.11, 1.22, 0.33, -0.44, -0.55]
        expected_pose = fixed_arm_pose + list(calculate_coupled_gripper_positions(fixed_arm_pose[4]))

        with patch.dict(os.environ, {"ACESIM_FIXED_ARM_POSE": json.dumps(fixed_arm_pose)}):
            env = self._instantiate(_config_path("default"))

        try:
            sample = env._read_arm_control_target()
            self.assertEqual(sample.joint_positions, expected_pose)
            for expected, joint_id, actuator_id in zip(expected_pose, env._arm_joint_ids, env._arm_actuator_ids):
                qpos_adr = env._mj_model.jnt_qposadr[joint_id]
                self.assertAlmostEqual(float(env._mj_data.qpos[qpos_adr]), expected)
                self.assertAlmostEqual(float(env._mj_data.ctrl[actuator_id]), expected)
        finally:
            env.close()

    def test_fixed_arm_pose_keeps_actuator_targets_without_rewriting_joint_state_each_step(self) -> None:
        fixed_arm_pose = [0.11, 1.22, 0.33, -0.44, -0.55]
        fixed_pose = fixed_arm_pose + list(calculate_coupled_gripper_positions(fixed_arm_pose[4]))

        with patch.dict(os.environ, {"ACESIM_FIXED_ARM_POSE": json.dumps(fixed_pose)}):
            env = self._instantiate(_config_path("default"))

        try:
            for joint_id, actuator_id in zip(env._arm_joint_ids, env._arm_actuator_ids):
                env._mj_data.qpos[env._mj_model.jnt_qposadr[joint_id]] = 0.0
                env._mj_data.qvel[env._mj_model.jnt_dofadr[joint_id]] = 4.0
                env._mj_data.ctrl[actuator_id] = 0.0

            env.step()

            for expected, joint_id, actuator_id in zip(fixed_pose, env._arm_joint_ids, env._arm_actuator_ids):
                qpos_adr = env._mj_model.jnt_qposadr[joint_id]
                qvel_adr = env._mj_model.jnt_dofadr[joint_id]
                self.assertNotAlmostEqual(float(env._mj_data.qpos[qpos_adr]), expected, places=4)
                self.assertNotAlmostEqual(float(env._mj_data.qvel[qvel_adr]), 0.0, places=4)
                self.assertAlmostEqual(float(env._mj_data.ctrl[actuator_id]), expected)
        finally:
            env.close()

    def test_fixed_arm_pose_rejects_wrong_length_value(self) -> None:
        _FakePX4Transport.created_count = 0
        with patch.dict(os.environ, {"ACESIM_FIXED_ARM_POSE": json.dumps([0.0] * 6)}):
            with self.assertRaisesRegex(ValueError, "ACESIM_FIXED_ARM_POSE"):
                self._instantiate(_config_path("default"))
        self.assertEqual(_FakePX4Transport.created_count, 0)

    def test_fixed_arm_pose_rejects_non_finite_value(self) -> None:
        _FakePX4Transport.created_count = 0
        payload = json.dumps([0.0, 0.0, 0.0, math.inf, 0.0, 0.0, 0.0])
        with patch.dict(os.environ, {"ACESIM_FIXED_ARM_POSE": payload}):
            with self.assertRaisesRegex(ValueError, "finite"):
                self._instantiate(_config_path("default"))
        self.assertEqual(_FakePX4Transport.created_count, 0)

    def test_fixed_arm_pose_rejects_uncoupled_seven_joint_pose(self) -> None:
        _FakePX4Transport.created_count = 0
        payload = json.dumps([0.0, 1.2, 0.0, 0.0, -0.8, -0.001, 0.001])
        with patch.dict(os.environ, {"ACESIM_FIXED_ARM_POSE": payload}):
            with self.assertRaisesRegex(ValueError, "coupled gripper"):
                self._instantiate(_config_path("default"))
        self.assertEqual(_FakePX4Transport.created_count, 0)

    def test_robot_act_five_joint_pose_is_expanded_to_coupled_gripper(self) -> None:
        class FiveJointRobot(_FakeRobotAgent):
            def act(self) -> tuple[list[float], None, None]:
                return ([0.2, 1.1, -0.3, 0.4, -0.8], None, None)

        with patch("acesim.env.mujoco.am_env.make_robot", lambda: FiveJointRobot()):
            env = self._instantiate(_config_path("default"))

        try:
            expected = [0.2, 1.1, -0.3, 0.4, -0.8] + list(calculate_coupled_gripper_positions(-0.8))
            sample = env._read_arm_control_target()
            self.assertEqual(sample.joint_positions, expected)
        finally:
            env.close()

    def test_arm_command_only_avoids_robot_agent_and_holds_home(self) -> None:
        endpoint = "inproc://acesim_arm_command_only_test"

        def fail_make_robot() -> _FakeRobotAgent:
            raise AssertionError("make_robot should not be called in command-only mode")

        with (
            patch.dict(
                os.environ,
                {"ACESIM_ARM_COMMAND_ENDPOINT": endpoint, "ACESIM_ARM_COMMAND_ONLY": "1"},
                clear=False,
            ),
            patch("acesim.env.mujoco.am_env.make_robot", fail_make_robot),
        ):
            env = self._instantiate(_config_path("default"))

        try:
            self.assertIsNone(env._robot)
            expected_home = env._current_arm_pose()[:5]
            expected_home = expected_home + list(calculate_coupled_gripper_positions(expected_home[4]))
            sample = env._read_arm_control_target()
            self.assertEqual(list(sample.joint_positions), expected_home)
        finally:
            env.close()

    def test_arm_command_only_requires_command_endpoint(self) -> None:
        _FakePX4Transport.created_count = 0
        with patch.dict(os.environ, {"ACESIM_ARM_COMMAND_ONLY": "1"}, clear=True):
            with self.assertRaisesRegex(ValueError, "ACESIM_ARM_COMMAND_ONLY.*ACESIM_ARM_COMMAND_ENDPOINT"):
                self._instantiate(_config_path("default"))
        self.assertEqual(_FakePX4Transport.created_count, 0)

    def test_arm_command_interpolates_from_home_by_simulation_time(self) -> None:
        endpoint = "inproc://acesim_arm_motion_test"
        with patch.dict(os.environ, {"ACESIM_ARM_COMMAND_ENDPOINT": endpoint}, clear=False):
            env = self._instantiate(_config_path("default"))

        try:
            target_arm_pose = [0.2, 1.4, 0.1, -0.4, -0.6]
            target_pose = target_arm_pose + list(calculate_coupled_gripper_positions(target_arm_pose[4]))
            reply = self._send_arm_motion_command(env, endpoint, target_arm_pose, duration_s=5.0)
            self.assertTrue(reply["ok"])
            self.assertAlmostEqual(float(cast(Any, reply["duration_s"])), 5.0)

            start_pose = env._current_arm_pose()
            sample_start = env._read_arm_control_target()
            self.assertEqual(list(sample_start.joint_positions), start_pose)

            env._sim_clock.advance_seconds(2.5)
            sample_mid = env._read_arm_control_target()
            self.assertNotEqual(list(sample_mid.joint_positions), start_pose)
            self.assertNotEqual(list(sample_mid.joint_positions), target_pose)

            env._sim_clock.advance_seconds(2.6)
            sample_end = env._read_arm_control_target()
            self.assertEqual(list(sample_end.joint_positions), target_pose)
            self.assertEqual(env._held_arm_pose, target_pose)
        finally:
            env.close()

    def test_arm_command_rejects_uncoupled_seven_joint_pose(self) -> None:
        endpoint = "inproc://acesim_arm_motion_uncoupled_test"
        with patch.dict(os.environ, {"ACESIM_ARM_COMMAND_ENDPOINT": endpoint}, clear=False):
            env = self._instantiate(_config_path("default"))

        try:
            reply = self._send_arm_motion_command(
                env,
                endpoint,
                [0.2, 1.4, 0.1, -0.4, -0.6, -0.01, 0.01],
                duration_s=5.0,
            )
            self.assertFalse(reply["ok"])
            self.assertIn("coupled gripper", str(reply["error"]))
        finally:
            env.close()

    def test_arm_command_interpolates_gripper_from_joint5_coupling(self) -> None:
        endpoint = "inproc://acesim_arm_motion_gripper_coupling_test"
        with patch.dict(os.environ, {"ACESIM_ARM_COMMAND_ENDPOINT": endpoint}, clear=False):
            env = self._instantiate(_config_path("default"))

        try:
            target_arm_pose = [0.0, 2.0, 0.0, -0.4, -1.2]
            expected_target = target_arm_pose + list(calculate_coupled_gripper_positions(target_arm_pose[4]))
            reply = self._send_arm_motion_command(env, endpoint, target_arm_pose, duration_s=4.0)
            self.assertTrue(reply["ok"])

            env._sim_clock.advance_seconds(2.0)
            sample_mid = env._read_arm_control_target()
            expected_mid_gripper = calculate_coupled_gripper_positions(sample_mid.joint_positions[4])
            self.assertAlmostEqual(sample_mid.joint_positions[5], expected_mid_gripper[0])
            self.assertAlmostEqual(sample_mid.joint_positions[6], expected_mid_gripper[1])

            env._sim_clock.advance_seconds(2.1)
            sample_end = env._read_arm_control_target()
            self.assertEqual(list(sample_end.joint_positions), expected_target)
        finally:
            env.close()

    def test_arm_command_extends_short_duration_to_joint_limits(self) -> None:
        endpoint = "inproc://acesim_arm_motion_limit_test"
        with patch.dict(os.environ, {"ACESIM_ARM_COMMAND_ENDPOINT": endpoint}, clear=False):
            env = self._instantiate(_config_path("default"))

        try:
            start_pose = env._current_arm_pose()
            target_pose = list(start_pose[:5])
            target_pose[0] += 1.0

            reply = self._send_arm_motion_command(env, endpoint, target_pose, duration_s=0.1)

            self.assertTrue(reply["ok"])
            self.assertGreater(float(cast(Any, reply["duration_s"])), 1.8)
            env._sim_clock.advance_seconds(0.11)
            early_sample = env._read_arm_control_target()
            expected_target_pose = target_pose + list(calculate_coupled_gripper_positions(target_pose[4]))
            self.assertNotEqual(list(early_sample.joint_positions), expected_target_pose)

            env._sim_clock.advance_seconds(float(cast(Any, reply["duration_s"])))
            final_sample = env._read_arm_control_target()
            self.assertEqual(list(final_sample.joint_positions), expected_target_pose)
        finally:
            env.close()

    def test_arm_command_respects_velocity_limit(self) -> None:
        endpoint = "inproc://acesim_arm_motion_velocity_test"
        with patch.dict(os.environ, {"ACESIM_ARM_COMMAND_ENDPOINT": endpoint}, clear=False):
            env = self._instantiate(_config_path("default"))

        try:
            start_pose = env._current_arm_pose()
            target_pose = list(start_pose[:5])
            target_pose[0] += 1.0
            target_pose[1] -= 0.5
            target_pose[4] -= 0.5
            reply = self._send_arm_motion_command(env, endpoint, target_pose, duration_s=0.1)
            duration_s = float(cast(Any, reply["duration_s"]))
            dt = duration_s / 240.0

            positions: list[list[float]] = []
            for _ in range(241):
                sample = env._read_arm_control_target()
                positions.append(list(sample.joint_positions))
                env._sim_clock.advance_seconds(dt)

            velocity_limits = env._arm_params.arm_motion_max_velocity
            velocities = [
                [(positions[index + 1][joint] - positions[index][joint]) / dt for joint in range(len(positions[index]))]
                for index in range(len(positions) - 1)
            ]

            for joint, limit in enumerate(velocity_limits):
                self.assertLessEqual(max(abs(row[joint]) for row in velocities), limit * 1.01 + 1e-5)
            self.assertFalse(hasattr(env._arm_params, "arm_motion_max_acceleration"))
        finally:
            env.close()

    def test_arm_motion_limit_config_rejects_non_positive_values(self) -> None:
        with tempfile.TemporaryDirectory(prefix="acesim_arm_limit_config_") as tmpdir:
            root = Path(tmpdir)
            config_path = root / "default.toml"
            shutil.copy2(_config_path("default"), config_path)
            asset_dir = root / "mujoco"
            asset_dir.mkdir(parents=True, exist_ok=True)
            asset_src = Path(__file__).resolve().parents[1] / "acesim" / "config" / "mujoco" / "x500_arm2x.toml"
            asset_text = asset_src.read_text(encoding="utf-8")
            replacement = "arm_motion_max_velocity = [1.0, 1.0, 0.0, 1.0, 1.0, 0.02, 0.02]"
            if "arm_motion_max_velocity" in asset_text:
                lines = [
                    replacement if line.strip().startswith("arm_motion_max_velocity") else line
                    for line in asset_text.splitlines()
                ]
                asset_text = "\n".join(lines) + "\n"
            else:
                asset_text = asset_text + "\n" + replacement + "\n"
            (asset_dir / "x500_arm2x.toml").write_text(asset_text, encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "arm_motion_max_velocity"):
                self._instantiate(config_path)

    def test_am_env_syncs_robot_to_home_keyframe_when_not_fixed_pose(self) -> None:
        _FakeRobotAgent.last_instance = None
        with patch.dict(os.environ, {}, clear=False):
            env = self._instantiate(_config_path("default"))

        try:
            self.assertIsNotNone(_FakeRobotAgent.last_instance)
            robot = _FakeRobotAgent.last_instance
            self.assertIsNotNone(robot)
            assert robot is not None
            self.assertTrue(robot.positions)
            self.assertEqual(len(robot.positions[-1]), 5)
        finally:
            env.close()

    def test_default_x500_arm_scene_injects_shared_landing_pad_geom_and_home_height(self) -> None:
        loader = ConfigLoader(_config_path("default"))
        from acesim.env.mujoco.mj_env import MJEnv

        scene_path = Path(__file__).resolve().parents[1] / "acesim" / "env" / "mujoco" / "scene" / "default.xml"
        asset_path = (
            Path(__file__).resolve().parents[1]
            / "acesim"
            / "env"
            / "mujoco"
            / "asset"
            / "x500_arm2x"
            / "x500_arm2x.xml"
        )
        merge_env = _MergeOnlyMJEnv(loader)
        merged_xml = MJEnv._merge_scene_robot_xml(merge_env, scene_path, asset_path)
        root = ET.fromstring(merged_xml)
        landing_pad = root.find("./worldbody/geom[@name='acesim_landing_pad']")
        self.assertIsNotNone(landing_pad)
        assert landing_pad is not None
        self.assertEqual(landing_pad.get("type"), "cylinder")
        self.assertEqual(landing_pad.get("size"), "3.5 0.02")
        self.assertEqual(landing_pad.get("pos"), "0 0 0.02")
        self.assertEqual(landing_pad.get("contype"), "1")
        self.assertEqual(landing_pad.get("conaffinity"), "1")

        scene_home = root.find("./keyframe/key[@name='scene_home']")
        self.assertIsNotNone(scene_home)
        assert scene_home is not None
        qpos = [float(value) for value in scene_home.get("qpos", "").split()]
        robot_home = ET.parse(asset_path).getroot().find("./keyframe/key[@name='home']")
        self.assertIsNotNone(robot_home)
        assert robot_home is not None
        robot_qpos = [float(value) for value in robot_home.get("qpos", "").split()]
        self.assertAlmostEqual(qpos[2], robot_qpos[2] + 0.048)

        model = mujoco.MjModel.from_xml_string(merged_xml)
        data = mujoco.MjData(model)
        scene_home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "scene_home")
        mujoco.mj_resetDataKeyframe(model, data, scene_home_id)
        mujoco.mj_forward(model, data)
        physical_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rotor_1")
        visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rotor_1_vis")
        self.assertGreaterEqual(physical_id, 0)
        self.assertGreaterEqual(visual_id, 0)
        for axis in range(3):
            self.assertAlmostEqual(float(data.xpos[visual_id][axis]), float(data.xpos[physical_id][axis]))

    def test_default_x500_arm_env_resets_to_scene_home_keyframe(self) -> None:
        loader = ConfigLoader(_config_path("default"))
        module_name, class_name = loader.get_sim_info()
        env_cls = getattr(import_module(module_name), class_name)
        env = env_cls(loader)
        try:
            asset_path = (
                Path(__file__).resolve().parents[1]
                / "acesim"
                / "env"
                / "mujoco"
                / "asset"
                / "x500_arm2x"
                / "x500_arm2x.xml"
            )
            robot_home = ET.parse(asset_path).getroot().find("./keyframe/key[@name='home']")
            self.assertIsNotNone(robot_home)
            assert robot_home is not None
            robot_qpos = [float(value) for value in robot_home.get("qpos", "").split()]
            self.assertAlmostEqual(float(env._mj_data.qpos[2]), robot_qpos[2] + 0.048)
            self.assertAlmostEqual(float(env._mj_data.qpos[3]), 1.0)
            self.assertAlmostEqual(float(env._mj_data.qpos[4]), 0.0)
            self.assertAlmostEqual(float(env._mj_data.qpos[5]), 0.0)
            self.assertAlmostEqual(float(env._mj_data.qpos[6]), 0.0)
        finally:
            env.close()

    def test_mj_env_initial_reset_prefers_scene_home_before_keyframe_zero(self) -> None:
        from acesim.env.mujoco.mj_env import MJEnv

        model = mujoco.MjModel.from_xml_string("""
            <mujoco>
              <worldbody>
                <body name="body" pos="0 0 0">
                  <freejoint/>
                  <geom type="box" size="0.01 0.01 0.01"/>
                </body>
              </worldbody>
              <keyframe>
                <key name="home" qpos="0 0 0.25 1 0 0 0"/>
                <key name="scene_home" qpos="0 0 1.25 1 0 0 0"/>
              </keyframe>
            </mujoco>
            """)
        data = mujoco.MjData(model)

        key_id = MJEnv._initial_keyframe_id_for_model(model)
        assert key_id == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "scene_home")
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        self.assertAlmostEqual(float(data.qpos[2]), 1.25)
