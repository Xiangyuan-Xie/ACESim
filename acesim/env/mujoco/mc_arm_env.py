import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
from acetele.core.make_robot import make_robot

from acesim.config.config_loader import ConfigLoader
from acesim.env.mujoco.multirotor_env import MultirotorEnv


class MCArmEnv(MultirotorEnv):
    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        self._robot = make_robot()
        self._initialize_arm_handles()
        self._reset_to_home()

    def _initialize_arm_handles(self):
        self._arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_gripper_left",
            "joint_gripper_right",
        ]
        self._arm_actuator_ids = [
            mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self._arm_joint_names
        ]

    def _resolve_home_keyframe(self):
        for name in ("scene_home", "home"):
            key_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_KEY, name)
            if key_id >= 0:
                return key_id, name
        return -1, ""

    def _load_asset_home_qpos(self):
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

    def _reset_to_home(self):
        key_id, key_name = self._resolve_home_keyframe()
        if key_id >= 0:
            if key_name == "home":
                mujoco.mj_resetData(self._mj_model, self._mj_data)
                base_joint_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
                if base_joint_id >= 0:
                    base_qpos_adr = self._mj_model.jnt_qposadr[base_joint_id]
                    robot_qpos_len = self._mj_model.nq - base_qpos_adr
                    key_qpos = self._load_asset_home_qpos()
                    if key_qpos:
                        copy_len = min(robot_qpos_len, len(key_qpos))
                        self._mj_data.qpos[base_qpos_adr : base_qpos_adr + copy_len] = key_qpos[:copy_len]
                    self._mj_data.qvel[:] = 0.0
                    mujoco.mj_forward(self._mj_model, self._mj_data)
                else:
                    mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)
            else:
                mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)
            home_pose = []
            for i, act_id in enumerate(self._arm_actuator_ids):
                j_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, self._arm_joint_names[i])
                if j_id >= 0 and act_id >= 0:
                    self._mj_data.ctrl[act_id] = self._mj_data.qpos[self._mj_model.jnt_qposadr[j_id]]
                    home_pose.append(self._mj_data.ctrl[act_id])
            print(f"Loading '{key_name}' keyframe: [{', '.join([f'{v:.3f}' for v in home_pose])}]")
        else:
            print("No 'home' keyframe found. Using default.")

    def _update_arm_control(self):
        if self._step_count % 5 == 0:
            joint_pos, _, _ = self._robot.act()
            for i, act_id in enumerate(self._arm_actuator_ids):
                if act_id >= 0 and i < len(joint_pos):
                    self._mj_data.ctrl[act_id] = joint_pos[i]

    def _update_custom_control(self):
        self._update_arm_control()  # arm: 50Hz

    def close(self):
        self._robot.close()
