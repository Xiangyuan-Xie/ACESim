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

    def _reset_to_home(self):
        key_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, key_id)
            home_pose = []
            for i, act_id in enumerate(self._arm_actuator_ids):
                j_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, self._arm_joint_names[i])
                if j_id >= 0 and act_id >= 0:
                    self._mj_data.ctrl[act_id] = self._mj_data.qpos[self._mj_model.jnt_qposadr[j_id]]
                    home_pose.append(self._mj_data.ctrl[act_id])
            print(f"Loading 'home' keyframe: [{', '.join([f'{v:.3f}' for v in home_pose])}]")
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
