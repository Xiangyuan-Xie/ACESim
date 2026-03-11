from acetele.core.make_robot import make_robot

from acesim.config.config_loader import ConfigLoader
from acesim.env.genesis.multirotor_env import MultirotorEnv


class MCArmEnv(MultirotorEnv):
    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        self._robot_agent = make_robot()
        self._arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_gripper_left",
            "joint_gripper_right",
        ]
        self._arm_dofs_idx_local = None

    def _ensure_arm_dofs(self):
        if self._arm_dofs_idx_local is not None:
            return
        self._arm_dofs_idx_local = self._resolve_joint_dof_indices(self._arm_joint_names)

    def _update_arm_control(self):
        if self._step_count % 5 != 0:
            return
        if not self._arm_dofs_idx_local:
            return
        joint_pos, _, _ = self._robot_agent.act()
        count = min(len(self._arm_dofs_idx_local), len(joint_pos))
        if count <= 0:
            return
        self._control_dofs_position(joint_pos[:count], self._arm_dofs_idx_local[:count])

    def _update_custom_control(self):
        self._ensure_arm_dofs()
        self._update_arm_control()

    def close(self):
        self._robot_agent.close()
        super().close()
