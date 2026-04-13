from __future__ import annotations

import unittest
from pathlib import Path


class Px4ArmSplitStaticTests(unittest.TestCase):
    def setUp(self) -> None:
        self.repo_root = Path(__file__).resolve().parents[1]

    def test_px4_msgs_defines_split_arm_messages(self) -> None:
        px4_msgs_dir = self.repo_root / "acesim" / "deploy" / "aircraft" / "px4_msgs" / "msg"
        self.assertTrue((px4_msgs_dir / "ArmJointCommand.msg").exists())
        self.assertTrue((px4_msgs_dir / "ArmJointState.msg").exists())

    def test_px4_uorb_defines_split_arm_messages(self) -> None:
        uorb_msg_dir = self.repo_root / "acesim" / "third_party" / "aircraft" / "PX4-Autopilot" / "msg"
        self.assertTrue((uorb_msg_dir / "ArmJointCommand.msg").exists())
        self.assertTrue((uorb_msg_dir / "ArmJointState.msg").exists())

    def test_dds_topics_expose_split_arm_inputs(self) -> None:
        dds_topics = (
            self.repo_root
            / "acesim"
            / "third_party"
            / "aircraft"
            / "PX4-Autopilot"
            / "src"
            / "modules"
            / "uxrce_dds_client"
            / "dds_topics.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("/fmu/in/arm_joint_command", dds_topics)
        self.assertIn("/fmu/in/arm_joint_state", dds_topics)
        self.assertNotIn("/fmu/in/arm_joint_status", dds_topics)

    def test_rl_mc_arm_control_uses_split_arm_topics(self) -> None:
        header = (
            self.repo_root
            / "acesim"
            / "third_party"
            / "aircraft"
            / "PX4-Autopilot"
            / "src"
            / "modules"
            / "rl_mc_arm_control"
            / "rl_mc_arm_control.hpp"
        ).read_text(encoding="utf-8")
        source = (
            self.repo_root
            / "acesim"
            / "third_party"
            / "aircraft"
            / "PX4-Autopilot"
            / "src"
            / "modules"
            / "rl_mc_arm_control"
            / "rl_mc_arm_control.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("arm_joint_command", header)
        self.assertIn("arm_joint_state", header)
        self.assertNotIn("_arm_joint_status_sub", header)
        self.assertIn("_arm_joint_command_sub.update", source)
        self.assertIn("_arm_joint_state_sub.update", source)
        self.assertNotIn("_arm_joint_status_sub.update", source)


if __name__ == "__main__":
    unittest.main()
