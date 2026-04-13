from __future__ import annotations

import importlib
import unittest
from types import SimpleNamespace

from acesim.utils.arm_servo_scheduler import ArmControlSample


class MCArmEnvCommandPublishingTests(unittest.TestCase):
    def test_publish_arm_command_exports_first_five_ros_joints(self) -> None:
        mc_arm_env = importlib.import_module("acesim.env.mujoco.mc_arm_env")

        published: list[tuple[int, list[float]]] = []

        fake_env = SimpleNamespace(
            _arm_command_publisher=SimpleNamespace(
                publish=lambda timestamp_us, joint_positions: published.append((timestamp_us, list(joint_positions)))
            ),
            _arm_ros_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5"],
        )

        mc_arm_env.MCArmEnv._publish_arm_command(
            fake_env,
            123456,
            ArmControlSample(joint_positions=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]),
        )

        self.assertEqual(published, [(123456, [1.0, 2.0, 3.0, 4.0, 5.0])])


if __name__ == "__main__":
    unittest.main()
