from __future__ import annotations

import unittest

from acesim.utils.arm_command_publisher import ArmCommandPublisher


class ArmCommandPublisherTests(unittest.TestCase):
    def test_pack_payload_encodes_timestamp_and_five_joint_positions(self) -> None:
        payload = ArmCommandPublisher.pack_payload(123456, [1.0, 2.0, 3.0, 4.0, 5.0])
        decoded = ArmCommandPublisher.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["joint_positions"], [1.0, 2.0, 3.0, 4.0, 5.0])

    def test_pack_payload_rejects_wrong_joint_count(self) -> None:
        with self.assertRaisesRegex(ValueError, "joint_positions must contain exactly 5 values"):
            ArmCommandPublisher.pack_payload(123456, [1.0, 2.0, 3.0])


if __name__ == "__main__":
    unittest.main()
