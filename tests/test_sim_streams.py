from __future__ import annotations

import unittest
from unittest.mock import patch

from acesim.utils import sim_streams
from acesim.utils.math import calculate_coupled_gripper_positions
from acesim.utils.sim_streams import (
    ArmCommandCodec,
    ArmCommandStreamParams,
    ArmCommandStreamPublisher,
    ArmCommandStreamSubscriber,
    ArmStateCodec,
    ClockCodec,
    ControlStreamCodec,
    LatestZmqPublisher,
    VehicleTruthCodec,
    VehicleTruthStatePublisher,
    VehicleTruthStreamParams,
)


class _FakeSocket:
    def __init__(self) -> None:
        self.sockopts: list[tuple[object, object]] = []
        self.sent: list[tuple[bytes, object]] = []
        self.bind_endpoint: str | None = None
        self.connect_endpoint: str | None = None
        self.recv_values: list[bytes] = []
        self.closed = False

    def setsockopt(self, option: object, value: object) -> None:
        self.sockopts.append((option, value))

    def bind(self, endpoint: str) -> None:
        self.bind_endpoint = endpoint

    def connect(self, endpoint: str) -> None:
        self.connect_endpoint = endpoint

    def send(self, payload: bytes, flags: object = None) -> None:
        self.sent.append((payload, flags))

    def recv(self, flags: object = None) -> bytes:
        if not self.recv_values:
            raise sim_streams.zmq.Again()
        return self.recv_values.pop(0)

    def close(self, linger: int = 0) -> None:
        self.closed = True


class _FakeContext:
    last_socket: _FakeSocket | None = None

    @classmethod
    def instance(cls) -> "_FakeContext":
        return cls()

    def socket(self, _socket_type: object) -> _FakeSocket:
        socket = _FakeSocket()
        _FakeContext.last_socket = socket
        return socket


class SimStreamsTests(unittest.TestCase):
    def test_coupled_gripper_positions_match_mujoco_slide_ranges(self) -> None:
        self.assertEqual(calculate_coupled_gripper_positions(0.0), (0.0, 0.0))

        closed_left, closed_right = calculate_coupled_gripper_positions(-1.723)

        self.assertAlmostEqual(closed_left, -0.04225, places=5)
        self.assertAlmostEqual(closed_right, -0.04225, places=5)
        self.assertGreaterEqual(closed_left, -0.04225)
        self.assertGreaterEqual(closed_right, -0.04225)
        self.assertLessEqual(closed_left, 0.0)
        self.assertLessEqual(closed_right, 0.0)

    def test_clock_codec_round_trips_timestamp(self) -> None:
        payload = ClockCodec.pack(2_500_000)

        self.assertEqual(ClockCodec.unpack(payload), 2_500_000)

    def test_arm_state_codec_round_trips_seven_joint_state_vectors(self) -> None:
        payload = ArmStateCodec.pack(
            123456,
            [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02],
            [1.1, 1.2, 1.3, 1.4, 1.5, -0.1, 0.2],
            [9.1, 9.2, 9.3, 9.4, 9.5, -1.0, 2.0],
        )

        decoded = ArmStateCodec.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["joint_count"], 7)
        self.assertEqual(decoded["positions"], [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02])
        self.assertEqual(decoded["velocities"], [1.1, 1.2, 1.3, 1.4, 1.5, -0.1, 0.2])
        self.assertEqual(decoded["efforts"], [9.1, 9.2, 9.3, 9.4, 9.5, -1.0, 2.0])

    def test_arm_state_codec_unpacks_legacy_five_joint_payload(self) -> None:
        payload = ArmStateCodec.LEGACY_STRUCT.pack(
            123456,
            0.1,
            0.2,
            0.3,
            0.4,
            0.5,
            1.1,
            1.2,
            1.3,
            1.4,
            1.5,
            9.1,
            9.2,
            9.3,
            9.4,
            9.5,
        )

        decoded = ArmStateCodec.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["positions"], [0.1, 0.2, 0.3, 0.4, 0.5])
        self.assertEqual(decoded["joint_count"], 5)

    def test_arm_state_codec_rejects_wrong_joint_count(self) -> None:
        with self.assertRaisesRegex(ValueError, "positions must contain exactly 7 values"):
            ArmStateCodec.pack(123456, [1.0, 2.0], [0.0] * 7, [0.0] * 7)

    def test_arm_command_codec_round_trips_seven_joint_command(self) -> None:
        payload = ArmCommandCodec.pack(
            123456,
            "cmd-7",
            [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02],
        )

        decoded = ArmCommandCodec.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["command_id"], "cmd-7")
        self.assertEqual(decoded["positions"], [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02])

    def test_arm_command_codec_rejects_wrong_joint_count_and_nonfinite_values(self) -> None:
        with self.assertRaisesRegex(ValueError, "positions must contain exactly 7 values"):
            ArmCommandCodec.pack(123456, "bad-count", [0.0] * 5)
        with self.assertRaisesRegex(ValueError, "positions must contain finite values"):
            ArmCommandCodec.pack(123456, "bad-finite", [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0, 0.0])

    def test_arm_command_stream_params_parse_config_and_endpoint_override(self) -> None:
        with patch.dict(
            "os.environ",
            {"ACESIM_ARM_COMMAND_STREAM_ENDPOINT": "tcp://127.0.0.1:5614"},
            clear=False,
        ):
            params = ArmCommandStreamParams.from_asset_params(
                {
                    "arm_command_stream": {
                        "enabled": True,
                        "zmq_endpoint": "tcp://0.0.0.0:5604",
                    }
                }
            )

        self.assertTrue(params.enabled)
        self.assertEqual(params.zmq_endpoint, "tcp://127.0.0.1:5614")

    def test_arm_command_stream_publisher_uses_ace_follower_endpoint_override(self) -> None:
        params = ArmCommandStreamParams(enabled=True, zmq_endpoint="tcp://127.0.0.1:5604")
        with (
            patch("acesim.utils.sim_streams.zmq.Context", _FakeContext),
            patch.dict(
                "os.environ",
                {"ACESIM_ACE_FOLLOWER_COMMAND_ENDPOINT": "tcp://0.0.0.0:5614"},
                clear=False,
            ),
        ):
            publisher = ArmCommandStreamPublisher(params)

        socket = _FakeContext.last_socket
        assert socket is not None
        self.assertEqual(socket.bind_endpoint, "tcp://0.0.0.0:5614")

        publisher.publish(123456, "cmd-1", [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02])
        self.assertEqual(
            ArmCommandCodec.unpack(socket.sent[-1][0])["positions"],
            [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02],
        )

    def test_arm_command_stream_subscriber_keeps_newest_available_command(self) -> None:
        params = ArmCommandStreamParams(enabled=True, zmq_endpoint="tcp://127.0.0.1:5604")
        with patch("acesim.utils.sim_streams.zmq.Context", _FakeContext):
            subscriber = ArmCommandStreamSubscriber(params)

        socket = _FakeContext.last_socket
        assert socket is not None
        socket.recv_values.extend(
            [
                ArmCommandCodec.pack(1, "old", [0.0] * 7),
                ArmCommandCodec.pack(2, "new", [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02]),
            ]
        )

        latest = subscriber.read_latest()

        self.assertIsNotNone(latest)
        assert latest is not None
        self.assertEqual(latest["timestamp_us"], 2)
        self.assertEqual(latest["command_id"], "new")
        self.assertEqual(latest["positions"], [0.1, 0.2, 0.3, 0.4, 0.5, -0.01, 0.02])

    def test_control_stream_codec_round_trips_released_controls(self) -> None:
        payload = ControlStreamCodec.pack(123456, [0.0, 0.25, 0.5, 1.0])

        decoded = ControlStreamCodec.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["channel_count"], 4)
        self.assertEqual(decoded["controls"], [0.0, 0.25, 0.5, 1.0])

    def test_control_stream_codec_rejects_too_many_channels(self) -> None:
        with self.assertRaisesRegex(ValueError, "controls supports at most 8 channels"):
            ControlStreamCodec.pack(123456, [0.1] * 9)

    def test_vehicle_truth_codec_round_trips_truth_state(self) -> None:
        payload = VehicleTruthCodec.pack(
            123456,
            [1.0, 2.0, 3.0],
            [0.9, 0.1, 0.2, 0.3],
            [4.0, 5.0, 6.0],
            [0.7, 0.8, 0.9],
        )

        decoded = VehicleTruthCodec.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["position_world_m_nwu"], [1.0, 2.0, 3.0])
        self.assertEqual(decoded["attitude_world_quat_scalar_first"], [0.9, 0.1, 0.2, 0.3])
        self.assertEqual(decoded["linear_velocity_world_mps_nwu"], [4.0, 5.0, 6.0])
        self.assertEqual(decoded["angular_velocity_body_radps_flu"], [0.7, 0.8, 0.9])

    def test_vehicle_truth_codec_rejects_wrong_vector_lengths(self) -> None:
        with self.assertRaisesRegex(ValueError, "position_world_m_nwu must contain exactly 3 values"):
            VehicleTruthCodec.pack(123456, [1.0, 2.0], [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3)
        with self.assertRaisesRegex(ValueError, "attitude_world_quat_scalar_first must contain exactly 4 values"):
            VehicleTruthCodec.pack(123456, [0.0] * 3, [1.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3)

    def test_vehicle_truth_stream_params_parse_config_and_endpoint_override(self) -> None:
        with patch.dict(
            "os.environ",
            {"ACESIM_TRUTH_ZMQ_ENDPOINT": "tcp://127.0.0.1:5615"},
            clear=False,
        ):
            params = VehicleTruthStreamParams.from_asset_params(
                {
                    "truth_stream": {
                        "enabled": True,
                        "rate_hz": 120.0,
                        "zmq_endpoint": "tcp://0.0.0.0:5605",
                    }
                }
            )

        self.assertTrue(params.enabled)
        self.assertEqual(params.rate_hz, 120.0)
        self.assertEqual(params.zmq_endpoint, "tcp://127.0.0.1:5615")

    def test_vehicle_truth_stream_publisher_disabled_does_not_create_socket(self) -> None:
        _FakeContext.last_socket = None
        with patch("acesim.utils.sim_streams.zmq.Context", _FakeContext):
            publisher = VehicleTruthStatePublisher(VehicleTruthStreamParams(enabled=False))

        self.assertIsNone(_FakeContext.last_socket)
        publisher.publish(123456, [0.0] * 3, [1.0, 0.0, 0.0, 0.0], [0.0] * 3, [0.0] * 3)
        publisher.close()

    def test_vehicle_truth_stream_publisher_sends_encoded_truth_payload(self) -> None:
        params = VehicleTruthStreamParams(enabled=True, rate_hz=120.0, zmq_endpoint="tcp://0.0.0.0:5605")
        with patch("acesim.utils.sim_streams.zmq.Context", _FakeContext):
            publisher = VehicleTruthStatePublisher(params)

        socket = _FakeContext.last_socket
        assert socket is not None
        self.assertEqual(socket.bind_endpoint, "tcp://0.0.0.0:5605")

        publisher.publish(123456, [1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0], [4.0, 5.0, 6.0], [0.1, 0.2, 0.3])

        self.assertEqual(
            VehicleTruthCodec.unpack(socket.sent[-1][0])["linear_velocity_world_mps_nwu"],
            [4.0, 5.0, 6.0],
        )

    def test_latest_zmq_publisher_uses_latest_sample_socket_options(self) -> None:
        with patch("acesim.utils.sim_streams.zmq.Context", _FakeContext):
            publisher = LatestZmqPublisher("tcp://0.0.0.0:5600")

        socket = _FakeContext.last_socket
        assert socket is not None
        self.assertEqual(socket.bind_endpoint, "tcp://0.0.0.0:5600")
        self.assertIn((sim_streams.zmq.LINGER, 0), socket.sockopts)
        self.assertIn((sim_streams.zmq.SNDHWM, 1), socket.sockopts)
        self.assertIn((sim_streams.zmq.CONFLATE, 1), socket.sockopts)

        publisher.publish(b"sample")
        self.assertEqual(socket.sent, [(b"sample", sim_streams.zmq.NOBLOCK)])

        publisher.close()
        self.assertTrue(socket.closed)

    def test_latest_zmq_publisher_disabled_does_not_create_socket(self) -> None:
        _FakeContext.last_socket = None
        with patch("acesim.utils.sim_streams.zmq.Context", _FakeContext):
            publisher = LatestZmqPublisher("tcp://0.0.0.0:5600", enabled=False)

        self.assertIsNone(_FakeContext.last_socket)
        publisher.publish(b"ignored")
        publisher.close()


if __name__ == "__main__":
    unittest.main()
