from __future__ import annotations

import unittest
from unittest.mock import patch

from acesim.utils import sim_streams
from acesim.utils.sim_streams import ArmStateCodec, ClockCodec, LatestZmqPublisher


class _FakeSocket:
    def __init__(self) -> None:
        self.sockopts: list[tuple[object, object]] = []
        self.sent: list[tuple[bytes, object]] = []
        self.bind_endpoint: str | None = None
        self.closed = False

    def setsockopt(self, option: object, value: object) -> None:
        self.sockopts.append((option, value))

    def bind(self, endpoint: str) -> None:
        self.bind_endpoint = endpoint

    def send(self, payload: bytes, flags: object = None) -> None:
        self.sent.append((payload, flags))

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
    def test_clock_codec_round_trips_timestamp(self) -> None:
        payload = ClockCodec.pack(2_500_000)

        self.assertEqual(ClockCodec.unpack(payload), 2_500_000)

    def test_arm_state_codec_round_trips_five_joint_state_vectors(self) -> None:
        payload = ArmStateCodec.pack(
            123456,
            [0.1, 0.2, 0.3, 0.4, 0.5],
            [1.1, 1.2, 1.3, 1.4, 1.5],
            [9.1, 9.2, 9.3, 9.4, 9.5],
        )

        decoded = ArmStateCodec.unpack(payload)

        self.assertEqual(decoded["timestamp_us"], 123456)
        self.assertEqual(decoded["positions"], [0.1, 0.2, 0.3, 0.4, 0.5])
        self.assertEqual(decoded["velocities"], [1.1, 1.2, 1.3, 1.4, 1.5])
        self.assertEqual(decoded["efforts"], [9.1, 9.2, 9.3, 9.4, 9.5])

    def test_arm_state_codec_rejects_wrong_joint_count(self) -> None:
        with self.assertRaisesRegex(ValueError, "positions must contain exactly 5 values"):
            ArmStateCodec.pack(123456, [1.0, 2.0], [0.0] * 5, [0.0] * 5)

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
