"""ZeroMQ publisher for the manipulator state exported by ``AMEnv``.

The wire format is fixed: one timestamp in microseconds followed by the
positions, velocities, and efforts of the exported arm and gripper joints.
"""

from __future__ import annotations

from typing import Sequence

from acesim.utils.sim_streams import ArmStateCodec, LatestZmqPublisher


class ArmStatePublisher:
    """Publish a fixed-size arm state payload over ZeroMQ."""

    JOINT_COUNT = ArmStateCodec.JOINT_COUNT

    def __init__(
        self,
        zmq_endpoint: str = "tcp://0.0.0.0:5603",
        enable_zmq: bool = True,
    ) -> None:
        self._publisher = LatestZmqPublisher(zmq_endpoint, enabled=enable_zmq)

    def publish(
        self,
        timestamp_us: int,
        positions: Sequence[float],
        velocities: Sequence[float],
        efforts: Sequence[float],
    ) -> None:
        """Publish one timestamped arm state sample."""

        payload = ArmStateCodec.pack(timestamp_us, positions, velocities, efforts)
        self._publisher.publish(payload)

    def close(self) -> None:
        """Close the PUB socket and stop future publication."""

        self._publisher.close()
