#!/usr/bin/env python3
"""Subscribe to the ACESim visual stream and print decoded samples."""

from __future__ import annotations

import argparse
import sys
import time

import zmq

from acesim.utils.vehicle_visual_state_publisher import VehicleVisualStatePublisher


def main() -> None:
    parser = argparse.ArgumentParser(description="Subscribe to ACESim's UE visual stream.")
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5601")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=10.0,
        help="Maximum time to wait for each sample before exiting with an error.",
    )
    args = parser.parse_args()
    if args.samples < 0:
        raise ValueError("--samples must be non-negative")
    if args.timeout_sec <= 0.0:
        raise ValueError("--timeout-sec must be positive")

    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt(zmq.RCVTIMEO, int(args.timeout_sec * 1000.0))
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.connect(args.endpoint)

    print(f"Listening on {args.endpoint}")
    received = 0
    try:
        while received < args.samples:
            try:
                payload = socket.recv()
            except zmq.Again:
                print(
                    f"Timed out waiting for sample {received + 1}/{args.samples} " f"after {args.timeout_sec:.3f}s",
                    file=sys.stderr,
                )
                raise SystemExit(1)
            sample = VehicleVisualStatePublisher.unpack(payload)
            received += 1
            print(
                f"sample={received} ts={sample['timestamp_us']} "
                f"pos={sample['position_world_m_nwu']} "
                f"quat={sample['attitude_world_quat_scalar_first']} "
                f"rotors={sample['rotor_count']}"
            )
    finally:
        socket.close(linger=0)
        time.sleep(0.05)


if __name__ == "__main__":
    main()
