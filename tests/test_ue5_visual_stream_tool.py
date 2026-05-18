import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import zmq

from acesim.utils.vehicle_visual_state_publisher import VehicleVisualStatePublisher

ROOT = Path(__file__).resolve().parents[1]


def _localhost_zmq_available() -> bool:
    probe = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import zmq; "
                "c=zmq.Context.instance(); "
                "s=c.socket(zmq.PUB); "
                "s.setsockopt(zmq.LINGER, 0); "
                "p=s.bind_to_random_port('tcp://127.0.0.1'); "
                "s.close(0); "
                "print(p)"
            ),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=3.0,
    )
    return probe.returncode == 0


def _visual_payload(timestamp_us: int) -> bytes:
    return VehicleVisualStatePublisher._STRUCT.pack(
        timestamp_us,
        1.0,
        2.0,
        3.0,
        1.0,
        0.0,
        0.0,
        0.0,
        4,
        0.1,
        0.2,
        0.3,
        0.4,
        0.0,
        0.0,
        0.0,
        0.0,
        10.0,
        20.0,
        30.0,
        40.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def test_verify_visual_stream_decodes_live_zmq_samples() -> None:
    if not _localhost_zmq_available():
        pytest.skip("localhost ZeroMQ sockets are unavailable in this environment")

    endpoint_ready = threading.Event()
    endpoint_holder: dict[str, str] = {}
    stop_requested = threading.Event()

    def publish_samples() -> None:
        context = zmq.Context.instance()
        socket = context.socket(zmq.PUB)
        socket.setsockopt(zmq.LINGER, 0)
        port = socket.bind_to_random_port("tcp://127.0.0.1")
        endpoint_holder["endpoint"] = f"tcp://127.0.0.1:{port}"
        endpoint_ready.set()

        timestamp_us = 1_000
        try:
            while not stop_requested.is_set():
                socket.send(_visual_payload(timestamp_us))
                timestamp_us += 1_000
                time.sleep(0.02)
        finally:
            socket.close(linger=0)

    publisher_thread = threading.Thread(target=publish_samples)
    publisher_thread.start()
    assert endpoint_ready.wait(timeout=3.0)

    try:
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "acesim" / "tools" / "ue5" / "verify_visual_stream.py"),
                "--endpoint",
                endpoint_holder["endpoint"],
                "--samples",
                "3",
                "--timeout-sec",
                "3.0",
            ],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    finally:
        stop_requested.set()
        publisher_thread.join(timeout=1.0)

    assert "Listening on " in result.stdout
    assert "sample=3" in result.stdout
    assert "pos=[1. 2. 3.]" in result.stdout
    assert "rotors=4" in result.stdout
