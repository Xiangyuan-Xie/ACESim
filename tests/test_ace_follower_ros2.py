from __future__ import annotations

from typing import Any

import pytest

from acesim.utils.math import calculate_coupled_gripper_positions
from tests.ros2_bridge_testbed import load_bridge_package_module


class _FakeArmCommandPublisher:
    instances: list["_FakeArmCommandPublisher"] = []

    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self.published: list[tuple[int, str, list[float]]] = []
        self.closed = False
        self.__class__.instances.append(self)

    def publish(self, timestamp_us: int, command_id: str, positions: list[float]) -> None:
        self.published.append((int(timestamp_us), command_id, list(positions)))

    def close(self) -> None:
        self.closed = True


class _FakeArmStateSubscriber:
    instances: list["_FakeArmStateSubscriber"] = []
    next_sample: dict[str, Any] | None = None

    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self.closed = False
        self.__class__.instances.append(self)

    def read_latest(self) -> dict[str, Any] | None:
        sample = self.__class__.next_sample
        self.__class__.next_sample = None
        return sample

    def close(self) -> None:
        self.closed = True


def _string(module: Any, value: str) -> Any:
    message = module.String()
    message.data = value
    return message


def _joint_state(module: Any, positions: list[float]) -> Any:
    message = module.JointState()
    message.position = positions
    return message


def _load_ace_follower_module(monkeypatch: pytest.MonkeyPatch) -> Any:
    _FakeArmCommandPublisher.instances.clear()
    _FakeArmStateSubscriber.instances.clear()
    _FakeArmStateSubscriber.next_sample = None
    module = load_bridge_package_module("acesim_ros2.ace_follower", "ace_follower.py")
    monkeypatch.setattr(module, "ArmCommandStreamPublisher", _FakeArmCommandPublisher)
    monkeypatch.setattr(module, "ArmStateStreamSubscriber", _FakeArmStateSubscriber)
    return module


def test_acetele_compatible_topics_use_default_reliable_qos(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()

    topics = [
        "/ace_follower/arm/state",
        "/ace_follower/gripper/state",
        "/ace_follower/arm/sync_status",
        "/ace_leader/arm/command",
        "/ace_leader/gripper/command",
        "/ace_leader/arm/sync_mode",
    ]
    qos_profiles = [node.publishers[topic].qos for topic in topics[:3]]
    qos_profiles.extend(node.subscriptions[f"{topic}__qos"] for topic in topics[3:])

    for qos in qos_profiles:
        assert qos.depth == 10
        assert "reliability" not in qos.kwargs
        assert "durability" not in qos.kwargs
        assert "history" not in qos.kwargs


def test_tracking_mode_forwards_leader_command_to_arm_command_stream(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()

    node.subscriptions["/ace_leader/arm/sync_mode"](_string(module, "tracking"))
    node.subscriptions["/ace_leader/gripper/command"](_joint_state(module, [1.0]))
    node.subscriptions["/ace_leader/arm/command"](_joint_state(module, [0.1, 0.2, 0.3, 0.4]))

    publisher = _FakeArmCommandPublisher.instances[0]
    expected_gripper = list(calculate_coupled_gripper_positions(0.0))
    assert publisher.published == [(0, "ace_leader", [0.1, 0.2, 0.3, 0.4, 0.0, *expected_gripper])]
    assert node.publishers["/ace_follower/arm/sync_status"].messages[-1].data == "tracking"

    node.destroy_node()
    assert publisher.closed


def test_tracking_mode_maps_zero_gripper_command_to_closed_mujoco_pose(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()

    node.subscriptions["/ace_leader/arm/sync_mode"](_string(module, "tracking"))
    node.subscriptions["/ace_leader/gripper/command"](_joint_state(module, [0.0]))
    node.subscriptions["/ace_leader/arm/command"](_joint_state(module, [0.1, 0.2, 0.3, 0.4]))

    publisher = _FakeArmCommandPublisher.instances[0]
    expected_gripper = list(calculate_coupled_gripper_positions(-1.723))
    assert publisher.published == [(0, "ace_leader", [0.1, 0.2, 0.3, 0.4, -1.723, *expected_gripper])]


def test_idle_and_ready_modes_do_not_forward_leader_commands(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()

    node.subscriptions["/ace_leader/arm/sync_mode"](_string(module, "idle"))
    node.subscriptions["/ace_leader/arm/command"](_joint_state(module, [0.1, 0.2, 0.3, 0.4]))
    node.subscriptions["/ace_leader/arm/sync_mode"](_string(module, "ready"))
    node.subscriptions["/ace_leader/arm/command"](_joint_state(module, [0.5, 0.4, 0.3, 0.2]))

    assert _FakeArmCommandPublisher.instances[0].published == []
    assert node.publishers["/ace_follower/arm/sync_status"].messages[-1].data == "ready"


def test_tracking_heartbeat_timeout_publishes_lost(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()
    node.parameters["heartbeat_timeout_sec"] = 0.5

    node.subscriptions["/ace_leader/arm/sync_mode"](_string(module, "tracking"))
    node.subscriptions["/ace_leader/arm/command"](_joint_state(module, [0.1, 0.2, 0.3, 0.4]))
    node._now_ns = 600_000_000
    node._sync_timer_callback()

    assert node.publishers["/ace_follower/arm/sync_status"].messages[-1].data == "lost"


def test_sync_status_is_republished_periodically(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()

    node.subscriptions["/ace_leader/arm/sync_mode"](_string(module, "sync_request"))
    publisher = node.publishers["/ace_follower/arm/sync_status"]
    message_count = len(publisher.messages)

    node._sync_timer_callback()

    assert len(publisher.messages) == message_count + 1
    assert publisher.messages[-1].data == "ready"


def test_arm_state_stream_is_republished_as_acetele_joint_states(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    _FakeArmStateSubscriber.next_sample = {
        "timestamp_us": 123456,
        "joint_count": 7,
        "positions": [0.1, 0.2, 0.3, 0.4, -0.8615, -0.01, -0.01],
        "velocities": [1.0, 1.1, 1.2, 1.3, 0.1723, -0.1, -0.1],
        "efforts": [2.0, 2.1, 2.2, 2.3, 2.4, -0.2, 0.4],
    }
    node = module.ACESimACEFollowerNode()

    node._state_timer_callback()

    arm_msg = node.publishers["/ace_follower/arm/state"].messages[-1]
    gripper_msg = node.publishers["/ace_follower/gripper/state"].messages[-1]
    assert arm_msg.name == ["joint_1", "joint_2", "joint_3", "joint_4"]
    assert arm_msg.position == [0.1, 0.2, 0.3, 0.4]
    assert arm_msg.velocity == [1.0, 1.1, 1.2, 1.3]
    assert arm_msg.effort == [2.0, 2.1, 2.2, 2.3]
    assert gripper_msg.name == ["joint_5"]
    assert gripper_msg.position == [pytest.approx(0.5)]
    assert gripper_msg.velocity == [pytest.approx(0.1)]
    assert gripper_msg.effort == [2.4]


def test_arm_state_stream_maps_open_and_closed_joint5_to_public_gripper(monkeypatch: pytest.MonkeyPatch) -> None:
    module = _load_ace_follower_module(monkeypatch)
    node = module.ACESimACEFollowerNode()

    for joint5, expected_public in ((0.0, 1.0), (-1.723, 0.0)):
        _FakeArmStateSubscriber.next_sample = {
            "timestamp_us": 123456,
            "joint_count": 7,
            "positions": [0.1, 0.2, 0.3, 0.4, joint5, 0.0, 0.0],
            "velocities": [0.0] * 7,
            "efforts": [0.0] * 7,
        }

        node._state_timer_callback()

        gripper_msg = node.publishers["/ace_follower/gripper/state"].messages[-1]
        assert gripper_msg.position == [pytest.approx(expected_public)]
