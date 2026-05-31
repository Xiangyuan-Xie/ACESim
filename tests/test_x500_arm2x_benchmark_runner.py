from __future__ import annotations

import importlib.util
import json
import math
import sys
import types
from pathlib import Path
from typing import Any, Iterable, Sequence, cast
from unittest.mock import patch

import pytest
import zmq

from acesim.benchmark.x500_arm2x_velocity import VelocityTrackingCommand, VelocityTrackingSummary
from acesim.utils.math import calculate_coupled_gripper_positions


def _load_runner_module() -> Any:
    module_name = "_test_acesim_ros2_x500_arm2x_benchmark"
    sys.modules.pop(module_name, None)
    module_path = (
        Path(__file__).resolve().parents[1]
        / "acesim"
        / "deploy"
        / "aircraft"
        / "acesim_ros2"
        / "acesim_ros2"
        / "benchmark"
        / "x500_arm2x.py"
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_default_arm_pose_cases_are_named_and_joint_limited() -> None:
    runner = _load_runner_module()

    cases = runner.default_arm_pose_cases()

    assert [case.name for case in cases] == [
        "home_folded",
        "center_carry",
        "forward_mid",
        "left_high",
        "right_high",
        "left_low",
        "right_low",
        "forward_grasp_low",
        "left_reach_high",
        "right_reach_forward",
    ]
    expected_first_five = {
        "home_folded": (-1.57, 3.14, 0.0, 0.0, 0.0),
        "center_carry": (0.0, 2.65, 0.0, -0.25, -0.35),
        "forward_mid": (0.0, 2.05, 0.0, -0.70, -0.55),
        "left_high": (0.55, 2.45, -0.30, -0.30, -0.45),
        "right_high": (-0.55, 2.45, 0.30, 0.25, -0.45),
        "left_low": (0.95, 1.85, -0.45, -0.65, -0.85),
        "right_low": (-0.75, 1.95, 0.18, 0.05, -0.55),
        "forward_grasp_low": (0.25, 1.60, 0.0, -0.85, -1.20),
        "left_reach_high": (1.20, 2.75, -0.65, -0.15, -0.70),
        "right_reach_forward": (-0.50, 2.10, 0.18, -0.25, -0.60),
    }
    for case in cases:
        assert len(case.pose) == 7
        runner.validate_arm_pose(case.pose)
        assert case.pose[:5] == expected_first_five[case.name]
        left_gripper, right_gripper = calculate_coupled_gripper_positions(case.pose[4])
        assert case.pose[5] == pytest.approx(left_gripper)
        assert case.pose[6] == pytest.approx(right_gripper)


def test_default_arm_pose_cases_have_diverse_first_two_joints() -> None:
    runner = _load_runner_module()
    cases = runner.default_arm_pose_cases()

    joint_1_values = [case.pose[0] for case in cases]
    joint_2_values = [case.pose[1] for case in cases]

    assert max(joint_1_values) - min(joint_1_values) >= 1.8
    assert max(joint_2_values) - min(joint_2_values) >= 1.5
    assert len({round(value, 2) for value in joint_1_values}) >= 6
    assert len({round(value, 2) for value in joint_2_values}) >= 5


def test_default_arm_motion_duration_is_slow() -> None:
    runner = _load_runner_module()

    assert runner.BenchmarkRuntimeConfig().arm_motion_duration_s == 10.0
    assert runner._parse_args([]).arm_motion_duration_s == 10.0


def test_velocity_setpoint_payload_converts_heading_frame_to_px4_ned() -> None:
    runner = _load_runner_module()
    command = VelocityTrackingCommand(
        active=True,
        segment_name="forward",
        velocity_h=(1.0, 0.0, 0.0),
        yaw_rate=0.25,
    )

    payload = runner.make_velocity_setpoint_payload(command, heading_w=math.pi / 2.0, timestamp_us=123)

    assert payload["timestamp"] == 123
    assert payload["velocity"] == pytest.approx((1.0, 0.0, 0.0))
    assert payload["yawspeed"] == -0.25
    assert all(math.isnan(value) for value in payload["position"])
    assert all(math.isnan(value) for value in payload["acceleration"])
    assert all(math.isnan(value) for value in payload["jerk"])
    assert math.isnan(payload["yaw"])


def test_offboard_control_mode_payload_requests_velocity_control() -> None:
    runner = _load_runner_module()

    payload = runner.make_offboard_control_mode_payload(timestamp_us=456)

    assert payload == {
        "timestamp": 456,
        "position": False,
        "velocity": True,
        "acceleration": False,
        "attitude": False,
        "body_rate": False,
        "thrust_and_torque": False,
        "direct_actuator": False,
    }


def test_offboard_control_mode_payload_can_request_position_control() -> None:
    runner = _load_runner_module()

    payload = runner.make_offboard_control_mode_payload(timestamp_us=789, position=True, velocity=False)

    assert payload["timestamp"] == 789
    assert payload["position"] is True
    assert payload["velocity"] is False


def test_position_setpoint_payload_targets_px4_ned_altitude() -> None:
    runner = _load_runner_module()

    payload = runner.make_position_setpoint_payload(position_ned=(math.nan, math.nan, -1.5), timestamp_us=101)

    assert payload["timestamp"] == 101
    assert math.isnan(payload["position"][0])
    assert math.isnan(payload["position"][1])
    assert payload["position"][2] == -1.5
    assert all(math.isnan(value) for value in payload["velocity"])
    assert all(math.isnan(value) for value in payload["acceleration"])
    assert all(math.isnan(value) for value in payload["jerk"])


def test_controller_accepts_versioned_px4_local_position_topic() -> None:
    runner = _load_runner_module()
    subscriptions: dict[str, object] = {}

    class FakeClockNow:
        nanoseconds = 123_000

    class FakeClock:
        def now(self) -> FakeClockNow:
            return FakeClockNow()

    class FakePublisher:
        def publish(self, message: object) -> None:
            pass

    class FakeNode:
        def create_publisher(self, msg_type: object, topic: str, qos: object) -> FakePublisher:
            return FakePublisher()

        def create_subscription(self, msg_type: object, topic: str, callback: object, qos: object) -> object:
            subscriptions[topic] = callback
            return object()

        def get_clock(self) -> FakeClock:
            return FakeClock()

        def destroy_node(self) -> None:
            pass

    class FakeQoSProfile:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs

    class OffboardControlMode:
        pass

    class TrajectorySetpoint:
        pass

    class VehicleAngularVelocity:
        pass

    class VehicleAttitude:
        pass

    class VehicleGlobalPosition:
        pass

    class VehicleLocalPosition:
        pass

    class VehicleLandDetected:
        pass

    fake_rclpy: Any = types.ModuleType("rclpy")
    fake_rclpy.create_node = lambda name: FakeNode()
    fake_rclpy.spin_once = lambda node, timeout_sec=0.0: None
    fake_qos: Any = types.ModuleType("rclpy.qos")
    fake_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=object())
    fake_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=object())
    fake_qos.QoSProfile = FakeQoSProfile
    fake_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=object())
    fake_px4_msgs: Any = types.ModuleType("px4_msgs")
    fake_px4_msgs_msg: Any = types.ModuleType("px4_msgs.msg")
    fake_px4_msgs_msg.OffboardControlMode = OffboardControlMode
    fake_px4_msgs_msg.TrajectorySetpoint = TrajectorySetpoint
    fake_px4_msgs_msg.VehicleAngularVelocity = VehicleAngularVelocity
    fake_px4_msgs_msg.VehicleAttitude = VehicleAttitude
    fake_px4_msgs_msg.VehicleGlobalPosition = VehicleGlobalPosition
    fake_px4_msgs_msg.VehicleLandDetected = VehicleLandDetected
    fake_px4_msgs_msg.VehicleLocalPosition = VehicleLocalPosition
    fake_px4_msgs.msg = fake_px4_msgs_msg

    with patch.dict(
        sys.modules,
        {
            "rclpy": fake_rclpy,
            "rclpy.qos": fake_qos,
            "px4_msgs": fake_px4_msgs,
            "px4_msgs.msg": fake_px4_msgs_msg,
        },
    ):
        controller = runner.X500Arm2xBenchmarkController(runner.BenchmarkRuntimeConfig())

    local_position = types.SimpleNamespace(xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True)
    cast(Any, subscriptions["/fmu/out/vehicle_local_position_v1"])(local_position)

    assert controller._local_position is local_position
    assert "/fmu/out/vehicle_local_position" in subscriptions
    assert "/fmu/out/vehicle_land_detected" in subscriptions
    controller.close()


def test_send_am_offboard_mode_command_uses_px4_custom_submode() -> None:
    runner = _load_runner_module()
    calls: list[tuple[object, ...]] = []

    class FakeMavlink:
        MAV_CMD_DO_SET_MODE = 176
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

    class FakeMavSender:
        def command_long_send(self, *args: object) -> None:
            calls.append(args)

    class FakeConnection:
        target_system = 42
        target_component = 84
        mav = FakeMavSender()

    runner.send_am_offboard_mode_command(FakeConnection(), mavlink=FakeMavlink)

    assert calls == [(42, 84, 176, 0, 1.0, 6.0, 1.0, 0.0, 0.0, 0.0, 0.0)]


def test_send_offboard_mode_command_uses_px4_default_submode() -> None:
    runner = _load_runner_module()
    calls: list[tuple[object, ...]] = []

    class FakeMavlink:
        MAV_CMD_DO_SET_MODE = 176
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

    class FakeMavSender:
        def command_long_send(self, *args: object) -> None:
            calls.append(args)

    class FakeConnection:
        target_system = 42
        target_component = 84
        mav = FakeMavSender()

    runner.send_offboard_mode_command(FakeConnection(), mavlink=FakeMavlink)

    assert calls == [(42, 84, 176, 0, 1.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0)]


def test_send_land_command_requests_current_position_land() -> None:
    runner = _load_runner_module()
    calls: list[tuple[object, ...]] = []

    class FakeMavlink:
        MAV_CMD_NAV_LAND = 21

    class FakeMavSender:
        def command_long_send(self, *args: object) -> None:
            calls.append(args)

    class FakeConnection:
        target_system = 42
        target_component = 84
        mav = FakeMavSender()

    runner.send_land_command(FakeConnection(), mavlink=FakeMavlink)

    assert calls == [(42, 84, 21, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]


def test_send_takeoff_command_uses_current_position_and_target_altitude() -> None:
    runner = _load_runner_module()
    calls: list[tuple[object, ...]] = []

    class FakeMavlink:
        MAV_CMD_NAV_TAKEOFF = 22

    class FakeMavSender:
        def command_long_send(self, *args: object) -> None:
            calls.append(args)

    class FakeConnection:
        target_system = 42
        target_component = 84
        mav = FakeMavSender()

    runner.send_takeoff_command(FakeConnection(), lat_deg=39.1, lon_deg=116.2, alt_amsl_m=51.5, mavlink=FakeMavlink)

    assert calls == [(42, 84, 22, 0, 0.0, 0.0, 0.0, math.nan, 39.1, 116.2, 51.5)]


def test_wait_command_ack_retries_command_until_ack() -> None:
    runner = _load_runner_module()
    now = [100.0]
    resend_times: list[float] = []

    class FakeAck:
        command = 176
        result = 0

    class FakeMav:
        def recv_match(self, type: str, blocking: bool, timeout: float) -> object | None:
            now[0] += 0.11
            if len(resend_times) >= 2:
                return FakeAck()
            return None

    def resend() -> None:
        resend_times.append(now[0])

    with patch.object(runner.time, "monotonic", lambda: now[0]):
        runner._wait_command_ack(
            FakeMav(),
            176,
            timeout_s=1.0,
            resend=resend,
            resend_interval_s=0.2,
        )

    assert len(resend_times) >= 2


def test_takeoff_accepts_small_altitude_tolerance() -> None:
    runner = _load_runner_module()
    now = [0.0]

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig(
            takeoff_altitude_m=1.5,
            takeoff_altitude_tolerance_m=0.02,
            takeoff_timeout_s=0.1,
        )
        _period_s = 0.02

        def _altitude_m(self) -> float:
            return 1.4999465942382812

        def _spin_once(self, timeout_s: float = 0.0) -> None:
            pass

    def monotonic() -> float:
        now[0] += 0.01
        return now[0]

    with patch.object(runner.time, "monotonic", monotonic):
        with patch.object(runner.time, "sleep", lambda _duration: None):
            takeoff_success, max_altitude_m = runner.X500Arm2xBenchmarkController._takeoff(
                FakeController(), (math.nan, math.nan, -1.5)
            )

    assert takeoff_success is True
    assert max_altitude_m == pytest.approx(1.4999465942382812)


def test_case_passed_requires_takeoff_samples_and_thresholds() -> None:
    runner = _load_runner_module()
    thresholds = runner.BenchmarkThresholds(
        max_rms_speed_error_mps=0.5,
        max_abs_lateral_bias_mps=0.2,
        max_rms_yaw_rate_error_radps=0.3,
    )
    summary = VelocityTrackingSummary(
        sample_count=10,
        rms_speed_error_norm_mps=0.4,
        max_abs_lateral_velocity_bias_mps=0.1,
        rms_yaw_rate_error_radps=0.2,
    )

    assert runner.case_passed(summary, thresholds, takeoff_success=True)
    assert not runner.case_passed(summary, thresholds, takeoff_success=False)
    assert not runner.case_passed(VelocityTrackingSummary(), thresholds, takeoff_success=True)
    assert not runner.case_passed(
        VelocityTrackingSummary(sample_count=10, rms_speed_error_norm_mps=0.6),
        thresholds,
        takeoff_success=True,
    )


def test_run_benchmark_reports_case_progress_percentages(capsys: pytest.CaptureFixture[str]) -> None:
    runner = _load_runner_module()
    cases = [
        runner.ArmPoseCase("case_a", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0)),
        runner.ArmPoseCase("case_b", (0.0, 1.2, 0.0, 0.0, -0.5, 0.0, 0.0)),
    ]

    def fake_run_case(case: Any, runtime_config: object, isolation: Any | None = None) -> object:
        case_name = getattr(case, "name")
        controller = getattr(case, "controller")
        return runner.BenchmarkCaseResult(
            name=case_name,
            pose=getattr(case, "pose"),
            controller=controller,
            takeoff_success=case_name == "case_a",
            max_altitude_m=1.5 if case_name == "case_a" else 0.0,
            tracking_duration_s=1.0 if case_name == "case_a" else 0.0,
            passed=case_name == "case_a" and controller == "am",
            summary=VelocityTrackingSummary(sample_count=1 if case_name == "case_a" else 0),
            error=None if case_name == "case_a" and controller == "am" else "startup failed",
        )

    with patch.object(runner, "run_case", fake_run_case):
        result = runner.run_benchmark(cases, runner.BenchmarkRuntimeConfig())

    captured = capsys.readouterr()
    assert "[  0%] starting 4 case(s)" in captured.err
    assert "completed case_a/am: PASS" in captured.err
    assert "completed case_a/px4_position: FAIL" in captured.err
    assert "[100%] completed case_b/px4_position: FAIL" in captured.err
    assert result["passed"] is False


def test_controller_variants_expand_each_pose_case() -> None:
    runner = _load_runner_module()
    cases = [
        runner.ArmPoseCase("case_a", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0)),
        runner.ArmPoseCase("case_b", (0.0, 1.2, 0.0, 0.0, -0.5, 0.0, 0.0)),
    ]

    expanded = runner.expand_controller_cases(cases)

    assert [(case.name, case.controller) for case in expanded] == [
        ("case_a", "am"),
        ("case_a", "px4_position"),
        ("case_b", "am"),
        ("case_b", "px4_position"),
    ]


def test_run_benchmark_reuses_slot_zero_for_serial_cases() -> None:
    runner = _load_runner_module()
    cases = [
        runner.ArmPoseCase("case_a", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0)),
        runner.ArmPoseCase("case_b", (0.0, 1.2, 0.0, 0.0, -0.5, 0.0, 0.0)),
    ]
    seen_slots: list[int] = []
    seen_instances: list[int] = []

    def fake_run_case(case: Any, runtime_config: object, isolation: Any | None = None) -> object:
        assert isolation is not None
        seen_slots.append(isolation.slot)
        seen_instances.append(isolation.px4_instance)
        return runner.BenchmarkCaseResult(
            name=getattr(case, "name"),
            pose=getattr(case, "pose"),
            controller=getattr(case, "controller"),
            takeoff_success=True,
            max_altitude_m=1.5,
            tracking_duration_s=1.0,
            passed=True,
            summary=VelocityTrackingSummary(sample_count=1),
            isolation=isolation,
        )

    with patch.object(runner, "run_case", fake_run_case):
        result = runner.run_benchmark(cases, runner.BenchmarkRuntimeConfig(jobs=1))

    assert result["passed"] is True
    assert seen_slots == [0, 0, 0, 0]
    assert seen_instances == [0, 0, 0, 0]


def test_run_benchmark_reuses_parallel_slots_when_controller_cases_exceed_jobs() -> None:
    runner = _load_runner_module()
    cases = [
        runner.ArmPoseCase(f"case_{index}", (0.0, 1.0 + index * 0.1, 0.0, 0.0, -0.5, 0.0, 0.0)) for index in range(3)
    ]
    seen: list[tuple[str, str, int, int]] = []

    class ImmediateFuture:
        def __init__(self, result: object) -> None:
            self._result = result

        def result(self) -> object:
            return self._result

    class FakeExecutor:
        def __init__(self, max_workers: int) -> None:
            self.max_workers = max_workers

        def __enter__(self) -> "FakeExecutor":
            return self

        def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
            pass

        def submit(self, fn: object, payload: tuple[Any, object, Any]) -> ImmediateFuture:
            case, runtime_config, isolation = payload
            seen.append((case.name, case.controller, isolation.slot, isolation.px4_instance))
            return ImmediateFuture(
                runner.BenchmarkCaseResult(
                    name=case.name,
                    pose=case.pose,
                    controller=case.controller,
                    takeoff_success=True,
                    max_altitude_m=1.5,
                    tracking_duration_s=1.0,
                    passed=True,
                    summary=VelocityTrackingSummary(sample_count=1),
                    isolation=isolation,
                )
            )

    def fake_as_completed(futures: Iterable[object]) -> list[object]:
        return list(futures)

    with patch.object(runner.concurrent.futures, "ProcessPoolExecutor", FakeExecutor):
        with patch.object(runner.concurrent.futures, "as_completed", fake_as_completed):
            result = runner.run_benchmark(cases, runner.BenchmarkRuntimeConfig(jobs=2))

    assert result["passed"] is True
    assert [item[2] for item in seen] == [0, 1, 0, 1, 0, 1]
    assert [item[3] for item in seen] == [0, 1, 0, 1, 0, 1]


def _sample_report_result(runner: Any) -> dict[str, object]:
    return {
        "asset": "x500_arm2x",
        "passed": False,
        "profile": {"cycles": 1},
        "thresholds": {
            "max_abs_lateral_bias_mps": 0.35,
            "max_rms_speed_error_mps": 0.65,
            "max_rms_yaw_rate_error_radps": 0.45,
        },
        "cases": [
            runner.BenchmarkCaseResult(
                name="case_a",
                pose=(0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0),
                controller="am",
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=42.0,
                passed=True,
                summary=VelocityTrackingSummary(
                    sample_count=10,
                    rms_speed_error_norm_mps=0.1,
                    max_abs_lateral_velocity_bias_mps=0.2,
                    rms_yaw_rate_error_radps=0.05,
                ),
                arm_motion_summary=runner.ArmMotionSummary(
                    sample_count=3,
                    duration_s=2.0,
                    max_horizontal_offset_m=0.03,
                    rms_horizontal_offset_m=0.02,
                    max_vertical_offset_m=0.01,
                ),
                arm_motion_samples=(
                    runner.ArmMotionSample(
                        0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    ),
                    runner.ArmMotionSample(
                        1.0, 0.02, 0.01, -1.5, 0.02, 0.01, 0.0, 0.0, 0.0, 0.0, 0.01, -0.01, 0.02, 0.01, -0.01, 0.02, 0.5
                    ),
                    runner.ArmMotionSample(
                        2.0, 0.03, 0.0, -1.49, 0.03, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02, -0.02, 0.03, 0.02, -0.02, 0.03, 1.0
                    ),
                ),
                velocity_tracking_samples=(
                    runner.VelocityTrackingSample(0.0, "forward", 0.4, 0.0, 0.0, 0.3, 0.01, 0.0, 0.0, 0.0, 0.1),
                    runner.VelocityTrackingSample(0.1, "forward", 0.4, 0.0, 0.0, 0.35, 0.0, 0.0, 0.0, 0.0, 0.05),
                ),
            ).to_dict(),
            runner.BenchmarkCaseResult(
                name="case_b",
                pose=(0.0, 1.2, 0.0, 0.0, -0.5, 0.0, 0.0),
                controller="px4_position",
                takeoff_success=False,
                max_altitude_m=0.4,
                tracking_duration_s=0.0,
                passed=False,
                summary=VelocityTrackingSummary(),
                error="startup failed",
            ).to_dict(),
        ],
    }


def test_paired_pose_cases_group_am_and_px4_results() -> None:
    runner = _load_runner_module()
    cases = [
        {"name": "pose_a", "controller": "am", "pose": [1.0], "passed": True},
        {"name": "pose_a", "controller": "px4_position", "pose": [1.0], "passed": False},
        {"name": "pose_b", "controller": "am", "pose": [2.0], "passed": True},
    ]

    pairs = runner._paired_pose_cases(cases)

    assert [pair["name"] for pair in pairs] == ["pose_a", "pose_b"]
    assert pairs[0]["pose"] == [1.0]
    assert pairs[0]["controllers"]["am"]["passed"] is True
    assert pairs[0]["controllers"]["px4_position"]["passed"] is False
    assert "px4_position" not in pairs[1]["controllers"]


def test_pair_metric_delta_uses_am_minus_px4_and_handles_missing_controller() -> None:
    runner = _load_runner_module()
    pair = {
        "controllers": {
            "am": {
                "summary": {"rms_speed_error_norm_mps": 0.30},
                "arm_motion_summary": {"max_horizontal_offset_m": 0.45},
            },
            "px4_position": {
                "summary": {"rms_speed_error_norm_mps": 0.10},
                "arm_motion_summary": {"max_horizontal_offset_m": 0.15},
            },
        }
    }

    assert runner._pair_metric_delta(pair, "summary", "rms_speed_error_norm_mps") == pytest.approx(0.20)
    assert runner._pair_metric_delta(pair, "arm_motion_summary", "max_horizontal_offset_m") == pytest.approx(0.30)
    assert (
        runner._pair_metric_delta(
            {"controllers": {"am": pair["controllers"]["am"]}}, "summary", "rms_speed_error_norm_mps"
        )
        is None
    )


def test_paired_metric_delta_grid_collects_overview_metrics() -> None:
    runner = _load_runner_module()
    pair = {
        "name": "pose_a",
        "controllers": {
            "am": {
                "summary": {
                    "rms_speed_error_norm_mps": 0.30,
                    "max_abs_lateral_velocity_bias_mps": 0.20,
                    "rms_yaw_rate_error_radps": 0.08,
                },
                "arm_motion_summary": {
                    "max_horizontal_offset_m": 0.45,
                    "rms_horizontal_offset_m": 0.25,
                    "max_vertical_offset_m": 0.05,
                    "max_abs_roll_rad": 0.09,
                    "max_abs_pitch_rad": 0.02,
                    "max_abs_yaw_rad": 0.01,
                },
            },
            "px4_position": {
                "summary": {
                    "rms_speed_error_norm_mps": 0.10,
                    "max_abs_lateral_velocity_bias_mps": 0.40,
                    "rms_yaw_rate_error_radps": 0.03,
                },
                "arm_motion_summary": {
                    "max_horizontal_offset_m": 0.15,
                    "rms_horizontal_offset_m": 0.10,
                    "max_vertical_offset_m": 0.01,
                    "max_abs_roll_rad": 0.02,
                    "max_abs_pitch_rad": 0.05,
                    "max_abs_yaw_rad": 0.01,
                },
            },
        },
    }

    labels, matrix = runner._paired_metric_delta_grid([pair])

    assert labels == ["max XY", "RMS XY", "max Z", "attitude", "RMS speed", "lateral bias", "yaw rate"]
    assert matrix.shape == (7, 1)
    assert matrix[:, 0] == pytest.approx([0.30, 0.15, 0.04, 0.04, 0.20, -0.20, 0.05])


def test_x500_arm2x_pose_renderer_returns_rgb_image() -> None:
    runner = _load_runner_module()

    image = runner._render_x500_arm2x_pose(
        runner.default_arm_pose_cases()[0].pose,
        width=96,
        height=72,
    )

    assert image.shape == (72, 96, 3)
    assert image.dtype.name == "uint8"
    assert int(image.max()) > int(image.min())


def test_x500_arm2x_rendered_arm_poses_are_visually_distinct() -> None:
    runner = _load_runner_module()

    home = runner._render_x500_arm2x_pose(
        runner.default_arm_pose_cases()[0].pose,
        width=96,
        height=72,
    )
    forward = runner._render_x500_arm2x_pose(
        runner.default_arm_pose_cases()[-1].pose,
        width=96,
        height=72,
    )

    assert float(abs(home.astype("int16") - forward.astype("int16")).mean()) > 0.5


def test_default_arm_pose_cases_remain_kinematically_distinct() -> None:
    runner = _load_runner_module()
    wrist_points = [
        runner._x500_arm2x_pose_body_positions(case.pose)["link_5"] for case in runner.default_arm_pose_cases()
    ]

    unique_wrist_points = {tuple(round(value, 2) for value in point) for point in wrist_points}

    assert len(unique_wrist_points) >= 4


def test_x500_arm2x_pose_body_positions_include_gripper_endpoints() -> None:
    runner = _load_runner_module()

    points = runner._x500_arm2x_pose_body_positions(runner.default_arm_pose_cases()[1].pose)
    gripper_distance = math.sqrt(
        sum((points["gripper_left"][axis] - points["gripper_right"][axis]) ** 2 for axis in range(3))
    )

    assert gripper_distance > 0.01


def test_x500_arm2x_visual_rotor_mocaps_follow_physical_rotor_bodies() -> None:
    runner = _load_runner_module()
    mujoco = pytest.importorskip("mujoco")
    model = runner._x500_arm2x_model()
    data = mujoco.MjData(model)
    runner._set_x500_arm2x_pose(model, data, runner.default_arm_pose_cases()[0].pose)

    runner._initialize_x500_arm2x_visual_mocaps(model, data)

    for index in range(1, 5):
        physical_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{index}")
        visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"rotor_{index}_vis")
        assert physical_id >= 0
        assert visual_id >= 0
        assert int(model.body_mocapid[visual_id]) >= 0
        assert data.xpos[visual_id] == pytest.approx(data.xpos[physical_id], abs=1e-6)


def test_case_palette_uses_distinct_colors_for_passed_cases() -> None:
    runner = _load_runner_module()

    colors = runner._case_palette(10)

    assert colors == [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#9467bd",
        "#d62728",
        "#8c564b",
        "#17becf",
        "#7f7f7f",
        "#bcbd22",
        "#e377c2",
    ]
    assert len(set(colors)) == 10
    assert "#2f855a" not in colors


def test_pose_gallery_uses_multiple_rows_for_many_cases(tmp_path: Path) -> None:
    runner = _load_runner_module()
    rendered_images: list[Any] = []

    def fake_render(pose: object, *, width: int, height: int, color: str) -> object:
        import numpy as np

        return np.full((height, width, 3), 255, dtype=np.uint8)

    class FakeAxis:
        def __init__(self) -> None:
            self.transAxes = object()

        def set_title(self, title: str) -> None:
            pass

        def set_axis_off(self) -> None:
            pass

        def imshow(self, image: object) -> None:
            rendered_images.append(image)

        def text(self, x: float, y: float, *args: object, **kwargs: object) -> None:
            raise AssertionError("case labels should be rendered inside the gallery raster")

    cases = [{"name": case.name, "pose": case.pose} for case in runner.default_arm_pose_cases()]
    with patch.object(runner, "_render_x500_arm2x_pose", fake_render):
        runner._draw_x500_arm2x_pose_gallery_panel(
            FakeAxis(),
            cases,
            [case["name"] for case in cases],
            runner._case_palette(len(cases)),
        )

    assert len(rendered_images) == 1
    gallery = rendered_images[0]
    assert gallery.shape[0] == 2 * (190 + 44) + 12
    assert gallery.shape[1] == 5 * 260 + 4 * 12


def test_write_benchmark_report_image_uses_one_pose_gallery_tile_per_pose_pair(tmp_path: Path) -> None:
    runner = _load_runner_module()
    seen_names: list[str] = []
    seen_colors: list[str] = []
    seen_status_texts: list[str] = []

    def fake_draw(
        ax: Any,
        cases: object,
        names: Sequence[str],
        colors: Sequence[str],
        status_texts: Sequence[str] | None = None,
    ) -> None:
        seen_names.extend(list(names))
        seen_colors.extend(list(colors))
        if status_texts is not None:
            seen_status_texts.extend(list(status_texts))
        ax.set_title("B  fake MuJoCo arm gallery")

    pose_cases: list[dict[str, object]] = []
    for index, pose_case in enumerate(runner.default_arm_pose_cases()):
        for controller in ("am", "px4_position"):
            pose_cases.append(
                runner.BenchmarkCaseResult(
                    name=pose_case.name,
                    pose=pose_case.pose,
                    controller=controller,
                    takeoff_success=True,
                    max_altitude_m=1.5,
                    tracking_duration_s=2.0,
                    passed=True,
                    summary=VelocityTrackingSummary(sample_count=2),
                ).to_dict()
            )
    result = {"asset": "x500_arm2x", "passed": True, "profile": {"cycles": 1}, "thresholds": {}, "cases": pose_cases}

    with patch.object(runner, "_draw_x500_arm2x_pose_gallery_panel", fake_draw):
        runner.write_benchmark_report_image(result, tmp_path / "report.png")

    assert seen_names == [case.name for case in runner.default_arm_pose_cases()]
    assert seen_colors == runner._case_palette(10)
    assert len(seen_status_texts) == 10
    assert all("AM PASS" in text and "PX4 PASS" in text for text in seen_status_texts)


def test_write_benchmark_report_image_places_overview_and_gallery_side_by_side(tmp_path: Path) -> None:
    runner = _load_runner_module()
    layout: dict[str, tuple[tuple[int, int], tuple[int, int]]] = {}

    def span_of(ax: Any) -> tuple[tuple[int, int], tuple[int, int]]:
        spec = ax.get_subplotspec()
        return (
            (int(spec.rowspan.start), int(spec.rowspan.stop)),
            (int(spec.colspan.start), int(spec.colspan.stop)),
        )

    def fake_overview(ax: Any, pairs: object) -> None:
        layout["overview"] = span_of(ax)
        ax.set_title("A fake overview")

    def fake_gallery(ax: Any, cases: object, names: object, colors: object, status_texts: object | None = None) -> None:
        layout["gallery"] = span_of(ax)
        ax.set_title("B fake gallery")

    with (
        patch.object(runner, "_draw_paired_overview_panel", fake_overview, create=True),
        patch.object(runner, "_draw_x500_arm2x_pose_gallery_panel", fake_gallery),
    ):
        runner.write_benchmark_report_image(_sample_report_result(runner), tmp_path / "report.png")

    assert layout["overview"][0] == layout["gallery"][0] == (0, 1)
    assert layout["overview"][1][1] <= layout["gallery"][1][0]


def test_write_benchmark_report_image_survives_pose_gallery_failure(tmp_path: Path) -> None:
    runner = _load_runner_module()
    output_path = tmp_path / "report.png"

    def fail_gallery(ax: object, cases: object, names: object, colors: object) -> None:
        raise RuntimeError("renderer unavailable")

    with patch.object(runner, "_draw_x500_arm2x_pose_gallery_panel", fail_gallery):
        runner.write_benchmark_report_image(_sample_report_result(runner), output_path)

    assert output_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output_path.stat().st_size > 1000


def test_write_benchmark_report_image_creates_png(tmp_path: Path) -> None:
    runner = _load_runner_module()
    output_path = tmp_path / "report.png"

    runner.write_benchmark_report_image(_sample_report_result(runner), output_path)

    assert output_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output_path.stat().st_size > 1000


def test_write_raw_output_saves_summary_and_case_csvs(tmp_path: Path) -> None:
    runner = _load_runner_module()
    output_dir = tmp_path / "raw"
    result = {
        "asset": "x500_arm2x",
        "passed": True,
        "profile": {"cycles": 1},
        "thresholds": {},
        "cases": [
            runner.BenchmarkCaseResult(
                name="case_a",
                pose=(0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0),
                controller="am",
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=2.0,
                passed=True,
                summary=VelocityTrackingSummary(sample_count=2),
                arm_motion_summary=runner.ArmMotionSummary(sample_count=1),
                arm_motion_samples=(
                    runner.ArmMotionSample(
                        0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                    ),
                ),
                velocity_tracking_samples=(
                    runner.VelocityTrackingSample(0.0, "forward", 0.4, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.1),
                ),
            ).to_dict()
        ],
    }

    runner.write_raw_output(result, output_dir)

    assert (output_dir / "summary.json").exists()
    case_dir = output_dir / "cases" / "case_a_am"
    assert (case_dir / "metadata.json").exists()
    arm_motion_csv = (case_dir / "arm_motion.csv").read_text(encoding="utf-8")
    assert "elapsed_s" in arm_motion_csv
    assert "roll_rad" in arm_motion_csv
    assert "pitch_rad" in arm_motion_csv
    assert "yaw_rad" in arm_motion_csv
    assert "segment_name" in (case_dir / "velocity_tracking.csv").read_text(encoding="utf-8")
    assert "arm_motion_samples" not in (output_dir / "summary.json").read_text(encoding="utf-8")


def test_write_raw_output_removes_stale_case_directories(tmp_path: Path) -> None:
    runner = _load_runner_module()
    output_dir = tmp_path / "raw"
    stale_dir = output_dir / "cases" / "old_case"
    stale_dir.mkdir(parents=True)
    (stale_dir / "metadata.json").write_text("stale", encoding="utf-8")
    result = {
        "asset": "x500_arm2x",
        "passed": True,
        "profile": {"cycles": 1},
        "thresholds": {},
        "cases": [
            runner.BenchmarkCaseResult(
                name="case_a",
                pose=(0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0),
                controller="am",
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=2.0,
                passed=True,
                summary=VelocityTrackingSummary(sample_count=2),
            ).to_dict()
        ],
    }

    runner.write_raw_output(result, output_dir)

    assert not stale_dir.exists()
    assert (output_dir / "cases" / "case_a_am" / "metadata.json").exists()


def test_main_writes_image_and_optional_json_without_printing_json(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runner = _load_runner_module()
    image_path = tmp_path / "report.png"
    json_path = tmp_path / "result.json"
    fake_result = {
        "asset": "x500_arm2x",
        "passed": True,
        "profile": {"cycles": 1},
        "thresholds": {},
        "cases": [
            runner.BenchmarkCaseResult(
                name="case_a",
                pose=(0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0),
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=42.0,
                passed=True,
                summary=VelocityTrackingSummary(sample_count=10),
            ).to_dict()
        ],
    }

    def fake_write_image(result: object, output_path: Path) -> None:
        output_path.write_bytes(b"\x89PNG\r\n\x1a\nfake")

    with patch.object(runner, "run_benchmark", lambda cases, runtime_config: fake_result):
        with patch.object(runner, "write_benchmark_report_image", fake_write_image):
            exit_code = runner.main(
                [
                    "--case",
                    "home_folded",
                    "--output",
                    str(image_path),
                    "--json-output",
                    str(json_path),
                ]
            )

    captured = capsys.readouterr()
    assert exit_code == 0
    assert image_path.read_bytes().startswith(b"\x89PNG")
    assert json_path.exists()
    assert json.loads(json_path.read_text())["passed"] is True
    assert "{" not in captured.out
    assert "report written to" in captured.out
    assert "json written to" in captured.out


def test_main_auto_jobs_clamps_to_available_px4_instance_slots(tmp_path: Path) -> None:
    runner = _load_runner_module()
    seen_jobs: list[int] = []
    fake_result = {
        "asset": "x500_arm2x",
        "passed": True,
        "profile": {"cycles": 1},
        "thresholds": {},
        "cases": [],
    }

    def fake_run_benchmark(cases: object, runtime_config: Any) -> dict[str, object]:
        seen_jobs.append(runtime_config.jobs)
        return fake_result

    with patch.object(runner, "run_benchmark", fake_run_benchmark):
        exit_code = runner.main([])

    assert exit_code == 0
    assert seen_jobs == [10]


def test_main_returns_zero_for_completed_failed_benchmark_by_default(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runner = _load_runner_module()
    image_path = tmp_path / "failed_report.png"
    fake_result = {
        "asset": "x500_arm2x",
        "passed": False,
        "profile": {"cycles": 1},
        "thresholds": {},
        "cases": [
            runner.BenchmarkCaseResult(
                name="forward_low",
                pose=(0.0, 2.05, 0.0, -1.35, -1.05, -0.02, 0.02),
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=42.0,
                passed=False,
                summary=VelocityTrackingSummary(sample_count=10),
                error=None,
            ).to_dict()
        ],
    }

    def fake_write_image(result: object, output_path: Path) -> None:
        output_path.write_bytes(b"\x89PNG\r\n\x1a\nfake")

    with patch.object(runner, "run_benchmark", lambda cases, runtime_config: fake_result):
        with patch.object(runner, "write_benchmark_report_image", fake_write_image):
            exit_code = runner.main(["--output", str(image_path)])

    captured = capsys.readouterr()
    assert exit_code == 0
    assert "result FAIL (0/1 cases passed)" in captured.out
    assert "report written to" in captured.out


def test_main_strict_exit_code_returns_one_for_failed_benchmark(tmp_path: Path) -> None:
    runner = _load_runner_module()
    image_path = tmp_path / "failed_report.png"
    fake_result = {
        "asset": "x500_arm2x",
        "passed": False,
        "profile": {"cycles": 1},
        "thresholds": {},
        "cases": [
            runner.BenchmarkCaseResult(
                name="forward_low",
                pose=(0.0, 2.05, 0.0, -1.35, -1.05, -0.02, 0.02),
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=42.0,
                passed=False,
                summary=VelocityTrackingSummary(sample_count=10),
            ).to_dict()
        ],
    }

    def fake_write_image(result: object, output_path: Path) -> None:
        output_path.write_bytes(b"\x89PNG\r\n\x1a\nfake")

    with patch.object(runner, "run_benchmark", lambda cases, runtime_config: fake_result):
        with patch.object(runner, "write_benchmark_report_image", fake_write_image):
            exit_code = runner.main(["--output", str(image_path), "--strict-exit-code"])

    assert exit_code == 1


def test_managed_stack_redirects_process_logs_to_case_log_dir_by_default(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runner = _load_runner_module()
    case = runner.ArmPoseCase("quiet_case", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0))

    class LoggingStack(runner.ManagedBenchmarkStack):  # type: ignore[name-defined]
        def _write_config(self, root: Path) -> Path:
            return root / "default.toml"

        def _start_processes(self, config_path: Path) -> None:
            process = self._popen(
                "dummy",
                [
                    sys.executable,
                    "-c",
                    "import sys; print('child stdout'); print('child stderr', file=sys.stderr)",
                ],
            )
            assert process.wait(timeout=5.0) == 0

    config = runner.BenchmarkRuntimeConfig(process_log_dir=str(tmp_path), verbose_process_logs=False)

    with LoggingStack(case, config):
        pass

    captured = capsys.readouterr()
    log_path = tmp_path / "quiet_case" / "dummy.log"
    assert "child stdout" not in captured.err
    assert "child stderr" not in captured.err
    assert "process logs:" in captured.err
    assert "child stdout" in log_path.read_text(encoding="utf-8")
    assert "child stderr" in log_path.read_text(encoding="utf-8")


def test_managed_stack_headless_uses_arm_command_endpoint_not_fixed_pose(tmp_path: Path) -> None:
    runner = _load_runner_module()
    case = runner.ArmPoseCase("dynamic_case", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0))
    commands: list[tuple[str, list[str], dict[str, str] | None]] = []

    class CapturingStack(runner.ManagedBenchmarkStack):  # type: ignore[name-defined]
        def _write_config(self, root: Path) -> Path:
            config_path = root / "default.toml"
            mujoco_dir = root / "mujoco"
            mujoco_dir.mkdir(parents=True)
            (mujoco_dir / "x500_arm2x.toml").write_text("[params]\n", encoding="utf-8")
            config_text = (
                "[basic]\n"
                "sim_type='mujoco'\n"
                "env_type='am'\n"
                "scene_name='default'\n"
                "asset_name='x500_arm2x'\n"
                "benchmark='multirotor'\n"
            )
            config_path.write_text(config_text, encoding="utf-8")
            return config_path

        def _popen(self, name: str, command: list[str], *, cwd: str | None = None, env: dict[str, str] | None = None):
            commands.append((name, command, env))

            class FakeProcess:
                returncode = 0

                def poll(self) -> int | None:
                    return None

                def wait(self, timeout: float | None = None) -> int:
                    return 0

                def terminate(self) -> None:
                    pass

                def kill(self) -> None:
                    pass

            process = FakeProcess()
            self._processes.append((name, process))
            return process

    fake_launch_common: Any = types.ModuleType("acesim_ros2.launch_common")
    fake_launch_common.detect_acesim_root = lambda: Path(__file__).resolve().parents[1] / "acesim"
    fake_launch_common.load_px4_repo_path = lambda _override: str(tmp_path / "px4")
    fake_launch_common.build_px4_additional_env = lambda _loader: {"PX4_SYS_AUTOSTART": "10016"}
    fake_launch_common.bridge_config_path = lambda: "bridges.yaml"
    fake_launch_common.load_bridge_entries = lambda _path: [
        {"name": "simulation_clock", "enabled": True, "endpoint": "tcp://127.0.0.1:5600"},
        {"name": "arm_state", "enabled": True, "endpoint": "tcp://127.0.0.1:5603"},
    ]
    fake_launch_common.resolve_bridge_host = lambda _mode: "127.0.0.1"
    fake_launch_common.build_graceful_shutdown_command = lambda command, **_kwargs: ["bash", "-lc", command]
    fake_launch_common.build_python_module_run_command = (
        lambda package, executable, additional_env=None, extra_args=None: json.dumps(
            {
                "package": package,
                "executable": executable,
                "env": additional_env or {},
                "args": extra_args or [],
            }
        )
    )
    fake_launch_common.build_px4_post_start_command = lambda _params: ["post"]
    fake_pkg: Any = types.ModuleType("acesim_ros2")
    fake_pkg.launch_common = fake_launch_common

    class FakeConfigLoader:
        def __init__(self, path: Path) -> None:
            self.path = path

        def get_asset_params(self) -> dict[str, object]:
            return {
                "rotor_direction": [1, 1, -1, -1],
                "motor_constant": 1,
                "moment_constant": 1,
                "rotor_drag_coeff": 1,
                "rolling_moment_coeff": 1,
                "rotor_radius": 1,
                "time_constant_up": 1,
                "time_constant_down": 1,
                "max_rot_velocity": 1,
                "max_relative_airspeed_mps": 1,
            }

    class FakePX4SensorParams:
        @classmethod
        def from_asset_params(cls, *args: object, **kwargs: object) -> object:
            return object()

    px4_bin = tmp_path / "px4" / "build" / "px4_sitl_default" / "bin"
    px4_bin.mkdir(parents=True)
    (px4_bin / "px4").write_text("", encoding="utf-8")

    with patch.dict(sys.modules, {"acesim_ros2": fake_pkg, "acesim_ros2.launch_common": fake_launch_common}):
        with patch("acesim.config.config_loader.ConfigLoader", FakeConfigLoader):
            with patch("acesim.utils.px4_transport.PX4SensorParams", FakePX4SensorParams):
                with patch.object(runner.time, "sleep", lambda _duration: None):
                    with CapturingStack(case, runner.BenchmarkRuntimeConfig(process_log_dir=str(tmp_path))):
                        pass

    headless = next(item for item in commands if item[0] == "headless")
    command_text = " ".join(headless[1])
    assert "ACESIM_FIXED_ARM_POSE" not in command_text
    assert "ACESIM_ARM_COMMAND_ENDPOINT" in command_text
    assert "ACESIM_ARM_COMMAND_ONLY" in command_text
    assert "--real-time-rate" not in command_text


def test_managed_stack_sets_px4_uxrce_port_env_to_match_agent(tmp_path: Path) -> None:
    runner = _load_runner_module()
    isolation = runner._default_isolation(1, runner.BenchmarkRuntimeConfig())
    case = runner.ArmPoseCase("xrce_case", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0))
    commands: list[tuple[str, list[str], dict[str, str] | None]] = []

    class CapturingStack(runner.ManagedBenchmarkStack):  # type: ignore[name-defined]
        def _write_config(self, root: Path) -> Path:
            config_path = root / "default.toml"
            mujoco_dir = root / "mujoco"
            mujoco_dir.mkdir(parents=True)
            (mujoco_dir / "x500_arm2x.toml").write_text("[params]\n", encoding="utf-8")
            config_text = (
                "[basic]\n"
                "sim_type='mujoco'\n"
                "env_type='am'\n"
                "scene_name='default'\n"
                "asset_name='x500_arm2x'\n"
                "benchmark='multirotor'\n"
            )
            config_path.write_text(config_text, encoding="utf-8")
            return config_path

        def _popen(self, name: str, command: list[str], *, cwd: str | None = None, env: dict[str, str] | None = None):
            commands.append((name, command, env))

            class FakeProcess:
                returncode = 0

                def poll(self) -> int | None:
                    return None

                def wait(self, timeout: float | None = None) -> int:
                    return 0

                def terminate(self) -> None:
                    pass

                def kill(self) -> None:
                    pass

            process = FakeProcess()
            self._processes.append((name, process))
            return process

    fake_launch_common: Any = types.ModuleType("acesim_ros2.launch_common")
    fake_launch_common.detect_acesim_root = lambda: Path(__file__).resolve().parents[1] / "acesim"
    fake_launch_common.load_px4_repo_path = lambda _override: str(tmp_path / "px4")
    fake_launch_common.build_px4_additional_env = lambda _loader: {"PX4_SYS_AUTOSTART": "10016"}
    fake_launch_common.bridge_config_path = lambda: "bridges.yaml"
    fake_launch_common.load_bridge_entries = lambda _path: [
        {"name": "simulation_clock", "enabled": True, "endpoint": "tcp://127.0.0.1:5600"},
        {"name": "arm_state", "enabled": True, "endpoint": "tcp://127.0.0.1:5603"},
    ]
    fake_launch_common.resolve_bridge_host = lambda _mode: "127.0.0.1"
    fake_launch_common.build_graceful_shutdown_command = lambda command, **_kwargs: ["bash", "-lc", command]
    fake_launch_common.build_python_module_run_command = (
        lambda package, executable, additional_env=None, extra_args=None: json.dumps(
            {
                "package": package,
                "executable": executable,
                "env": additional_env or {},
                "args": extra_args or [],
            }
        )
    )
    fake_launch_common.build_px4_post_start_command = lambda _params: ["post"]
    fake_pkg: Any = types.ModuleType("acesim_ros2")
    fake_pkg.launch_common = fake_launch_common

    class FakeConfigLoader:
        def __init__(self, path: Path) -> None:
            self.path = path

        def get_asset_params(self) -> dict[str, object]:
            return {
                "rotor_direction": [1, 1, -1, -1],
                "motor_constant": 1,
                "moment_constant": 1,
                "rotor_drag_coeff": 1,
                "rolling_moment_coeff": 1,
                "rotor_radius": 1,
                "time_constant_up": 1,
                "time_constant_down": 1,
                "max_rot_velocity": 1,
                "max_relative_airspeed_mps": 1,
            }

    class FakePX4SensorParams:
        @classmethod
        def from_asset_params(cls, *args: object, **kwargs: object) -> object:
            return object()

    px4_bin = tmp_path / "px4" / "build" / "px4_sitl_default" / "bin"
    px4_bin.mkdir(parents=True)
    (px4_bin / "px4").write_text("", encoding="utf-8")

    with patch.dict(sys.modules, {"acesim_ros2": fake_pkg, "acesim_ros2.launch_common": fake_launch_common}):
        with patch("acesim.config.config_loader.ConfigLoader", FakeConfigLoader):
            with patch("acesim.utils.px4_transport.PX4SensorParams", FakePX4SensorParams):
                with patch.object(runner.time, "sleep", lambda _duration: None):
                    with CapturingStack(
                        case,
                        runner.BenchmarkRuntimeConfig(process_log_dir=str(tmp_path)),
                        isolation,
                    ):
                        pass

    microxrce = next(item for item in commands if item[0] == "microxrce")
    px4 = next(item for item in commands if item[0] == "px4")
    assert f"MicroXRCEAgent udp4 -p {isolation.xrce_port}" in " ".join(microxrce[1])
    assert px4[2] is not None
    assert px4[2]["PX4_PARAM_UXRCE_DDS_PRT"] == str(isolation.xrce_port)
    assert px4[2]["PX4_UXRCE_DDS_PORT"] == str(isolation.xrce_port)
    assert px4[2]["PX4_UXRCE_DDS_NS"] == ""


def test_run_case_retries_transient_post_start_failure_once() -> None:
    runner = _load_runner_module()
    calls: list[str] = []
    fake_rclpy: Any = types.ModuleType("rclpy")
    fake_rclpy.init = lambda args=None: calls.append("init")
    fake_rclpy.shutdown = lambda: calls.append("shutdown")

    class FakeStack:
        attempts = 0

        def __init__(self, case: object, runtime_config: object, isolation: object | None = None) -> None:
            pass

        def __enter__(self) -> "FakeStack":
            FakeStack.attempts += 1
            calls.append(f"stack_enter:{FakeStack.attempts}")
            if FakeStack.attempts == 1:
                raise RuntimeError("PX4 post-start setup failed with exit code 1")
            return self

        def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
            calls.append("stack_exit")

    class FakeController:
        def __init__(self, runtime_config: object, isolation: object | None = None) -> None:
            calls.append("controller")

        def run(self, case: object) -> object:
            calls.append("run")
            return runner.BenchmarkCaseResult(
                name="home_folded",
                pose=runner.default_arm_pose_cases()[0].pose,
                takeoff_success=True,
                max_altitude_m=1.5,
                tracking_duration_s=1.0,
                passed=True,
                summary=VelocityTrackingSummary(sample_count=1),
            )

        def close(self) -> None:
            calls.append("close")

    with patch.dict(sys.modules, {"rclpy": fake_rclpy}):
        with patch.object(runner, "ManagedBenchmarkStack", FakeStack):
            with patch.object(runner, "X500Arm2xBenchmarkController", FakeController):
                result = runner.run_case(
                    runner.default_arm_pose_cases()[0],
                    runner.BenchmarkRuntimeConfig(case_start_attempts=2),
                )

    assert result.passed is True
    assert FakeStack.attempts == 2
    assert calls == ["stack_enter:1", "stack_enter:2", "init", "controller", "run", "close", "shutdown", "stack_exit"]


def test_run_case_shuts_down_rclpy_when_controller_creation_fails() -> None:
    runner = _load_runner_module()
    calls: list[str] = []
    fake_rclpy: Any = types.ModuleType("rclpy")
    fake_rclpy.init = lambda args=None: calls.append("init")
    fake_rclpy.shutdown = lambda: calls.append("shutdown")

    class FakeStack:
        def __init__(self, case: object, runtime_config: object, isolation: object | None = None) -> None:
            pass

        def __enter__(self) -> "FakeStack":
            return self

        def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
            calls.append("stack_exit")

    def fail_controller(runtime_config: object, isolation: object | None = None) -> object:
        calls.append("controller")
        raise RuntimeError("controller failed")

    with patch.dict(sys.modules, {"rclpy": fake_rclpy}):
        with patch.object(runner, "ManagedBenchmarkStack", FakeStack):
            with patch.object(runner, "X500Arm2xBenchmarkController", fail_controller):
                result = runner.run_case(runner.default_arm_pose_cases()[0], runner.BenchmarkRuntimeConfig())

    assert result.passed is False
    assert result.error == "controller failed"
    assert calls == ["init", "controller", "shutdown", "stack_exit"]


def test_managed_stack_cleans_processes_when_enter_fails() -> None:
    runner = _load_runner_module()
    calls: list[str] = []

    class FakeProcess:
        def __init__(self) -> None:
            self._running = True

        def poll(self) -> int | None:
            return None if self._running else 0

        def terminate(self) -> None:
            calls.append("terminate")
            self._running = False

        def wait(self, timeout: float | None = None) -> int:
            calls.append(f"wait:{timeout}")
            return 0

        def kill(self) -> None:
            calls.append("kill")
            self._running = False

    class FailingStack(runner.ManagedBenchmarkStack):  # type: ignore[name-defined]
        def _write_config(self, root: Path) -> Path:
            return root / "default.toml"

        def _start_processes(self, config_path: Path) -> None:
            self._processes.append(("started", FakeProcess()))
            raise RuntimeError("startup failed")

    stack = FailingStack(runner.default_arm_pose_cases()[0], runner.BenchmarkRuntimeConfig())

    with pytest.raises(RuntimeError, match="startup failed"):
        stack.__enter__()

    assert calls == ["terminate", "wait:5.0"]


def test_controller_attempts_landing_and_disarm_after_armed_failure() -> None:
    runner = _load_runner_module()
    calls: list[str] = []

    class FakeMav:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = types.SimpleNamespace(
                heartbeat_send=lambda *args: calls.append("send_heartbeat"),
                command_long_send=lambda *args: calls.append(f"mode:{args[2]}"),
            )

        def wait_heartbeat(self, timeout: float | None = None) -> None:
            calls.append("heartbeat")

        def close(self) -> None:
            calls.append("close")

    class FakeMavUtil:
        mavlink = types.SimpleNamespace(
            MAV_CMD_DO_SET_MODE=176,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_TYPE_GENERIC=0,
            MAV_AUTOPILOT_INVALID=8,
        )

        @staticmethod
        def mavlink_connection(
            url: str,
            source_system: int,
            source_component: int,
            autoreconnect: bool = False,
        ) -> FakeMav:
            calls.append(f"connect:{source_system}:{source_component}:{autoreconnect}")
            return FakeMav()

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig()

        def _wait_for_local_position(self, *, timeout_s: float) -> None:
            calls.append("wait_local_position")

        def _wait_for_global_position(self, *, timeout_s: float) -> None:
            calls.append("wait_global_position")

        def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
            calls.append(f"publish_for:{command.segment_name}")

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            calls.append(f"publish:{command.segment_name}")

        def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
            calls.append("takeoff_target")
            return (math.nan, math.nan, -1.5)

        def _takeoff_target_global(self) -> tuple[float, float, float]:
            calls.append("takeoff_global")
            return (39.1, 116.2, 51.5)

        def _takeoff(self, position_ned: tuple[float, float, float]) -> tuple[bool, float]:
            calls.append("takeoff")
            raise RuntimeError("tracking failure")

        def _land_and_disarm(self, mav: object, mavlink: object) -> None:
            calls.append("land_disarm")

        def _altitude_m(self) -> float:
            return 0.7

    def fake_ack(
        mav: object,
        command: int,
        *,
        timeout_s: float,
        tick: object | None = None,
        resend: object | None = None,
    ) -> None:
        calls.append(f"ack:{command}")

    def fake_arm(mav: object, arm: bool, *, mavlink: object) -> None:
        calls.append(f"arm:{arm}")

    fake_pymavlink: Any = types.ModuleType("pymavlink")
    fake_pymavlink.mavutil = FakeMavUtil
    with patch.dict(sys.modules, {"pymavlink": fake_pymavlink}):
        with patch.object(runner, "_wait_command_ack", fake_ack):
            with patch.object(runner, "_send_arm_command", fake_arm):
                result = runner.X500Arm2xBenchmarkController.run(FakeController(), runner.default_arm_pose_cases()[0])

    assert result.passed is False
    assert result.error == "tracking failure"
    assert "connect:250:190:True" in calls
    assert "arm:True" in calls
    assert "land_disarm" in calls
    assert calls[-1] == "close"


def test_controller_auto_takes_off_before_am_profile() -> None:
    runner = _load_runner_module()
    calls: list[str] = []

    class FakeMav:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = types.SimpleNamespace(
                heartbeat_send=lambda *args: calls.append("send_heartbeat"),
                command_long_send=self._command_long_send,
            )

        def _command_long_send(self, *args: object) -> None:
            command = int(cast(Any, args[2]))
            if command == 176:
                calls.append(f"mode_sub:{args[6]:.1f}")
            elif command == 22:
                calls.append("nav_takeoff")

        def wait_heartbeat(self, timeout: float | None = None) -> None:
            calls.append("heartbeat")

        def close(self) -> None:
            calls.append("close")

    class FakeMavUtil:
        mavlink = types.SimpleNamespace(
            MAV_CMD_DO_SET_MODE=176,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_TYPE_GENERIC=0,
            MAV_AUTOPILOT_INVALID=8,
        )

        @staticmethod
        def mavlink_connection(
            url: str,
            source_system: int,
            source_component: int,
            autoreconnect: bool = False,
        ) -> FakeMav:
            calls.append("connect")
            return FakeMav()

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig()

        def _wait_for_local_position(self, *, timeout_s: float) -> None:
            calls.append("wait_local_position")

        def _wait_for_global_position(self, *, timeout_s: float) -> None:
            calls.append("wait_global_position")

        def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
            calls.append(f"publish_for:{command.segment_name}")

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            calls.append(f"publish:{command.segment_name}")

        def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
            calls.append("takeoff_target")
            return (math.nan, math.nan, -1.5)

        def _takeoff_target_global(self) -> tuple[float, float, float]:
            calls.append("takeoff_global")
            return (39.1, 116.2, 51.5)

        def _takeoff(self, position_ned: tuple[float, float, float]) -> tuple[bool, float]:
            calls.append("takeoff")
            return True, 1.5

        def _wait_for_settled_local_position(
            self,
            *,
            duration_s: float,
            timeout_s: float,
            position_ned: tuple[float, float, float] | None = None,
            setpoint_tick: object | None = None,
        ) -> None:
            calls.append(f"settle:{duration_s:g}")

        def _move_arm_and_record_base_motion(
            self,
            case: object,
            hold_command: VelocityTrackingCommand,
        ) -> list[object]:
            calls.append(f"arm_motion:{hold_command.segment_name}")
            return []

        def _run_profile(self, profile: object) -> tuple[object, float, tuple[object, ...]]:
            calls.append("profile")
            return runner.VelocityTrackingMetrics(), 1.0, ()

        def _land_and_disarm(self, mav: object, mavlink: object) -> None:
            calls.append("land_disarm")

        def _altitude_m(self) -> float:
            return 1.5

    def fake_ack(
        mav: object,
        command: int,
        *,
        timeout_s: float,
        tick: object | None = None,
        resend: object | None = None,
    ) -> None:
        calls.append(f"ack:{command}")

    def fake_arm(mav: object, arm: bool, *, mavlink: object) -> None:
        calls.append(f"arm:{arm}")

    fake_pymavlink: Any = types.ModuleType("pymavlink")
    fake_pymavlink.mavutil = FakeMavUtil
    with patch.dict(sys.modules, {"pymavlink": fake_pymavlink}):
        with patch.object(runner, "_wait_command_ack", fake_ack):
            with patch.object(runner, "_send_arm_command", fake_arm):
                result = runner.X500Arm2xBenchmarkController.run(FakeController(), runner.default_arm_pose_cases()[0])

    assert result.takeoff_success is True
    assert [call for call in calls if call.startswith("mode_sub:")] == ["mode_sub:1.0"]
    assert calls.index("arm:True") < calls.index("nav_takeoff") < calls.index("takeoff")
    assert calls.index("takeoff") < calls.index("publish_for:hold")
    assert calls.index("publish_for:hold") < calls.index("mode_sub:1.0")
    assert calls.index("mode_sub:1.0") < calls.index("arm_motion:hold")
    assert calls.index("arm_motion:hold") < calls.index("profile")
    assert calls.index("mode_sub:1.0") < calls.index("profile")
    assert "land_disarm" in calls


def test_controller_waits_for_base_to_settle_after_arm_motion_under_am_offboard() -> None:
    runner = _load_runner_module()
    calls: list[str] = []
    config = runner.BenchmarkRuntimeConfig(post_arm_motion_settle_s=2.0)

    class FakeMav:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = types.SimpleNamespace(
                heartbeat_send=lambda *args: None,
                command_long_send=self._command_long_send,
            )

        def _command_long_send(self, *args: object) -> None:
            if int(cast(Any, args[2])) == 176:
                calls.append(f"mode:{args[6]:.1f}")

        def wait_heartbeat(self, timeout: float | None = None) -> None:
            pass

        def close(self) -> None:
            pass

    class FakeMavUtil:
        mavlink = types.SimpleNamespace(
            MAV_CMD_DO_SET_MODE=176,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_TYPE_GENERIC=0,
            MAV_AUTOPILOT_INVALID=8,
        )

        @staticmethod
        def mavlink_connection(
            url: str,
            source_system: int,
            source_component: int,
            autoreconnect: bool = False,
        ) -> FakeMav:
            return FakeMav()

    class FakeController:
        _config = config

        def _wait_for_local_position(self, *, timeout_s: float) -> None:
            pass

        def _wait_for_global_position(self, *, timeout_s: float) -> None:
            pass

        def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
            return (math.nan, math.nan, -1.5)

        def _takeoff_target_global(self) -> tuple[float, float, float]:
            return (39.1, 116.2, 51.5)

        def _takeoff(self, position_ned: tuple[float, float, float]) -> tuple[bool, float]:
            return True, 1.5

        def _wait_for_settled_local_position(
            self,
            *,
            duration_s: float,
            timeout_s: float,
            position_ned: tuple[float, float, float] | None = None,
            setpoint_tick: object | None = None,
        ) -> None:
            z = "none" if position_ned is None else f"{position_ned[2]:.1f}"
            calls.append(f"settle:{duration_s:g}:{timeout_s:g}:{z}:{setpoint_tick is not None}")

        def _publish_position_setpoint(self, position_ned: tuple[float, float, float]) -> None:
            calls.append(f"position_setpoint:{position_ned[2]:.1f}")

        def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
            calls.append(f"publish_for:{command.segment_name}:{duration_s:g}")

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            calls.append(f"publish:{command.segment_name}")

        def _move_arm_and_record_base_motion(
            self,
            case: object,
            hold_command: VelocityTrackingCommand,
        ) -> list[object]:
            calls.append(f"arm_motion:{hold_command.segment_name}")
            return []

        def _run_profile(self, profile: object) -> tuple[object, float, tuple[object, ...]]:
            calls.append("profile")
            return runner.VelocityTrackingMetrics(), 1.0, ()

        def _land_and_disarm(self, mav: object, mavlink: object) -> None:
            pass

        def _altitude_m(self) -> float:
            return 1.5

    fake_pymavlink: Any = types.ModuleType("pymavlink")
    fake_pymavlink.mavutil = FakeMavUtil
    with patch.dict(sys.modules, {"pymavlink": fake_pymavlink}):
        with patch.object(runner, "_wait_command_ack", lambda *args, **kwargs: None):
            with patch.object(runner, "_send_arm_command", lambda *args, **kwargs: None):
                result = runner.X500Arm2xBenchmarkController.run(FakeController(), runner.default_arm_pose_cases()[1])

    assert result.takeoff_success is True
    assert "settle:2:12:none:True" in calls
    assert calls.index("mode:1.0") < calls.index("arm_motion:hold") < calls.index("settle:2:12:none:True")
    assert "position_setpoint:-1.5" not in calls


def test_px4_position_variant_moves_arm_under_position_hold_then_runs_velocity_profile() -> None:
    runner = _load_runner_module()
    calls: list[str] = []
    case = runner.ControllerCase(
        runner.ArmPoseCase("px4_case", (0.0, 1.0, 0.0, 0.0, -0.5, 0.0, 0.0)),
        "px4_position",
    )

    class FakeMav:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = types.SimpleNamespace(
                heartbeat_send=lambda *args: None,
                command_long_send=self._command_long_send,
            )

        def _command_long_send(self, *args: object) -> None:
            if int(cast(Any, args[2])) == 176:
                calls.append(f"mode:{args[6]:.1f}")

        def wait_heartbeat(self, timeout: float | None = None) -> None:
            pass

        def close(self) -> None:
            pass

    class FakeMavUtil:
        mavlink = types.SimpleNamespace(
            MAV_CMD_DO_SET_MODE=176,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_TYPE_GENERIC=0,
            MAV_AUTOPILOT_INVALID=8,
        )

        @staticmethod
        def mavlink_connection(
            url: str,
            source_system: int,
            source_component: int,
            autoreconnect: bool = False,
        ) -> FakeMav:
            return FakeMav()

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig()

        def _wait_for_local_position(self, *, timeout_s: float) -> None:
            pass

        def _wait_for_global_position(self, *, timeout_s: float) -> None:
            pass

        def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
            return (math.nan, math.nan, -1.5)

        def _takeoff_target_global(self) -> tuple[float, float, float]:
            return (39.1, 116.2, 51.5)

        def _takeoff(self, position_ned: tuple[float, float, float]) -> tuple[bool, float]:
            calls.append("takeoff")
            return True, 1.5

        def _current_position_ned(self) -> tuple[float, float, float]:
            return (0.1, -0.2, -1.5)

        def _wait_for_settled_local_position(
            self,
            *,
            duration_s: float,
            timeout_s: float,
            position_ned: tuple[float, float, float] | None = None,
            setpoint_tick: object | None = None,
        ) -> None:
            if position_ned is not None:
                calls.append(f"settle_position:{position_ned[2]:.1f}")
            elif setpoint_tick is not None:
                calls.append("settle_tick")
            else:
                calls.append("settle_spin")

        def _publish_position_setpoint(self, position_ned: tuple[float, float, float]) -> None:
            calls.append(f"position:{position_ned[2]:.1f}")

        def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
            calls.append(f"publish_for:{command.segment_name}")

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            calls.append(f"publish:{command.segment_name}")

        def _move_arm_and_record_base_motion(
            self,
            case: object,
            hold: object,
        ) -> list[object]:
            calls.append(f"arm_motion_hold:{type(hold).__name__}")
            return []

        def _run_profile(self, profile: object) -> tuple[object, float, tuple[object, ...]]:
            calls.append("profile")
            return runner.VelocityTrackingMetrics(), 1.0, ()

        def _land_and_disarm(self, mav: object, mavlink: object) -> None:
            pass

        def _altitude_m(self) -> float:
            return 1.5

    fake_pymavlink: Any = types.ModuleType("pymavlink")
    fake_pymavlink.mavutil = FakeMavUtil
    with patch.dict(sys.modules, {"pymavlink": fake_pymavlink}):
        with patch.object(runner, "_wait_command_ack", lambda *args, **kwargs: calls.append(f"ack:{args[1]}")):
            with patch.object(runner, "_send_arm_command", lambda *args, **kwargs: None):
                result = runner.X500Arm2xBenchmarkController.run(FakeController(), case)

    assert result.controller == "px4_position"
    assert "mode:1.0" not in calls
    assert calls.index("takeoff") < calls.index("position:-1.5")
    assert calls.index("position:-1.5") < calls.index("mode:0.0")
    assert calls.index("mode:0.0") < calls.index("arm_motion_hold:tuple")
    assert calls.index("arm_motion_hold:tuple") < calls.index("publish_for:hold") < calls.index("profile")


def test_arm_motion_sampling_uses_am_hold_command_instead_of_position_setpoint() -> None:
    runner = _load_runner_module()
    calls: list[str] = []
    monotonic_values = iter([0.0, 0.0, 0.02, 0.04, 0.06])

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig(arm_motion_duration_s=0.03)

        def _send_arm_motion_command(self, case: object) -> dict[str, object]:
            calls.append("send_arm_motion")
            return {"ok": True, "duration_s": 0.05}

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            calls.append(f"publish_command:{command.segment_name}:{command.velocity_h}")

        def _publish_position_setpoint(self, position_ned: tuple[float, float, float]) -> None:
            calls.append(f"publish_position:{position_ned[2]:.1f}")

        def _local_position_xyz_ned(self) -> tuple[float, float, float]:
            return (1.0, 2.0, -1.5)

        def _local_velocity_ned(self) -> tuple[float, float, float]:
            return (0.0, 0.0, 0.0)

        def _attitude_rpy(self) -> tuple[float, float, float]:
            return (0.01, -0.02, 0.03)

        def _sleep_and_spin(self, duration_s: float | None = None) -> None:
            calls.append("sleep")

    hold_command = VelocityTrackingCommand(active=True, segment_name="hold", velocity_h=(0.0, 0.0, 0.0))
    with patch.object(runner.time, "monotonic", lambda: next(monotonic_values)):
        samples = runner.X500Arm2xBenchmarkController._move_arm_and_record_base_motion(
            FakeController(),
            runner.default_arm_pose_cases()[0],
            hold_command,
        )

    assert len(samples) == 3
    assert samples[-1].progress == pytest.approx(0.8)
    assert samples[-1].roll_rad == pytest.approx(0.01)
    assert samples[-1].pitch_rad == pytest.approx(-0.02)
    assert samples[-1].yaw_rad == pytest.approx(0.03)
    assert calls == [
        "send_arm_motion",
        "publish_command:hold:(0.0, 0.0, 0.0)",
        "sleep",
        "publish_command:hold:(0.0, 0.0, 0.0)",
        "sleep",
        "publish_command:hold:(0.0, 0.0, 0.0)",
        "sleep",
    ]


def test_arm_motion_command_sends_five_joint_pose_to_am_env() -> None:
    runner = _load_runner_module()
    sent_payloads: list[dict[str, object]] = []

    class FakeSocket:
        def setsockopt(self, *_args: object) -> None:
            pass

        def connect(self, endpoint: str) -> None:
            assert endpoint == "tcp://127.0.0.1:5604"

        def send_json(self, payload: dict[str, object]) -> None:
            sent_payloads.append(payload)

        def recv_json(self) -> dict[str, object]:
            return {"ok": True, "duration_s": 3.0}

        def close(self, linger: int = 0) -> None:
            pass

    class FakeContext:
        def socket(self, socket_type: object) -> FakeSocket:
            assert socket_type == zmq.REQ
            return FakeSocket()

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig(arm_motion_duration_s=3.0)
        _isolation = runner._default_isolation(0, _config)

    with patch.object(runner.zmq.Context, "instance", lambda: FakeContext()):
        reply = runner.X500Arm2xBenchmarkController._send_arm_motion_command(
            FakeController(),
            runner.default_arm_pose_cases()[3],
        )

    assert reply["ok"] is True
    assert len(sent_payloads) == 1
    assert sent_payloads[0]["type"] == "move_joint_pose"
    assert sent_payloads[0]["pose"] == list(runner.default_arm_pose_cases()[3].pose[:5])


def test_settled_position_wait_keeps_publishing_at_controller_loop_rate() -> None:
    runner = _load_runner_module()
    calls: list[str] = []
    monotonic_values = iter([0.0, 0.0, 0.0, 0.02, 0.02])

    class FakeController:
        _period_s = 0.02

        def _publish_position_setpoint(self, position_ned: tuple[float, float, float]) -> None:
            calls.append(f"publish:{position_ned[2]:.1f}")

        def _spin_once(self, timeout_s: float = 0.0) -> None:
            calls.append(f"spin:{timeout_s:g}")

        def _sleep_and_spin(self, duration_s: float | None = None) -> None:
            calls.append("sleep_and_spin")

        def _local_position_valid(self) -> bool:
            return True

        def _ground_speed_mps(self) -> float:
            return 0.0

    with patch.object(runner.time, "monotonic", lambda: next(monotonic_values)):
        with patch.object(runner.time, "sleep", lambda duration: calls.append(f"direct_sleep:{duration:g}")):
            runner.X500Arm2xBenchmarkController._wait_for_settled_local_position(
                FakeController(),
                duration_s=0.01,
                timeout_s=1.0,
                position_ned=(0.0, 0.0, -1.5),
            )

    assert calls == ["publish:-1.5", "sleep_and_spin", "publish:-1.5"]


def test_controller_reports_takeoff_timeout_without_masking_cleanup_failure() -> None:
    runner = _load_runner_module()

    class FakeMav:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = types.SimpleNamespace(
                heartbeat_send=lambda *args: None,
                command_long_send=lambda *args: None,
            )

        def wait_heartbeat(self, timeout: float | None = None) -> None:
            pass

        def close(self) -> None:
            pass

    class FakeMavUtil:
        mavlink = types.SimpleNamespace(
            MAV_CMD_DO_SET_MODE=176,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_TYPE_GENERIC=0,
            MAV_AUTOPILOT_INVALID=8,
        )

        @staticmethod
        def mavlink_connection(
            url: str,
            source_system: int,
            source_component: int,
            autoreconnect: bool = False,
        ) -> FakeMav:
            return FakeMav()

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig(takeoff_altitude_m=1.5)

        def _wait_for_local_position(self, *, timeout_s: float) -> None:
            pass

        def _wait_for_global_position(self, *, timeout_s: float) -> None:
            pass

        def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
            pass

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            pass

        def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
            return (math.nan, math.nan, -1.5)

        def _takeoff_target_global(self) -> tuple[float, float, float]:
            return (39.1, 116.2, 51.5)

        def _takeoff(self, command_or_position: object) -> tuple[bool, float]:
            return False, 0.4

        def _land_and_disarm(self, mav: object, mavlink: object) -> None:
            raise RuntimeError("landing failed")

        def _altitude_m(self) -> float:
            return 0.4

    fake_pymavlink: Any = types.ModuleType("pymavlink")
    fake_pymavlink.mavutil = FakeMavUtil
    with patch.dict(sys.modules, {"pymavlink": fake_pymavlink}):
        with patch.object(runner, "_wait_command_ack", lambda *args, **kwargs: None):
            with patch.object(runner, "_send_arm_command", lambda *args, **kwargs: None):
                result = runner.X500Arm2xBenchmarkController.run(FakeController(), runner.default_arm_pose_cases()[0])

    assert result.passed is False
    assert result.takeoff_success is False
    assert result.max_altitude_m == 0.4
    assert result.error is not None
    assert result.error.startswith("Timed out reaching takeoff altitude 1.5 m")
    assert "cleanup failed: landing failed" in result.error


def test_controller_preserves_profile_summary_when_landing_fails() -> None:
    runner = _load_runner_module()

    class FakeMav:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = types.SimpleNamespace(
                heartbeat_send=lambda *args: None,
                command_long_send=lambda *args: None,
            )

        def wait_heartbeat(self, timeout: float | None = None) -> None:
            pass

        def close(self) -> None:
            pass

    class FakeMavUtil:
        mavlink = types.SimpleNamespace(
            MAV_CMD_DO_SET_MODE=176,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_TYPE_GENERIC=0,
            MAV_AUTOPILOT_INVALID=8,
        )

        @staticmethod
        def mavlink_connection(
            url: str,
            source_system: int,
            source_component: int,
            autoreconnect: bool = False,
        ) -> FakeMav:
            return FakeMav()

    class FakeMetrics:
        def summary(self) -> VelocityTrackingSummary:
            return VelocityTrackingSummary(sample_count=12, rms_speed_error_norm_mps=0.2)

    class FakeController:
        _config = runner.BenchmarkRuntimeConfig()

        def _wait_for_local_position(self, *, timeout_s: float) -> None:
            pass

        def _wait_for_global_position(self, *, timeout_s: float) -> None:
            pass

        def _takeoff_target_position_ned(self) -> tuple[float, float, float]:
            return (math.nan, math.nan, -1.5)

        def _takeoff_target_global(self) -> tuple[float, float, float]:
            return (39.1, 116.2, 51.5)

        def _takeoff(self, position_ned: tuple[float, float, float]) -> tuple[bool, float]:
            return True, 1.7

        def _wait_for_settled_local_position(
            self,
            *,
            duration_s: float,
            timeout_s: float,
            position_ned: tuple[float, float, float] | None = None,
            setpoint_tick: object | None = None,
        ) -> None:
            pass

        def _publish_for(self, command: VelocityTrackingCommand, *, duration_s: float) -> None:
            pass

        def _publish_command(self, command: VelocityTrackingCommand) -> None:
            pass

        def _move_arm_and_record_base_motion(
            self,
            case: object,
            hold_command: VelocityTrackingCommand,
        ) -> list[object]:
            return []

        def _run_profile(self, profile: object) -> tuple[object, float, tuple[object, ...]]:
            return FakeMetrics(), 42.0, ()

        def _land_and_disarm(self, mav: object, mavlink: object) -> None:
            raise RuntimeError("landing failed")

        def _altitude_m(self) -> float:
            return 2.0

    fake_pymavlink: Any = types.ModuleType("pymavlink")
    fake_pymavlink.mavutil = FakeMavUtil
    with patch.dict(sys.modules, {"pymavlink": fake_pymavlink}):
        with patch.object(runner, "_wait_command_ack", lambda *args, **kwargs: None):
            with patch.object(runner, "_send_arm_command", lambda *args, **kwargs: None):
                result = runner.X500Arm2xBenchmarkController.run(FakeController(), runner.default_arm_pose_cases()[0])

    assert result.takeoff_success is True
    assert result.tracking_duration_s == 42.0
    assert result.summary.sample_count == 12
    assert result.max_altitude_m == 2.0
    assert result.error == "landing failed"


def test_parse_args_rejects_non_positive_profile_cycles() -> None:
    runner = _load_runner_module()

    with pytest.raises(SystemExit):
        runner._parse_args(["--profile-cycles", "0"])

    with pytest.raises(SystemExit):
        runner._parse_args(["--profile-cycles", "-1"])


def test_parse_args_defaults_to_single_case_start_attempt() -> None:
    runner = _load_runner_module()

    args = runner._parse_args([])

    assert args.case_start_attempts == 1


def test_parse_args_rejects_invalid_timing_options() -> None:
    runner = _load_runner_module()

    for args in (
        ["--takeoff-altitude-tolerance-m", "-0.1"],
        ["--case-start-attempts", "0"],
        ["--post-takeoff-settle-s", "-0.1"],
        ["--am-offboard-settle-s", "-0.1"],
        ["--max-profile-altitude-m", "0"],
    ):
        with pytest.raises(SystemExit):
            runner._parse_args(args)


def test_parse_args_rejects_removed_real_time_rate_option() -> None:
    runner = _load_runner_module()

    with pytest.raises(SystemExit):
        runner._parse_args(["--real-time-rate", "1"])
