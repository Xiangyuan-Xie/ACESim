from __future__ import annotations

import importlib.util
import os
import sys
import types
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from unittest.mock import patch


def _load_px4_post_start_setup_module() -> ModuleType:
    module_name = "_test_acesim_ros2_px4_post_start_setup"
    for name in [
        module_name,
        "pymavlink",
        "pymavlink.mavutil",
        "rclpy",
        "px4_msgs",
    ]:
        sys.modules.pop(name, None)

    pymavlink_module: ModuleType = types.ModuleType("pymavlink")
    mavutil_module: ModuleType = types.ModuleType("pymavlink.mavutil")

    class _Mavlink:
        MAV_TYPE_GENERIC = 0
        MAV_AUTOPILOT_INVALID = 8
        MAV_CMD_SET_MESSAGE_INTERVAL = 511
        MAV_CMD_RUN_PREARM_CHECKS = 401
        MAV_CMD_COMPONENT_ARM_DISARM = 400
        MAV_RESULT_ACCEPTED = 0
        MAV_MODE_FLAG_SAFETY_ARMED = 128
        MAV_SYS_STATUS_PREARM_CHECK = 268435456
        MAVLINK_MSG_ID_SYS_STATUS = 1
        MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
        MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN = 49
        MAVLINK_MSG_ID_ESTIMATOR_STATUS = 230
        ESTIMATOR_ATTITUDE = 1
        ESTIMATOR_VELOCITY_HORIZ = 2
        ESTIMATOR_VELOCITY_VERT = 4
        ESTIMATOR_POS_HORIZ_REL = 8
        ESTIMATOR_POS_HORIZ_ABS = 16
        ESTIMATOR_POS_VERT_ABS = 32
        ESTIMATOR_CONST_POS_MODE = 128
        ESTIMATOR_GPS_GLITCH = 1024
        ESTIMATOR_ACCEL_ERROR = 2048

    def mavlink_connection(*_args: object, **_kwargs: object) -> object:
        raise RuntimeError("mavlink connection should be patched in tests")

    setattr(mavutil_module, "mavlink", _Mavlink)
    setattr(mavutil_module, "mavlink_connection", mavlink_connection)
    setattr(pymavlink_module, "mavutil", mavutil_module)

    sys.modules["pymavlink"] = pymavlink_module
    sys.modules["pymavlink.mavutil"] = mavutil_module

    module_path = (
        Path(__file__).resolve().parents[1]
        / "acesim"
        / "deploy"
        / "aircraft"
        / "acesim_ros2"
        / "acesim_ros2"
        / "px4_post_start_setup.py"
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to create module spec for {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


@dataclass
class _MavlinkMessage:
    mavlink_type: str

    def get_type(self) -> str:
        return self.mavlink_type


@dataclass
class _LocalPositionNed(_MavlinkMessage):
    mavlink_type: str = "LOCAL_POSITION_NED"
    x: float = 0.0
    y: float = 0.0
    z: float = -0.2
    vx: float = 0.01
    vy: float = 0.02
    vz: float = 0.03


@dataclass
class _GlobalPositionInt(_MavlinkMessage):
    mavlink_type: str = "GLOBAL_POSITION_INT"
    lat: int = 399832900
    lon: int = 1163474500
    alt: int = 50000


@dataclass
class _GpsGlobalOrigin(_MavlinkMessage):
    mavlink_type: str = "GPS_GLOBAL_ORIGIN"
    latitude: int = 399832900
    longitude: int = 1163474500
    altitude: int = 50000


@dataclass
class _EstimatorStatus(_MavlinkMessage):
    mavlink_type: str = "ESTIMATOR_STATUS"
    flags: int = 0
    mag_ratio: float = 0.1
    vel_ratio: float = 0.1
    pos_horiz_ratio: float = 0.1
    pos_vert_ratio: float = 0.1


@dataclass
class _SysStatus(_MavlinkMessage):
    mavlink_type: str = "SYS_STATUS"
    onboard_control_sensors_health: int = 268435456


@dataclass
class _Statustext(_MavlinkMessage):
    mavlink_type: str = "STATUSTEXT"
    text: str = "Preflight Fail: heading estimate not stable"


@dataclass
class _CommandAck(_MavlinkMessage):
    mavlink_type: str = "COMMAND_ACK"
    command: int = 400
    result: int = 0


@dataclass
class _Heartbeat(_MavlinkMessage):
    mavlink_type: str = "HEARTBEAT"
    base_mode: int = 0


class _FakeMavSender:
    def __init__(self) -> None:
        self.command_long_requests: list[tuple[object, ...]] = []
        self.message_interval_requests: list[tuple[object, ...]] = []
        self.prearm_check_requests: list[tuple[object, ...]] = []
        self.arm_requests: list[tuple[object, ...]] = []
        self.origin_requests: list[tuple[object, ...]] = []

    def heartbeat_send(self, *_args: object) -> None:
        return None

    def command_long_send(self, *args: object) -> None:
        self.command_long_requests.append(args)
        command = args[2] if len(args) > 2 else None
        if command == 511:
            self.message_interval_requests.append(args)
        elif command == 401:
            self.prearm_check_requests.append(args)
        elif command == 400:
            self.arm_requests.append(args)

    def set_gps_global_origin_send(self, *args: object) -> None:
        self.origin_requests.append(args)


class _FakeMavConnection:
    def __init__(self, messages: list[_MavlinkMessage]) -> None:
        self.messages = messages
        self.mav = _FakeMavSender()
        self.target_system = 1
        self.target_component = 1

    def recv_match(self, **_kwargs: object) -> _MavlinkMessage | None:
        if not self.messages:
            return None
        return self.messages.pop(0)


class _VerifyArmableError(RuntimeError):
    pass


class PX4PostStartSetupTests(unittest.TestCase):
    module: ModuleType

    @classmethod
    def setUpClass(cls) -> None:
        cls.module = _load_px4_post_start_setup_module()

    def _ready_estimator(self) -> _EstimatorStatus:
        return _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS)

    def test_post_start_module_does_not_require_ros2_topics(self) -> None:
        self.assertFalse(hasattr(self.module, "rclpy"))
        self.assertFalse(hasattr(self.module, "PX4ReadinessNode"))

    def test_readiness_succeeds_with_stable_mavlink_samples(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertEqual(failures, [])

    def test_readiness_rejects_missing_local_position(self) -> None:
        failures = self.module.readiness_failures(
            None,
            _GlobalPositionInt(),
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertIn("LOCAL_POSITION_NED not received", failures)

    def test_readiness_rejects_missing_global_position(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            None,
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertIn("GLOBAL_POSITION_INT not received", failures)

    def test_readiness_rejects_missing_estimator_status(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            None,
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertIn("ESTIMATOR_STATUS not received", failures)

    def test_readiness_rejects_non_finite_local_position(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(x=float("nan")),
            _GlobalPositionInt(),
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertIn("LOCAL_POSITION_NED x=nan is not finite", failures)

    def test_readiness_rejects_home_origin_mismatch(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(lat=399000000),
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertTrue(any("GLOBAL_POSITION_INT origin mismatch" in failure for failure in failures))

    def test_readiness_rejects_estimator_missing_required_flags(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            _EstimatorStatus(flags=0),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertTrue(any("ESTIMATOR_STATUS missing required flags" in failure for failure in failures))

    def test_readiness_allows_const_pos_mode_in_static_mocap_startup(self) -> None:
        const_pos_mode = 128
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS | const_pos_mode),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertEqual(failures, [])

    def test_readiness_rejects_estimator_fault_flags(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS | self.module.ESTIMATOR_FORBIDDEN_FLAGS),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertTrue(any("ESTIMATOR_STATUS has forbidden flags" in failure for failure in failures))

    def test_readiness_rejects_heading_innovation_ratio_at_prearm_threshold(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS, mag_ratio=0.5),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertTrue(any("heading" in failure and "mag_ratio" in failure for failure in failures))

    def test_readiness_rejects_missing_heading_innovation_ratio(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS, mag_ratio=float("nan")),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertTrue(any("heading" in failure and "mag_ratio" in failure for failure in failures))

    def test_readiness_rejects_other_prearm_innovation_ratios_at_threshold(self) -> None:
        for field_name in ("vel_ratio", "pos_horiz_ratio", "pos_vert_ratio"):
            estimator = self._ready_estimator()
            setattr(estimator, field_name, 0.5)
            failures = self.module.readiness_failures(
                _LocalPositionNed(),
                _GlobalPositionInt(),
                estimator,
                39.98329,
                116.34745,
                sys_status=_SysStatus(),
            )

            self.assertTrue(any(field_name in failure for failure in failures), field_name)

    def test_readiness_allows_unavailable_non_heading_innovation_ratios_when_prearm_bit_is_set(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            _EstimatorStatus(
                flags=self.module.REQUIRED_ESTIMATOR_FLAGS,
                vel_ratio=float("nan"),
                pos_horiz_ratio=float("nan"),
                pos_vert_ratio=float("nan"),
            ),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
        )

        self.assertEqual(failures, [])

    def test_readiness_rejects_missing_sys_status_prearm_bit(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(onboard_control_sensors_health=0),
        )

        self.assertTrue(any("MAV_SYS_STATUS_PREARM_CHECK" in failure for failure in failures))

    def test_readiness_rejects_stale_mavlink_messages(self) -> None:
        failures = self.module.readiness_failures(
            _LocalPositionNed(),
            _GlobalPositionInt(),
            self._ready_estimator(),
            39.98329,
            116.34745,
            sys_status=_SysStatus(),
            message_ages_sec={"LOCAL_POSITION_NED": self.module.READINESS_STALE_SEC + 0.1},
        )

        self.assertTrue(any("LOCAL_POSITION_NED is stale" in failure for failure in failures))

    def test_ekf_origin_ack_accepts_matching_gps_global_origin(self) -> None:
        failures = self.module.ekf_origin_failures(_GpsGlobalOrigin(), 39.98329, 116.34745, 50.0)

        self.assertEqual(failures, [])

    def test_ekf_origin_ack_rejects_mismatched_gps_global_origin(self) -> None:
        failures = self.module.ekf_origin_failures(_GpsGlobalOrigin(latitude=399000000), 39.98329, 116.34745, 50.0)

        self.assertTrue(any("GPS_GLOBAL_ORIGIN mismatch" in failure for failure in failures))

    def test_wait_for_estimator_ready_succeeds_after_prearm_stable_window(self) -> None:
        mav = _FakeMavConnection(
            [
                _LocalPositionNed(),
                _GlobalPositionInt(),
                self._ready_estimator(),
                _SysStatus(),
            ]
        )

        with patch.object(self.module, "REQUIRED_READY_STABLE_SEC", 0.0):
            self.module.wait_for_estimator_ready(mav, 39.98329, 116.34745, timeout_sec=1.0)

        self.assertGreaterEqual(len(mav.mav.message_interval_requests), 4)
        self.assertEqual(mav.mav.prearm_check_requests, [])

    def test_wait_for_estimator_ready_does_not_send_prearm_checks_while_heading_is_unstable(self) -> None:
        mav = _FakeMavConnection(
            [
                _LocalPositionNed(),
                _GlobalPositionInt(),
                _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS, mag_ratio=0.9),
                _SysStatus(onboard_control_sensors_health=0),
            ]
        )

        with self.assertRaisesRegex(RuntimeError, "heading innovation"):
            self.module.wait_for_estimator_ready(mav, 39.98329, 116.34745, timeout_sec=0.01)

        self.assertEqual(mav.mav.prearm_check_requests, [])

    def test_wait_for_estimator_ready_does_not_send_prearm_checks_when_prearm_bit_is_missing(self) -> None:
        mav = _FakeMavConnection(
            [
                _LocalPositionNed(),
                _GlobalPositionInt(),
                self._ready_estimator(),
                _SysStatus(onboard_control_sensors_health=0),
            ]
        )

        with patch.object(self.module, "REQUIRED_READY_STABLE_SEC", 0.0):
            with self.assertRaisesRegex(RuntimeError, "MAV_SYS_STATUS_PREARM_CHECK"):
                self.module.wait_for_estimator_ready(mav, 39.98329, 116.34745, timeout_sec=0.01)

        self.assertEqual(mav.mav.prearm_check_requests, [])

    def test_wait_for_estimator_ready_reports_missing_mavlink_messages(self) -> None:
        mav = _FakeMavConnection([])

        with self.assertRaisesRegex(RuntimeError, "LOCAL_POSITION_NED not received"):
            self.module.wait_for_estimator_ready(mav, 39.98329, 116.34745, timeout_sec=0.01)

    def test_wait_for_estimator_ready_reports_recent_preflight_statustext(self) -> None:
        mav = _FakeMavConnection([_Statustext()])

        with self.assertRaisesRegex(RuntimeError, "heading estimate not stable"):
            self.module.wait_for_estimator_ready(mav, 39.98329, 116.34745, timeout_sec=0.01)

    def test_main_returns_zero_for_keyboard_interrupt(self) -> None:
        with patch.object(self.module, "run_post_start_setup", side_effect=KeyboardInterrupt):
            exit_code = self.module.main(["mocap", "39.0", "116.0", "50.0"])

        self.assertEqual(exit_code, 0)

    def test_main_returns_zero_for_sigterm_shutdown_request(self) -> None:
        with patch.object(self.module, "run_post_start_setup", side_effect=self.module.ShutdownRequested):
            exit_code = self.module.main(["mocap", "39.0", "116.0", "50.0"])

        self.assertEqual(exit_code, 0)

    def test_main_returns_nonzero_for_runtime_error(self) -> None:
        with patch.object(self.module, "run_post_start_setup", side_effect=RuntimeError("boom")):
            exit_code = self.module.main(["mocap", "39.0", "116.0", "50.0"])

        self.assertEqual(exit_code, 1)

    def test_send_ekf_origin_uses_mavlink_set_gps_global_origin(self) -> None:
        mav = _FakeMavConnection([_GpsGlobalOrigin()])

        self.module.send_ekf_origin(mav, 39.98329, 116.34745, 50.0, timeout_sec=1.0)

        self.assertEqual(mav.mav.origin_requests, [(1, 399832900, 1163474500, 50000)])

    def test_mocap_post_start_sends_ekf_origin_then_waits_for_readiness_without_listener_polling(self) -> None:
        calls: list[str] = []
        mav = object()

        with patch.object(self.module.mavutil, "mavlink_connection", return_value=mav):
            with patch.object(self.module, "wait_for_mavlink", side_effect=lambda _mav: calls.append("wait_mavlink")):
                with patch.object(
                    self.module,
                    "send_ekf_origin",
                    side_effect=lambda _mav, _lat, _lon, _alt: calls.append("send_origin"),
                ):
                    with patch.object(
                        self.module,
                        "wait_for_estimator_ready",
                        side_effect=lambda _mav, _lat, _lon, **_kwargs: calls.append("wait_ready"),
                    ):
                        with patch.object(
                            self.module, "verify_armable", side_effect=lambda _mav: calls.append("verify_armable")
                        ):
                            with patch.dict(os.environ, {}, clear=True):
                                self.module.run_post_start_setup(["mocap", "39.98329", "116.34745", "50.0"])

        self.assertEqual(
            calls,
            [
                "wait_mavlink",
                "send_origin",
                "wait_ready",
                "verify_armable",
            ],
        )

    def test_mocap_post_start_skips_armability_when_explicitly_disabled(self) -> None:
        calls: list[str] = []
        mav = object()

        with patch.object(self.module.mavutil, "mavlink_connection", return_value=mav):
            with patch.object(self.module, "wait_for_mavlink", side_effect=lambda _mav: calls.append("wait_mavlink")):
                with patch.object(
                    self.module,
                    "send_ekf_origin",
                    side_effect=lambda _mav, _lat, _lon, _alt: calls.append("send_origin"),
                ):
                    with patch.object(
                        self.module,
                        "wait_for_estimator_ready",
                        side_effect=lambda _mav, _lat, _lon, **_kwargs: calls.append("wait_ready"),
                    ):
                        with patch.object(
                            self.module, "verify_armable", side_effect=lambda _mav: calls.append("verify_armable")
                        ):
                            with patch.dict(os.environ, {"ACESIM_PX4_VERIFY_ARMABLE": "0"}):
                                self.module.run_post_start_setup(["mocap", "39.98329", "116.34745", "50.0"])

        self.assertEqual(
            calls,
            [
                "wait_mavlink",
                "send_origin",
                "wait_ready",
            ],
        )

    def test_mocap_post_start_retries_readiness_when_armability_rejects(self) -> None:
        calls: list[str] = []
        mav = object()

        def verify_once_then_succeed(_mav: object) -> None:
            calls.append("verify_armable")
            if calls.count("verify_armable") == 1:
                raise _VerifyArmableError("PX4 arm command rejected with MAV_RESULT 1")

        with patch.object(self.module.mavutil, "mavlink_connection", return_value=mav):
            with patch.object(self.module, "wait_for_mavlink", side_effect=lambda _mav: calls.append("wait_mavlink")):
                with patch.object(
                    self.module,
                    "send_ekf_origin",
                    side_effect=lambda _mav, _lat, _lon, _alt: calls.append("send_origin"),
                ):
                    with patch.object(
                        self.module,
                        "wait_for_estimator_ready",
                        side_effect=lambda _mav, _lat, _lon, **_kwargs: calls.append("wait_ready"),
                    ):
                        with patch.object(self.module, "verify_armable", side_effect=verify_once_then_succeed):
                            with patch.dict(os.environ, {}, clear=True):
                                self.module.run_post_start_setup(["mocap", "39.98329", "116.34745", "50.0"])

        self.assertEqual(
            calls,
            [
                "wait_mavlink",
                "send_origin",
                "wait_ready",
                "verify_armable",
                "wait_ready",
                "verify_armable",
            ],
        )

    def test_mocap_post_start_retries_transient_readiness_timeouts(self) -> None:
        calls: list[str] = []
        mav = object()

        def readiness_once_then_succeed(_mav: object, _lat: float, _lon: float, **_kwargs: object) -> None:
            calls.append("wait_ready")
            if calls.count("wait_ready") == 1:
                raise RuntimeError("PX4 estimator did not become ready for arming before timeout")

        with patch.object(self.module, "ARMABILITY_TOTAL_TIMEOUT_SEC", 2.0):
            with patch.object(self.module, "ARMABILITY_RETRY_DELAY_SEC", 0.0):
                with patch.object(self.module, "time") as time_mock:
                    now = [100.0]
                    time_mock.monotonic.side_effect = lambda: now.__setitem__(0, now[0] + 0.1) or now[0]
                    time_mock.sleep.side_effect = lambda _delay: None
                    with patch.object(self.module.mavutil, "mavlink_connection", return_value=mav):
                        with patch.object(
                            self.module, "wait_for_mavlink", side_effect=lambda _mav: calls.append("wait_mavlink")
                        ):
                            with patch.object(
                                self.module,
                                "send_ekf_origin",
                                side_effect=lambda _mav, _lat, _lon, _alt: calls.append("send_origin"),
                            ):
                                with patch.object(
                                    self.module,
                                    "wait_for_estimator_ready",
                                    side_effect=readiness_once_then_succeed,
                                ):
                                    with patch.object(
                                        self.module,
                                        "verify_armable",
                                        side_effect=lambda _mav: calls.append("verify_armable"),
                                    ):
                                        with patch.dict(os.environ, {}, clear=True):
                                            self.module.run_post_start_setup(["mocap", "39.98329", "116.34745", "50.0"])

        self.assertEqual(
            calls,
            [
                "wait_mavlink",
                "send_origin",
                "wait_ready",
                "wait_ready",
                "verify_armable",
            ],
        )

    def test_mocap_post_start_prints_ue_ready_message_after_armability_verification(self) -> None:
        mav = object()

        with patch.object(self.module.mavutil, "mavlink_connection", return_value=mav):
            with patch.object(self.module, "wait_for_mavlink"):
                with patch.object(self.module, "send_ekf_origin"):
                    with patch.object(self.module, "wait_for_estimator_ready"):
                        with patch.object(self.module, "verify_armable"):
                            with patch.dict(
                                os.environ,
                                {
                                    "ACESIM_PX4_VERIFY_ARMABLE": "1",
                                    "ACESIM_PX4_READY_CONTEXT": "UE mode",
                                },
                            ):
                                with patch("builtins.print") as print_mock:
                                    self.module.run_post_start_setup(["mocap", "39.98329", "116.34745", "50.0"])

        printed = "\n".join(str(call.args[0]) for call in print_mock.call_args_list if call.args)
        self.assertIn("PX4 estimator and armability verified for UE mode", printed)

    def test_verify_armable_arms_then_disarms(self) -> None:
        mav = _FakeMavConnection(
            [
                _CommandAck(),
                _Heartbeat(base_mode=128),
                _CommandAck(),
                _Heartbeat(base_mode=0),
            ]
        )

        self.module.verify_armable(mav, arm_timeout_sec=1.0, disarm_timeout_sec=1.0)

        self.assertEqual([request[4] for request in mav.mav.arm_requests], [1.0, 0.0])

    def test_verify_armable_rejects_failed_arm_ack(self) -> None:
        mav = _FakeMavConnection([_CommandAck(result=1)])

        with self.assertRaisesRegex(RuntimeError, "arm command rejected"):
            self.module.verify_armable(mav, arm_timeout_sec=1.0, disarm_timeout_sec=1.0)

    def test_mocap_post_start_logs_ekf_origin_setup(self) -> None:
        mav = object()

        with patch.object(self.module.mavutil, "mavlink_connection", return_value=mav):
            with patch.object(self.module, "wait_for_mavlink"):
                with patch.object(self.module, "send_ekf_origin"):
                    with patch.object(self.module, "wait_for_estimator_ready"):
                        with patch.object(self.module, "verify_armable"):
                            with patch("builtins.print") as print_mock:
                                self.module.run_post_start_setup(["mocap", "39.98329", "116.34745", "50.0"])

        printed_lines = [args[0] for args, _kwargs in print_mock.call_args_list]
        self.assertIn(
            "PX4 post-start setup: setting EKF origin to lat=39.98329 lon=116.34745 alt=50.0",
            printed_lines,
        )
        self.assertIn("PX4 post-start setup: EKF origin command sent", printed_lines)

    def test_readiness_timeout_diagnostics_include_ratios_prearm_bit_and_recent_preflight_status(self) -> None:
        mav = _FakeMavConnection(
            [
                _LocalPositionNed(),
                _GlobalPositionInt(),
                _EstimatorStatus(flags=self.module.REQUIRED_ESTIMATOR_FLAGS, mag_ratio=0.9),
                _SysStatus(onboard_control_sensors_health=0),
                _Statustext(),
            ]
        )

        with self.assertRaises(RuntimeError) as context:
            self.module.wait_for_estimator_ready(mav, 39.98329, 116.34745, timeout_sec=0.01)

        message = str(context.exception)
        self.assertIn("ESTIMATOR_STATUS mag_ratio", message)
        self.assertIn("SYS_STATUS MAV_SYS_STATUS_PREARM_CHECK", message)
        self.assertIn("heading estimate not stable", message)


if __name__ == "__main__":
    unittest.main()
