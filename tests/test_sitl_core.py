from __future__ import annotations

import os
import signal
import subprocess
import sys
import unittest
from pathlib import Path
from typing import cast
from unittest.mock import patch

from acesim.config.config_loader import ConfigLoader
from acesim.sitl.process import ProcessSpec
from acesim.utils.px4_transport import PX4SensorParams

_ProcessEntry = tuple[ProcessSpec, object]


class _FakeConfigLoader:
    def __init__(self, asset_name: str, asset_params: dict[str, object] | None = None) -> None:
        self._asset_name = asset_name
        self._asset_params = asset_params or {}

    def get_asset_name(self) -> str:
        return self._asset_name

    def get_asset_params(self) -> dict[str, object]:
        return self._asset_params


class _FakePX4SensorParams:
    def __init__(self, fusion_mode: str = "mocap") -> None:
        self.gps_home_lat_lon = (39.98329, 116.34745)
        self.gps_alt_start = 50.0
        self.fusion_mode = fusion_mode
        self.ekf2_ev_ctrl = 11
        self.ekf2_hgt_ref = "Vision"
        self.ekf2_ev_delay_ms = 0
        self.ekf2_ev_pos_body_m = (0.0, 0.0, 0.0)
        self.ekf2_ev_noise_md = 0
        self.ekf2_evp_noise = 0.0
        self.ekf2_evv_noise = 0.0
        self.ekf2_eva_noise = 0.0
        self.ekf2_gps_ctrl = 0
        self.ekf2_mag_type = 0

    @classmethod
    def from_asset_params(cls, asset_params: dict[str, object], dynamic_hil_sensor_fields: bool = False):
        return cls(str(asset_params.get("fusion_mode", "mocap")))


class SITLCoreTests(unittest.TestCase):
    def test_build_px4_env_maps_assets_and_preserves_gz_suppression(self) -> None:
        from acesim.sitl import build_px4_env

        env = build_px4_env(
            cast(ConfigLoader, _FakeConfigLoader("advanced_plane", {"fusion_mode": "mocap"})),
            sensor_params_cls=cast(type[PX4SensorParams], _FakePX4SensorParams),
        )

        self.assertEqual(env["PX4_SYS_AUTOSTART"], "1039")
        self.assertEqual(env["PX4_SIM_MODEL"], "none")
        self.assertEqual(env["PX4_SIMULATOR"], "none")
        self.assertEqual(env["PX4_PARAM_SIM_GZ_EN"], "0")
        self.assertEqual(env["PX4_PARAM_EKF2_HGT_REF"], "3")
        self.assertEqual(env["PX4_PARAM_SYS_HAS_GPS"], "0")

    def test_build_px4_make_command_exports_sorted_environment(self) -> None:
        from acesim.sitl import build_px4_make_command

        command = build_px4_make_command({"B": "two words", "A": "1"})

        self.assertEqual(command, "env A=1 B='two words' make px4_sitl none")

    def test_config_derives_instance_ports(self) -> None:
        from acesim.sitl import PX4SITLConfig

        config = PX4SITLConfig(px4_repo=Path("/tmp/px4"), px4_instance=3)

        self.assertEqual(config.readiness_mode, "background")
        self.assertEqual(config.play_start_delay_sec, 0.0)
        self.assertEqual(config.px4_sim_tcp_port, 4563)
        self.assertEqual(config.mavlink_udp_port, 14543)
        self.assertEqual(config.px4_mavlink_url, "udpin:0.0.0.0:14543")

    def test_config_rejects_instances_outside_px4_mavlink_mapping(self) -> None:
        from acesim.sitl import PX4SITLConfig

        with self.assertRaisesRegex(ValueError, "px4_instance must be <= 9"):
            PX4SITLConfig(px4_repo=Path("/tmp/px4"), px4_instance=10)

    def test_runner_default_specs_are_pure_sitl(self) -> None:
        from acesim.sitl import PX4SITLConfig, PX4SITLRunner

        config = PX4SITLConfig(
            px4_repo=Path("/tmp/px4"),
            config_path=Path("/tmp/config.toml"),
            headless=True,
            readiness_mode="off",
        )
        specs = PX4SITLRunner(config).build_process_specs(
            px4_env={"PX4_SYS_AUTOSTART": "10016", "PX4_SIM_MODEL": "none"}
        )

        self.assertEqual([spec.name for spec in specs], ["acesim", "px4"])
        self.assertEqual(specs[0].command[:3], [sys.executable, "-m", "acesim.sitl.play"])
        self.assertIn("--headless", specs[0].command)
        self.assertIn("make px4_sitl none", " ".join(specs[1].command))
        self.assertEqual(specs[1].cwd, Path("/tmp/px4"))
        self.assertNotIn("--ros2-bridge", " ".join(token for spec in specs for token in spec.command))

    def test_runner_uses_direct_px4_binary_for_nonzero_instance(self) -> None:
        from acesim.sitl import PX4SITLConfig, PX4SITLRunner

        config = PX4SITLConfig(px4_repo=Path("/tmp/px4"), px4_instance=4, readiness_mode="off")
        specs = PX4SITLRunner(config).build_process_specs(px4_env={"PX4_SIM_MODEL": "none"})

        px4_command = " ".join(specs[1].command)
        self.assertIn("/tmp/px4/build/px4_sitl_default/bin/px4", px4_command)
        self.assertIn("-d -i 4", px4_command)
        self.assertIn("-w /tmp/px4/build/px4_sitl_default/rootfs/4", px4_command)
        self.assertIn("/tmp/px4/build/px4_sitl_default/etc", px4_command)
        self.assertEqual(specs[0].env["ACESIM_PX4_SIM_TCP_PORT"], "4564")

    def test_module_help_does_not_require_ros2_overlay(self) -> None:
        result = subprocess.run(
            [sys.executable, "-m", "acesim.sitl", "--help"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--px4-repo", result.stdout)
        self.assertNotIn("--ros2-bridge", result.stdout)
        self.assertNotIn("--bridge-mode", result.stdout)
        self.assertIn("--readiness-mode", result.stdout)

    def test_process_log_formats_wrapped_command_without_shell_script(self) -> None:
        from acesim.sitl.process import build_graceful_shutdown_command, format_process_command_for_log

        command = build_graceful_shutdown_command("env PYTHONUNBUFFERED=1 ros2 run acesim_ros2 acesim_bridge")

        self.assertEqual(
            format_process_command_for_log(command),
            "env PYTHONUNBUFFERED=1 ros2 run acesim_ros2 acesim_bridge",
        )
        self.assertNotIn("_cleanup_after_signal", format_process_command_for_log(command))

    def test_core_play_headless_loop_does_not_yield_after_each_step(self) -> None:
        from acesim.sitl import play

        calls: list[str] = []

        class _FakeEnv:
            def step(self) -> None:
                calls.append("step")
                os.kill(os.getpid(), signal.SIGTERM)

            def run(self) -> None:
                raise AssertionError("headless mode should not enter GUI run")

            def close(self) -> None:
                calls.append("close")

        with (
            patch.object(sys, "argv", ["acesim.sitl.play", "--headless"]),
            patch.object(play, "make_env", return_value=_FakeEnv()),
        ):
            exit_code = play.main()

        self.assertFalse(hasattr(play, "time"))
        self.assertEqual(exit_code, 0)
        self.assertEqual(calls, ["step", "close"])

    def test_runner_does_not_start_px4_when_acesim_startup_fails(self) -> None:
        from acesim.sitl import PX4SITLConfig, PX4SITLRunner

        started: list[str] = []

        class _FakeProcess:
            def __init__(self, returncode: int | None) -> None:
                self._returncode = returncode

            def poll(self) -> int | None:
                return self._returncode

            def send_signal(self, _signum: int) -> None:
                return None

            def kill(self) -> None:
                return None

        class _FakeSupervisor:
            def __init__(self) -> None:
                self._processes: list[_ProcessEntry] = []

            def start(self, spec):
                started.append(spec.name)
                process = _FakeProcess(255 if spec.name == "acesim" else None)
                self._processes.append((spec, process))
                return process

            def poll_exited(self):
                for spec, process in self._processes:
                    returncode = process.poll()
                    if returncode is not None:
                        return spec, returncode
                return None

            def terminate_all(self) -> None:
                return None

        runner = PX4SITLRunner(
            PX4SITLConfig(
                px4_repo=Path("/tmp/px4"),
                readiness_mode="off",
            )
        )

        with patch("acesim.sitl.runner.ProcessSupervisor", _FakeSupervisor):
            with patch("acesim.sitl.runner.wait_for_acesim_ready_marker", return_value=True):
                with patch.object(runner, "_config_loader", return_value=_FakeConfigLoader("x500_arm2x")):
                    exit_code = runner.run()

        self.assertEqual(exit_code, 255)
        self.assertEqual(started, ["acesim"])

    def test_runner_waits_for_acesim_ready_marker_before_starting_px4(self) -> None:
        from acesim.sitl import PX4SITLConfig, PX4SITLRunner

        started: list[str] = []
        marker_checks: list[Path] = []

        class _FakeProcess:
            def poll(self) -> int | None:
                return None

            def send_signal(self, _signum: int) -> None:
                return None

            def kill(self) -> None:
                return None

        class _FakeSupervisor:
            def __init__(self) -> None:
                self._processes: list[_ProcessEntry] = []

            def start(self, spec):
                started.append(spec.name)
                process = _FakeProcess()
                self._processes.append((spec, process))
                return process

            def poll_exited(self):
                return None

            def wait_for_any_exit(self, poll_period_sec: float = 0.2):
                return self._processes[-1][0], 0

            def terminate_all(self) -> None:
                return None

        def _fake_wait(path: Path, timeout_sec: float = 10.0) -> bool:
            marker_checks.append(path)
            self.assertEqual(started, ["acesim"])
            return True

        runner = PX4SITLRunner(PX4SITLConfig(px4_repo=Path("/tmp/px4"), readiness_mode="off"))

        with patch("acesim.sitl.runner.ProcessSupervisor", _FakeSupervisor):
            with patch("acesim.sitl.runner.wait_for_acesim_ready_marker", side_effect=_fake_wait):
                with patch.object(runner, "_config_loader", return_value=_FakeConfigLoader("x500_arm2x")):
                    runner.run()

        self.assertEqual(len(marker_checks), 1)
        self.assertEqual(marker_checks[0].name, "ready")
        self.assertEqual(started[:2], ["acesim", "px4"])

    def test_background_readiness_runs_non_destructive_diagnostics(self) -> None:
        from acesim.sitl import PX4SITLConfig, PX4SITLRunner

        calls: list[str] = []

        class _FakeProcess:
            def poll(self) -> int | None:
                return None

            def send_signal(self, _signum: int) -> None:
                return None

            def kill(self) -> None:
                return None

        class _FakeSupervisor:
            def __init__(self) -> None:
                self._processes: list[_ProcessEntry] = []

            def start(self, spec):
                process = _FakeProcess()
                self._processes.append((spec, process))
                return process

            def poll_exited(self):
                return None

            def wait_for_any_exit(self, poll_period_sec: float = 0.2):
                return self._processes[-1][0], 0

            def terminate_all(self) -> None:
                return None

        class _FakeMav:
            pass

        runner = PX4SITLRunner(PX4SITLConfig(px4_repo=Path("/tmp/px4"), readiness_mode="background"))

        with patch("acesim.sitl.runner.ProcessSupervisor", _FakeSupervisor):
            with patch("acesim.sitl.runner.wait_for_acesim_ready_marker", return_value=True):
                with patch("acesim.sitl.runner.mavutil.mavlink_connection", return_value=_FakeMav()):
                    with patch(
                        "acesim.sitl.runner.wait_for_mavlink", side_effect=lambda *args: calls.append("mavlink")
                    ):
                        with patch(
                            "acesim.sitl.runner.run_required_px4_setup",
                            side_effect=lambda *args: calls.append("setup"),
                        ):
                            with patch(
                                "acesim.sitl.runner.start_background_readiness_diagnostics",
                                side_effect=lambda *args: calls.append("background"),
                            ):
                                with patch.object(
                                    runner, "_config_loader", return_value=_FakeConfigLoader("x500_arm2x")
                                ):
                                    runner.run()

        self.assertEqual(calls, ["mavlink", "setup", "background"])

    def test_wait_readiness_runs_strict_armability_path(self) -> None:
        from acesim.sitl import PX4SITLConfig, PX4SITLRunner

        calls: list[str] = []

        class _FakeProcess:
            def poll(self) -> int | None:
                return None

            def send_signal(self, _signum: int) -> None:
                return None

            def kill(self) -> None:
                return None

        class _FakeSupervisor:
            def __init__(self) -> None:
                self._processes: list[_ProcessEntry] = []

            def start(self, spec):
                process = _FakeProcess()
                self._processes.append((spec, process))
                return process

            def poll_exited(self):
                return None

            def wait_for_any_exit(self, poll_period_sec: float = 0.2):
                return self._processes[-1][0], 0

            def terminate_all(self) -> None:
                return None

        class _FakeMav:
            pass

        runner = PX4SITLRunner(PX4SITLConfig(px4_repo=Path("/tmp/px4"), readiness_mode="wait"))

        with patch("acesim.sitl.runner.ProcessSupervisor", _FakeSupervisor):
            with patch("acesim.sitl.runner.wait_for_acesim_ready_marker", return_value=True):
                with patch("acesim.sitl.runner.mavutil.mavlink_connection", return_value=_FakeMav()):
                    with patch(
                        "acesim.sitl.runner.wait_for_mavlink", side_effect=lambda *args: calls.append("mavlink")
                    ):
                        with patch(
                            "acesim.sitl.runner.run_strict_px4_readiness",
                            side_effect=lambda *args: calls.append("strict"),
                        ):
                            with patch.object(runner, "_config_loader", return_value=_FakeConfigLoader("x500_arm2x")):
                                runner.run()

        self.assertEqual(calls, ["mavlink", "strict"])

    def test_no_readiness_check_cli_maps_to_off_mode(self) -> None:
        from acesim.sitl.cli import build_arg_parser, config_from_args

        args = build_arg_parser().parse_args(["--no-readiness-check"])

        self.assertEqual(config_from_args(args).readiness_mode, "off")

    def test_readiness_mode_cli_sets_explicit_mode(self) -> None:
        from acesim.sitl.cli import build_arg_parser, config_from_args

        args = build_arg_parser().parse_args(["--readiness-mode", "wait"])

        self.assertEqual(config_from_args(args).readiness_mode, "wait")

    def test_mocap_required_setup_runs_without_armability_verification(self) -> None:
        from acesim.sitl import readiness

        calls: list[str] = []
        mav = object()

        with patch.object(readiness, "send_ekf_origin", side_effect=lambda *args: calls.append("origin")):
            with patch.object(readiness, "request_readiness_streams") as streams:
                with patch.object(readiness, "request_message_interval") as interval:
                    with patch.object(readiness, "send_heartbeat"):
                        with patch.object(readiness, "wait_for_mocap_armability") as armability:
                            readiness.run_required_px4_setup(mav, "mocap", 1.0, 2.0, 3.0)

        self.assertEqual(calls, ["origin"])
        streams.assert_not_called()
        interval.assert_called_once_with(mav, readiness.GPS_GLOBAL_ORIGIN_MSG_ID)
        armability.assert_not_called()

    def test_non_mocap_required_setup_skips_ekf_origin(self) -> None:
        from acesim.sitl import readiness

        with patch.object(readiness, "request_readiness_streams") as streams:
            with patch.object(readiness, "send_ekf_origin") as origin:
                readiness.run_required_px4_setup(object(), "hil", 1.0, 2.0, 3.0)

        streams.assert_not_called()
        origin.assert_not_called()

    def test_background_diagnostics_do_not_verify_armability(self) -> None:
        from acesim.sitl import readiness

        with patch.object(readiness, "wait_for_estimator_ready_quietly") as estimator:
            with patch.object(readiness, "wait_for_mocap_armability") as armability:
                readiness.run_background_readiness_diagnostics(object(), "mocap", 1.0, 2.0, 3.0)

        estimator.assert_called_once()
        armability.assert_not_called()

    def test_background_diagnostics_does_not_print_transient_readiness_waits(self) -> None:
        from acesim.sitl import readiness

        class _NeverReadyMav:
            def recv_match(self, **_kwargs):
                return None

        printed: list[str] = []

        with patch.object(readiness, "request_readiness_streams"):
            with patch(
                "acesim.sitl.readiness.print", side_effect=lambda *args, **_kwargs: printed.append(str(args[0]))
            ):
                readiness.wait_for_estimator_ready_quietly(_NeverReadyMav(), 1.0, 2.0, timeout_sec=0.01)

        self.assertFalse(any("waiting for MAVLink readiness" in line for line in printed))


if __name__ == "__main__":
    unittest.main()
