from __future__ import annotations

import inspect
from pathlib import Path

import tomli

from acesim.utils.arm_servo_scheduler import ArmControlSample, ArmServoScheduler, ArmStateSample
from acesim.utils.simulation_clock import SimulationClock


class _RecordingPublisher:
    def __init__(self) -> None:
        self.calls: list[tuple[int, tuple[float, ...], tuple[float, ...], tuple[float, ...]]] = []

    def publish(
        self,
        timestamp_us: int,
        positions,
        velocities,
        efforts,
    ) -> None:
        self.calls.append(
            (
                int(timestamp_us),
                tuple(float(value) for value in positions),
                tuple(float(value) for value in velocities),
                tuple(float(value) for value in efforts),
            )
        )


def test_arm_servo_scheduler_exposes_only_joint_state_delay_config() -> None:
    parameters = inspect.signature(ArmServoScheduler).parameters

    assert "joint_state_delay_ms" in parameters
    assert "command_delay_ms" not in parameters


def test_arm_asset_configs_only_model_joint_state_delay() -> None:
    repo_root = Path(__file__).resolve().parents[1]

    for config_path in (
        repo_root / "acesim" / "config" / "mujoco" / "x500_arm2x.toml",
        repo_root / "acesim" / "config" / "genesis" / "x500_arm2x.toml",
    ):
        config = tomli.loads(config_path.read_text(encoding="utf-8"))
        delay_config = config["params"]["arm"]["delay"]

        assert delay_config["joint_state_delay_ms"] == [0.0, 0.0]
        assert "command_delay_ms" not in delay_config


def test_arm_joint_state_delay_releases_original_sample_timestamp() -> None:
    clock = SimulationClock()
    publisher = _RecordingPublisher()

    def read_control() -> ArmControlSample | None:
        return None

    def apply_control(_sample: ArmControlSample) -> None:
        raise AssertionError("control should not be due in this test")

    def read_state() -> ArmStateSample:
        return ArmStateSample(positions=[1.0], velocities=[2.0], efforts=[3.0])

    scheduler = ArmServoScheduler(
        clock=clock,
        publisher=publisher,  # type: ignore[arg-type]
        control_rate_hz=50.0,
        state_publish_rate_hz=100.0,
        read_control_target=read_control,
        apply_control=apply_control,
        read_state=read_state,
        joint_state_delay_ms=(2.0, 2.0),
    )

    clock.advance_us(10_000)
    scheduler.update()
    assert publisher.calls == []

    clock.advance_us(1_999)
    scheduler.update()
    assert publisher.calls == []

    clock.advance_us(1)
    scheduler.update()
    assert publisher.calls == [(10_000, (1.0,), (2.0,), (3.0,))]
    clock.close()
