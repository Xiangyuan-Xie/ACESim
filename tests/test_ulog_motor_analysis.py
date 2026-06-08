from __future__ import annotations

import numpy as np

from acesim.tools.analysis_ulog import (
    AnalysisResult,
    build_nav_intervals,
    format_log_row,
    run_tui,
    summarize_delay_report,
    summarize_motor_window,
    summarize_px4_timing_evidence,
)


def test_build_nav_intervals_ignores_pre_start_samples() -> None:
    intervals = build_nav_intervals(
        start_timestamp=100,
        last_timestamp=5100,
        timestamps=np.array([50, 100, 1100, 2100, 4100]),
        nav_states=np.array([4, 2, 2, 17, 2]),
    )

    assert intervals == [
        (0.0, 0.002, 2),
        (0.002, 0.004, 17),
        (0.004, 0.005, 2),
    ]


def test_summarize_motor_window_reports_per_motor_and_all_motor_stats() -> None:
    times = np.array([0.0, 1.0, 2.0])
    controls = np.array(
        [
            [0.1, 0.2, 0.3, 0.4],
            [0.5, 0.6, 0.7, 0.8],
            [0.9, 1.0, 0.2, 0.1],
        ]
    )

    lines = summarize_motor_window("middle", times, controls, (times >= 1.0) & (times <= 2.0))

    assert lines[0] == "middle:"
    assert "  motor 0: mean=0.700000, min=0.500000, max=0.900000, samples=2" in lines
    assert "  motor 3: mean=0.450000, min=0.100000, max=0.800000, samples=2" in lines
    assert "  all motors: mean=0.600000, min=0.100000, max=1.000000" in lines


def test_summarize_motor_window_handles_empty_windows() -> None:
    lines = summarize_motor_window(
        "empty",
        np.array([0.0]),
        np.array([[0.1, 0.2, 0.3, 0.4]]),
        np.array([False]),
    )

    assert lines == ["empty:", "  no samples"]


def test_summarize_px4_timing_evidence_reports_source_backed_sensor_combined_rate() -> None:
    topics = {
        "sensor_combined": {
            "timestamp": np.array([0, 5_000, 10_000, 15_000], dtype=np.uint64),
            "timestamp_sample": np.array([0, 5_000, 10_000, 15_000], dtype=np.uint64),
        }
    }

    lines = summarize_px4_timing_evidence(topics)

    joined = "\n".join(lines)
    assert "topic=sensor_combined" in joined
    assert "ulog_recorded_rate=200.000 Hz" in joined
    assert "timestamp_sample_delay=0.000 ms" in joined
    assert "recommended_sim_rate=200.000 Hz" in joined
    assert "VehicleIMU.cpp" in joined
    assert "confidence=high" in joined


def test_summarize_px4_timing_evidence_marks_missing_topics_as_insufficient() -> None:
    lines = summarize_px4_timing_evidence({})

    joined = "\n".join(lines)
    assert "topic=vehicle_visual_odometry" in joined
    assert "ulog_recorded_rate=insufficient evidence" in joined
    assert "confidence=insufficient" in joined


def test_summarize_px4_timing_evidence_marks_actuator_latency_not_estimable_without_sample_timestamp() -> None:
    topics = {
        "actuator_outputs": {
            "timestamp": np.array([0, 100_000, 200_000], dtype=np.uint64),
        }
    }

    lines = summarize_px4_timing_evidence(topics)

    joined = "\n".join(lines)
    assert "topic=actuator_outputs" in joined
    assert "ulog_recorded_rate=10.000 Hz" in joined
    assert "timestamp_sample_delay=not estimable" in joined
    assert "no timestamp_sample" in joined


def test_summarize_delay_report_marks_am30_strong_and_diagnostic_ranges() -> None:
    topics = {
        "am_policy_observation": {
            "timestamp": np.array([1_000, 2_000, 3_000], dtype=np.uint64),
            "timestamp_sample": np.array([800, 1_850, 2_900], dtype=np.uint64),
            "am_setpoint_timestamp": np.array([900, 1_000, 1_500], dtype=np.uint64),
            **{f"observation[{idx}]": np.zeros(3, dtype=float) for idx in range(30)},
        },
        "actuator_outputs": {
            "timestamp": np.array([1_000, 2_000, 3_000], dtype=np.uint64),
        },
        "arm_joint_state": {
            "timestamp": np.array([500, 1_500, 2_500], dtype=np.uint64),
        },
    }

    lines = summarize_delay_report(topics)
    joined = "\n".join(lines)

    assert "am_policy_observation_dim=30" in joined
    assert "am_policy_compute_delay_ms" in joined
    assert "evidence=strong" in joined
    assert "actuator_delay_ms" in joined
    assert "evidence=not_estimable" in joined
    assert "arm_joint_state_latest_source_age_ms" in joined
    assert "evidence=diagnostic_only" in joined
    assert "suggested_toml" in joined


def test_summarize_delay_report_warns_for_legacy_am35_observation() -> None:
    topics = {
        "am_policy_observation": {
            "timestamp": np.array([1_000, 2_000], dtype=np.uint64),
            "timestamp_sample": np.array([900, 1_900], dtype=np.uint64),
            **{f"observation[{idx}]": np.zeros(2, dtype=float) for idx in range(35)},
        }
    }

    lines = summarize_delay_report(topics)

    assert any("am_policy_observation_dim=35" in line for line in lines)
    assert any("legacy observation dimension" in line for line in lines)


def test_format_log_row_shows_only_filename_and_modified_date(tmp_path) -> None:
    log_path = tmp_path / "15_34_33.ulg"
    log_path.write_text("placeholder", encoding="utf-8")

    row = format_log_row(3, log_path, modified_time=1777736133.0)

    assert row == "  3  15_34_33.ulg  2026-05-02 23:35:33"
    assert str(tmp_path) not in row


def test_run_tui_exits_after_selected_log_is_analyzed(tmp_path, monkeypatch) -> None:
    log_path = tmp_path / "15_34_33.ulg"
    log_path.write_text("placeholder", encoding="utf-8")
    output_dir = tmp_path / "analysis"
    result = AnalysisResult(
        log_path=log_path,
        figure_path=output_dir / "figure.png",
        summary_path=output_dir / "summary.txt",
        am_active_detected=True,
        am_warning_count=0,
    )

    class FakeScreen:
        def __init__(self) -> None:
            self.getch_calls = 0

        def erase(self) -> None:
            pass

        def getmaxyx(self) -> tuple[int, int]:
            return (24, 100)

        def addnstr(self, *args) -> None:
            pass

        def refresh(self) -> None:
            pass

        def getch(self) -> int:
            self.getch_calls += 1
            if self.getch_calls > 1:
                raise AssertionError("TUI should exit immediately after analyzing a selected log")
            return 10

    screen = FakeScreen()
    monkeypatch.setattr("acesim.tools.analysis_ulog.curses.curs_set", lambda _visibility: None)
    monkeypatch.setattr("acesim.tools.analysis_ulog.curses.wrapper", lambda callback: callback(screen))
    monkeypatch.setattr("acesim.tools.analysis_ulog.analyze_log", lambda selected, output: result)

    assert run_tui([log_path], output_dir) == result
    assert screen.getch_calls == 1
