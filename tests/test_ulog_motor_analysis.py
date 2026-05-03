from __future__ import annotations

import numpy as np

from acesim.tools.analysis_ulog import (
    AnalysisResult,
    build_nav_intervals,
    format_log_row,
    run_tui,
    summarize_motor_window,
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
    monkeypatch.setattr("acesim.tools.ulog_motor_tui.curses.curs_set", lambda _visibility: None)
    monkeypatch.setattr("acesim.tools.ulog_motor_tui.curses.wrapper", lambda callback: callback(screen))
    monkeypatch.setattr("acesim.tools.ulog_motor_tui.analyze_log", lambda selected, output: result)

    assert run_tui([log_path], output_dir) == result
    assert screen.getch_calls == 1
