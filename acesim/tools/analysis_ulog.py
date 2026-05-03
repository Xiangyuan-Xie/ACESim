from __future__ import annotations

import argparse
import curses
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Sequence

import numpy as np
from pyulog import ULog

DEFAULT_LOG_GLOB = "acesim/third_party/aircraft/PX4-Autopilot/" "build/px4_sitl_default/rootfs/log/**/*.ulg"
DEFAULT_OUTPUT_DIR = "analysis_outputs"

NAV_STATE_NAMES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    10: "ACRO",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
    23: "EXTERNAL1",
    24: "EXTERNAL2",
    25: "EXTERNAL3",
    26: "EXTERNAL4",
    27: "EXTERNAL5",
    28: "EXTERNAL6",
    29: "EXTERNAL7",
    30: "EXTERNAL8",
}


@dataclass(frozen=True)
class AnalysisResult:
    log_path: Path
    figure_path: Path
    summary_path: Path
    am_active_detected: bool
    am_warning_count: int


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def discover_logs(pattern: str = DEFAULT_LOG_GLOB, root: Path | None = None) -> list[Path]:
    base = project_root() if root is None else root
    logs = sorted(base.glob(pattern), key=lambda path: path.stat().st_mtime, reverse=True)
    return [path.resolve() for path in logs if path.is_file()]


def format_log_row(index: int, log_path: Path, modified_time: float | None = None) -> str:
    timestamp = log_path.stat().st_mtime if modified_time is None else modified_time
    modified = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")
    return f"{index:3d}  {log_path.name}  {modified}"


def build_nav_intervals(
    *,
    start_timestamp: int,
    last_timestamp: int,
    timestamps: np.ndarray,
    nav_states: np.ndarray,
) -> list[tuple[float, float, int]]:
    relative_times = (np.asarray(timestamps, dtype=np.float64) - float(start_timestamp)) / 1e6
    states = np.asarray(nav_states, dtype=int)
    valid = relative_times >= 0.0
    if not np.any(valid):
        return []

    times = relative_times[valid]
    states = states[valid]
    change_indices = [0]
    for idx in range(1, len(states)):
        if states[idx] != states[idx - 1]:
            change_indices.append(idx)

    log_end_s = (float(last_timestamp) - float(start_timestamp)) / 1e6
    intervals: list[tuple[float, float, int]] = []
    for pos, idx in enumerate(change_indices):
        start_s = float(times[idx])
        end_s = float(times[change_indices[pos + 1]]) if pos + 1 < len(change_indices) else log_end_s
        intervals.append((start_s, end_s, int(states[idx])))
    return intervals


def summarize_motor_window(
    label: str,
    times_s: np.ndarray,
    controls: np.ndarray,
    mask: np.ndarray,
) -> list[str]:
    lines = [f"{label}:"]
    if not np.any(mask):
        lines.append("  no samples")
        return lines

    selected = controls[mask, :]
    for motor_idx in range(selected.shape[1]):
        values = selected[:, motor_idx]
        lines.append(
            f"  motor {motor_idx}: mean={np.nanmean(values):.6f}, "
            f"min={np.nanmin(values):.6f}, max={np.nanmax(values):.6f}, "
            f"samples={len(values)}"
        )
    lines.append(
        f"  all motors: mean={np.nanmean(selected):.6f}, "
        f"min={np.nanmin(selected):.6f}, max={np.nanmax(selected):.6f}"
    )
    return lines


def _topic(ulog: ULog, name: str):
    for dataset in ulog.data_list:
        if dataset.name == name:
            return dataset
    raise KeyError(f"ULog does not contain topic '{name}'")


def _relative_seconds(timestamps: np.ndarray, start_timestamp: int) -> np.ndarray:
    return (np.asarray(timestamps, dtype=np.float64) - float(start_timestamp)) / 1e6


def _safe_output_stem(log_path: Path) -> str:
    date_part = log_path.parent.name if log_path.parent.name else "ulog"
    return f"{date_part}_{log_path.stem}".replace(os.sep, "_")


def analyze_log(log_path: Path, output_dir: Path) -> AnalysisResult:
    import matplotlib

    matplotlib.use("Agg")

    ulog = ULog(str(log_path))
    motor_topic = _topic(ulog, "actuator_motors")
    vehicle_status = _topic(ulog, "vehicle_status")

    times_s = _relative_seconds(motor_topic.data["timestamp"], ulog.start_timestamp)
    controls = np.vstack([np.asarray(motor_topic.data[f"control[{idx}]"], dtype=np.float64) for idx in range(4)]).T

    intervals = build_nav_intervals(
        start_timestamp=ulog.start_timestamp,
        last_timestamp=ulog.last_timestamp,
        timestamps=vehicle_status.data["timestamp"],
        nav_states=vehicle_status.data["nav_state"],
    )
    am_active_detected = any(state == 14 or 23 <= state <= 30 for _, _, state in intervals)
    am_warning_times = [
        (message.timestamp - ulog.start_timestamp) / 1e6
        for message in ulog.logged_messages
        if "AM Position Offboard requires fresh supported offboard_control_mode" in message.message
    ]

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = _safe_output_stem(log_path)
    figure_path = output_dir / f"{stem}_actuator_motors_nav_states.png"
    summary_path = output_dir / f"{stem}_motor_summary.txt"

    _plot_motor_controls(
        figure_path=figure_path,
        log_path=log_path,
        times_s=times_s,
        controls=controls,
        intervals=intervals,
        warning_times_s=am_warning_times,
    )
    summary_path.write_text(
        "\n".join(
            _summary_lines(
                log_path=log_path,
                duration_s=(ulog.last_timestamp - ulog.start_timestamp) / 1e6,
                intervals=intervals,
                am_active_detected=am_active_detected,
                am_warning_count=len(am_warning_times),
                times_s=times_s,
                controls=controls,
            )
        )
        + "\n",
        encoding="utf-8",
    )

    return AnalysisResult(
        log_path=log_path,
        figure_path=figure_path,
        summary_path=summary_path,
        am_active_detected=am_active_detected,
        am_warning_count=len(am_warning_times),
    )


def _plot_motor_controls(
    *,
    figure_path: Path,
    log_path: Path,
    times_s: np.ndarray,
    controls: np.ndarray,
    intervals: Sequence[tuple[float, float, int]],
    warning_times_s: Sequence[float],
) -> None:
    import matplotlib.pyplot as plt

    colors = {2: "#DDEEFF", 17: "#FFF3C4", 4: "#E7F6DF", 14: "#F5D0E0"}
    fig, ax = plt.subplots(figsize=(12, 6), dpi=160)

    for start_s, end_s, state in intervals:
        ax.axvspan(start_s, end_s, color=colors.get(state, "#EEEEEE"), alpha=0.55, linewidth=0)
        if end_s - start_s > 1.2:
            ax.text(
                (start_s + end_s) / 2,
                0.96,
                NAV_STATE_NAMES.get(state, str(state)),
                ha="center",
                va="top",
                fontsize=9,
                color="#333333",
                transform=ax.get_xaxis_transform(),
            )

    for motor_idx in range(controls.shape[1]):
        ax.plot(times_s, controls[:, motor_idx], linewidth=1.6, label=f"motor {motor_idx} control[{motor_idx}]")

    if warning_times_s:
        ax.vlines(
            warning_times_s,
            ymin=0.0,
            ymax=0.08,
            colors="#B3261E",
            alpha=0.35,
            linewidth=0.8,
            label="AM Position offboard-check warnings",
        )

    ax.set_title(f"PX4 actuator_motors normalized thrust - {log_path.name}")
    ax.set_xlabel("time since log start [s]")
    ax.set_ylabel("normalized motor thrust [0..1]")
    ax.set_xlim(0, max(times_s) if len(times_s) else 1.0)
    ax.set_ylim(-0.03, 1.03)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(figure_path)
    plt.close(fig)


def _summary_lines(
    *,
    log_path: Path,
    duration_s: float,
    intervals: Sequence[tuple[float, float, int]],
    am_active_detected: bool,
    am_warning_count: int,
    times_s: np.ndarray,
    controls: np.ndarray,
) -> list[str]:
    lines = [
        f"ULog: {log_path}",
        f"Duration: {duration_s:.3f} s",
        "",
        "Detected vehicle_status nav_state intervals:",
    ]
    if intervals:
        for start_s, end_s, state in intervals:
            lines.append(f"  {start_s:8.3f} - {end_s:8.3f} s: {NAV_STATE_NAMES.get(state, str(state))} ({state})")
    else:
        lines.append("  none")

    lines.extend(
        [
            "",
            "AM Position evidence:",
            "  AM nav_state detected: " + ("yes" if am_active_detected else "no"),
            f"  AM Position offboard-check warnings: {am_warning_count}",
            "",
        ]
    )
    if not am_active_detected:
        lines.append("Note: no OFFBOARD (14) or EXTERNAL1..8 (23..30) nav_state interval was detected.")
        lines.append("Strictly speaking, there is no confirmed AM Position active window in this log.")
        lines.append("")

    lines.extend(
        summarize_motor_window("Full log actuator_motors control[0..3]", times_s, controls, np.isfinite(times_s))
    )
    lines.extend(
        summarize_motor_window("Armed/takeoff-through-end window (t >= 11.192 s)", times_s, controls, times_s >= 11.192)
    )
    lines.extend(
        summarize_motor_window(
            "AUTO_TAKEOFF + AUTO_LOITER window (11.192 <= t < 29.192 s)",
            times_s,
            controls,
            (times_s >= 11.192) & (times_s < 29.192),
        )
    )
    lines.extend(summarize_motor_window("POSCTL after 29.192 s window", times_s, controls, times_s >= 29.192))
    return lines


def run_tui(logs: Sequence[Path], output_dir: Path) -> AnalysisResult | None:
    if not logs:
        raise FileNotFoundError(f"No .ulg logs found with default pattern: {DEFAULT_LOG_GLOB}")

    selected: dict[str, AnalysisResult | None] = {"result": None}

    def main(stdscr) -> None:
        curses.curs_set(0)
        index = 0
        offset = 0
        status = "Enter: analyze  Up/Down/PgUp/PgDn/Home/End: select  q: quit"

        while True:
            stdscr.erase()
            height, width = stdscr.getmaxyx()
            visible_rows = max(1, height - 4)
            offset = min(offset, max(0, len(logs) - visible_rows))
            if index < offset:
                offset = index
            elif index >= offset + visible_rows:
                offset = index - visible_rows + 1

            stdscr.addnstr(0, 0, "PX4 ULog motor-thrust analyzer", width - 1, curses.A_BOLD)
            stdscr.addnstr(1, 0, status, width - 1)

            for row, log_path in enumerate(logs[offset : offset + visible_rows], start=3):
                item_index = offset + row - 3
                display = format_log_row(item_index + 1, log_path)
                attr = curses.A_REVERSE if item_index == index else curses.A_NORMAL
                stdscr.addnstr(row, 0, display, width - 1, attr)

            key = stdscr.getch()
            if key in (ord("q"), ord("Q"), 27):
                return
            if key in (curses.KEY_UP, ord("k")):
                index = max(0, index - 1)
            elif key in (curses.KEY_DOWN, ord("j")):
                index = min(len(logs) - 1, index + 1)
            elif key == curses.KEY_PPAGE:
                index = max(0, index - visible_rows)
            elif key == curses.KEY_NPAGE:
                index = min(len(logs) - 1, index + visible_rows)
            elif key == curses.KEY_HOME:
                index = 0
            elif key == curses.KEY_END:
                index = len(logs) - 1
            elif key in (curses.KEY_ENTER, 10, 13):
                status = f"Analyzing {logs[index].name} ..."
                stdscr.addnstr(1, 0, status.ljust(width - 1), width - 1)
                stdscr.refresh()
                selected["result"] = analyze_log(logs[index], output_dir)
                result = selected["result"]
                assert result is not None
                status = f"Done: {result.figure_path}"
                stdscr.addnstr(1, 0, status.ljust(width - 1), width - 1)
                stdscr.refresh()
                return

    curses.wrapper(main)
    return selected["result"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze PX4 ULog normalized motor thrust with an optional curses picker."
    )
    parser.add_argument("log", nargs="?", type=Path, help="Analyze this .ulg directly. Omit to open the curses picker.")
    parser.add_argument("--output-dir", type=Path, default=project_root() / DEFAULT_OUTPUT_DIR)
    parser.add_argument("--glob", default=DEFAULT_LOG_GLOB, help="Log glob used by the curses picker.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.log is not None:
        result = analyze_log(args.log.resolve(), args.output_dir.resolve())
        print(f"Figure: {result.figure_path}")
        print(f"Summary: {result.summary_path}")
        if not result.am_active_detected:
            print("Note: no confirmed AM Position active nav_state was detected in this log.")
        return

    tui_result = run_tui(discover_logs(args.glob), args.output_dir.resolve())
    if tui_result is not None:
        print(f"Figure: {tui_result.figure_path}")
        print(f"Summary: {tui_result.summary_path}")


if __name__ == "__main__":
    main()
