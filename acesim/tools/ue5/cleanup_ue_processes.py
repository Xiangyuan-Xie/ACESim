#!/usr/bin/env python3
"""Safely clean up stale ACESim/Unreal helper processes for the current user."""

from __future__ import annotations

import argparse
import getpass
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

TARGET_COMMANDS = {
    "UnrealEditor",
    "UnrealEditor-Cmd",
    "ACESimUE",
    "UnrealTraceServer",
    "zenserver",
    "ShaderCompileWorker",
    "acesim_play_ue",
}


def _arg_tokens(args: str) -> list[str]:
    # ps has already flattened argv; split conservatively enough for executable names.
    return args.split()


def _is_target_process(command: str, args: str) -> bool:
    if command in TARGET_COMMANDS:
        return True

    tokens = _arg_tokens(args)
    if tokens and Path(tokens[0]).name in TARGET_COMMANDS:
        return True

    if command.startswith("python"):
        return any(Path(token).name in TARGET_COMMANDS for token in tokens[1:])

    return False


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    ppid: int
    user: str
    stat: str
    elapsed: str
    command: str
    args: str


def parse_ps_output(ps_output: str, *, current_user: str, current_pid: int) -> list[ProcessInfo]:
    processes: list[ProcessInfo] = []
    for raw_line in ps_output.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split(maxsplit=6)
        if len(parts) < 7:
            continue
        pid_text, ppid_text, user, stat, elapsed, command, args = parts
        try:
            pid = int(pid_text)
            ppid = int(ppid_text)
        except ValueError:
            continue
        if user != current_user or pid == current_pid:
            continue
        if _is_target_process(command, args):
            processes.append(ProcessInfo(pid, ppid, user, stat, elapsed, command, args))
    return processes


def find_processes(*, current_user: str | None = None) -> list[ProcessInfo]:
    user = current_user or getpass.getuser()
    output = subprocess.check_output(
        ["ps", "-eo", "pid,ppid,user,stat,etime,comm,args"],
        text=True,
        encoding="utf-8",
    )
    return parse_ps_output(output, current_user=user, current_pid=os.getpid())


def cleanup_processes(
    processes: list[ProcessInfo],
    *,
    dry_run: bool,
    wait_sec: float,
) -> dict[str, list[int]]:
    terminated: list[int] = []
    killed: list[int] = []
    if dry_run or not processes:
        return {"terminated": terminated, "killed": killed}

    for process in processes:
        try:
            os.kill(process.pid, signal.SIGTERM)
            terminated.append(process.pid)
        except ProcessLookupError:
            continue
    time.sleep(wait_sec)
    for process in processes:
        try:
            os.kill(process.pid, 0)
        except ProcessLookupError:
            continue
        os.kill(process.pid, signal.SIGKILL)
        killed.append(process.pid)
    return {"terminated": terminated, "killed": killed}


def main() -> None:
    parser = argparse.ArgumentParser(description="Clean stale ACESim UE/Unreal processes for the current user.")
    parser.add_argument("--dry-run", action="store_true", help="List matching processes without signalling them.")
    parser.add_argument("--wait-sec", type=float, default=3.0, help="Seconds to wait after SIGTERM before SIGKILL.")
    args = parser.parse_args()

    processes = find_processes()
    if not processes:
        print("No matching Unreal/ACESim UE processes are running.")
        return

    for process in processes:
        print(f"{process.pid}\t{process.elapsed}\t{process.command}\t{process.args}")
    result = cleanup_processes(processes, dry_run=args.dry_run, wait_sec=args.wait_sec)
    if args.dry_run:
        print(f"Dry run only; {len(processes)} process(es) would be cleaned.")
    else:
        print(f"Sent SIGTERM to: {result['terminated']}")
        print(f"Sent SIGKILL to: {result['killed']}")


if __name__ == "__main__":
    main()
