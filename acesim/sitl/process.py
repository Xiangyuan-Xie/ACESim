from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping


@dataclass(frozen=True)
class ProcessSpec:
    name: str
    command: list[str]
    cwd: Path | None = None
    env: dict[str, str] = field(default_factory=dict)


class ProcessSupervisor:
    def __init__(self) -> None:
        self._processes: list[tuple[ProcessSpec, subprocess.Popen[bytes]]] = []

    def start(self, spec: ProcessSpec) -> subprocess.Popen[bytes]:
        env = os.environ.copy()
        env.update(spec.env)
        print(f"[{spec.name}] starting: {format_process_command_for_log(spec.command)}", flush=True)
        process = subprocess.Popen(spec.command, cwd=spec.cwd, env=env)
        self._processes.append((spec, process))
        return process

    def terminate_all(self, timeout_sec: float = 5.0) -> None:
        for spec, process in reversed(self._processes):
            if process.poll() is None:
                print(f"[{spec.name}] stopping", flush=True)
                process.send_signal(signal.SIGTERM)
        deadline = time.monotonic() + timeout_sec
        for spec, process in reversed(self._processes):
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                print(f"[{spec.name}] killing", flush=True)
                process.kill()
        self._processes.clear()

    def poll_exited(self) -> tuple[ProcessSpec, int] | None:
        for spec, process in self._processes:
            returncode = process.poll()
            if returncode is not None:
                return spec, int(returncode)
        return None

    def wait_for_any_exit(self, poll_period_sec: float = 0.2) -> tuple[ProcessSpec, int]:
        while True:
            exited = self.poll_exited()
            if exited is not None:
                return exited
            time.sleep(poll_period_sec)


def build_graceful_shutdown_command(command: str, *, filter_px4_prompt: bool = False) -> list[str]:
    output_filter_setup = ""
    child_output_redirect = ""
    output_filter_cleanup = ""
    if filter_px4_prompt:
        filter_script = (
            "import os, re, signal\n"
            "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
            "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            "ansi_pattern = re.compile(rb'\\x1b\\[[0-9;?]*[ -/]*[@-~]')\n"
            "prompt_pattern = re.compile(rb'pxh>\\s?')\n"
            "buffer = b''\n"
            "def clean(chunk):\n"
            "    chunk = ansi_pattern.sub(b'', chunk)\n"
            "    chunk = chunk.replace(b'\\r', b'')\n"
            "    return prompt_pattern.sub(b'', chunk)\n"
            "while True:\n"
            "    data = os.read(0, 4096)\n"
            "    if not data:\n"
            "        break\n"
            "    buffer += data\n"
            "    lines = buffer.split(b'\\n')\n"
            "    for line in lines[:-1]:\n"
            "        line = clean(line + b'\\n')\n"
            "        if line.strip():\n"
            "            os.write(1, line)\n"
            "    buffer = lines[-1]\n"
            "    if len(buffer) > 65536:\n"
            "        buffer = clean(buffer)\n"
            "        if buffer.strip():\n"
            "            os.write(1, buffer)\n"
            "        buffer = b''\n"
            "buffer = clean(buffer)\n"
            "if buffer.strip():\n"
            "    os.write(1, buffer)\n"
        )
        output_filter_setup = (
            "_filter_dir=$(mktemp -d)\n"
            '_filter_pipe="$_filter_dir/px4-output"\n'
            'mkfifo "$_filter_pipe"\n'
            f'python3 -c {shlex.quote(filter_script)} < "$_filter_pipe" &\n'
            "_filter_pid=$!\n"
        )
        child_output_redirect = ' > "$_filter_pipe" 2>&1'
        output_filter_cleanup = (
            'wait "$_filter_pid" 2>/dev/null || true\n' 'rm -rf "$_filter_dir" 2>/dev/null || true\n'
        )
    script = (
        "_signal_received=0\n"
        f"{output_filter_setup}"
        "_cleanup_after_signal() {\n"
        "  _signal_received=1\n"
        "  _sig=$1\n"
        '  if [ "$_sig" = INT ]; then\n'
        '    kill -INT -- "-$_child_pid" 2>/dev/null || true\n'
        "  else\n"
        '    kill -TERM -- "-$_child_pid" 2>/dev/null || true\n'
        "  fi\n"
        "  for _ in 1 2 3 4 5; do\n"
        '    kill -0 -- "-$_child_pid" 2>/dev/null || return 0\n'
        "    sleep 0.2\n"
        "  done\n"
        '  kill -TERM -- "-$_child_pid" 2>/dev/null || true\n'
        "  for _ in 1 2 3 4 5; do\n"
        '    kill -0 -- "-$_child_pid" 2>/dev/null || return 0\n'
        "    sleep 0.2\n"
        "  done\n"
        '  kill -KILL -- "-$_child_pid" 2>/dev/null || true\n'
        "}\n"
        "_forward_sigint() { _cleanup_after_signal INT; }\n"
        "_forward_sigterm() { _cleanup_after_signal TERM; }\n"
        "trap _forward_sigint INT\n"
        "trap _forward_sigterm TERM\n"
        f'setsid bash -lc "$1"{child_output_redirect} &\n'
        "_child_pid=$!\n"
        'wait "$_child_pid"\n'
        "_status=$?\n"
        f"{output_filter_cleanup}"
        'if [ "$_signal_received" -eq 1 ]; then wait "$_child_pid" 2>/dev/null || true; exit 0; fi\n'
        'exit "$_status"'
    )
    return ["bash", "-lc", script, "_", command]


def format_process_command_for_log(command: list[str]) -> str:
    if len(command) >= 5 and command[:2] == ["bash", "-lc"] and command[3] == "_":
        return command[4]
    return " ".join(shlex.quote(token) for token in command)


def build_python_module_run_command(
    package: str,
    executable: str,
    additional_env: Mapping[str, str] | None = None,
    extra_args: list[str] | None = None,
) -> str:
    env = {"PYTHONUNBUFFERED": "1"}
    if additional_env:
        env.update(dict(additional_env))
    exports = " ".join(f"{name}={shlex.quote(value)}" for name, value in sorted(env.items()))
    args = " ".join(shlex.quote(arg) for arg in (extra_args or []))
    suffix = f" {args}" if args else ""
    return f"env {exports} ros2 run {shlex.quote(package)} {shlex.quote(executable)}{suffix}"
