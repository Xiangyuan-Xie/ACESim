from __future__ import annotations

import re
from pathlib import Path


def _wsl_windows_host_ip() -> str:
    resolv_conf = Path("/etc/resolv.conf")
    if not resolv_conf.exists():
        return "127.0.0.1"

    try:
        for raw_line in resolv_conf.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line.startswith("nameserver"):
                continue
            parts = re.split(r"\s+", line)
            if len(parts) >= 2:
                return parts[1]
    except OSError:
        pass

    return "127.0.0.1"


def resolve_endpoint(mode: str, port: int) -> str:
    if mode == "linux":
        host = "127.0.0.1"
    else:
        host = _wsl_windows_host_ip()
    return f"tcp://{host}:{port}"
