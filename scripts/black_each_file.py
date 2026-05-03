#!/usr/bin/env python3
"""Run Black one file at a time for pre-commit.

Black 26.1.0 can hang in this environment when a single invocation receives
multiple files. Per-file invocations keep the same formatter and arguments while
avoiding that batch path.
"""

from __future__ import annotations

import subprocess
import sys


def main() -> int:
    args = sys.argv[1:]
    black_args: list[str] = []
    files: list[str] = []

    for arg in args:
        if arg.endswith((".py", ".pyi")):
            files.append(arg)
        else:
            black_args.append(arg)

    for file_path in files:
        subprocess.run([sys.executable, "-m", "black", "--quiet", *black_args, file_path], check=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
