#!/usr/bin/env python3
"""Validate Conventional Commit headers with a required scope."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ALLOWED_TYPES = (
    "build",
    "chore",
    "ci",
    "docs",
    "feat",
    "fix",
    "perf",
    "refactor",
    "revert",
    "style",
    "test",
)

HEADER_RE = re.compile(rf"^({'|'.join(ALLOWED_TYPES)})\([a-z0-9][a-z0-9._-]*\)!?: .+$")


def _first_non_comment_line(message_path: Path) -> str:
    for line in message_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped and not stripped.startswith("#"):
            return stripped
    return ""


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("Usage: check_commit_msg.py <commit-msg-file>", file=sys.stderr)
        return 2

    header = _first_non_comment_line(Path(argv[1]))
    if HEADER_RE.match(header):
        return 0

    print("Commit message must follow: type(scope): description", file=sys.stderr)
    print(f"Allowed types: {', '.join(ALLOWED_TYPES)}", file=sys.stderr)
    print("Scope is required and may contain lowercase letters, digits, '.', '_' or '-'.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
