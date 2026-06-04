from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path

from acesim.config.config_loader import ConfigLoader
from acesim.core.play import make_env


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the ACESim MuJoCo frontend for core SITL.")
    parser.add_argument("--config", type=Path, default=None)
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--headless", action="store_true")
    mode_group.add_argument("--gui", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    shutdown_requested = False

    def _request_shutdown(_signum: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGTERM, _request_shutdown)
    config_loader = ConfigLoader(args.config) if args.config is not None else ConfigLoader()
    env = make_env(config_loader)
    ready_file = os.environ.get("ACESIM_SITL_READY_FILE")
    if ready_file:
        Path(ready_file).write_text("ready\n", encoding="utf-8")
    try:
        if args.gui:
            env.run()
        else:
            while not shutdown_requested:
                env.step()
    except KeyboardInterrupt:
        return 0
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
