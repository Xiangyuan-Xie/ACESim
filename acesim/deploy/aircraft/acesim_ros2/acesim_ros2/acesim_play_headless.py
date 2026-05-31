from __future__ import annotations

import argparse
import signal
from pathlib import Path

from acesim.config.config_loader import ConfigLoader
from acesim.core.play import make_env
from acesim.env.base_env import BaseEnv

_shutdown_requested = False


def _request_shutdown(_signum: int, _frame: object) -> None:
    global _shutdown_requested
    _shutdown_requested = True


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", help="Path to an ACESim config file.")
    return parser


def _parse_args() -> argparse.Namespace:
    return _build_arg_parser().parse_args()


def main() -> int:
    global _shutdown_requested
    args = _parse_args()
    _shutdown_requested = False
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)
    env: BaseEnv | None = None
    try:
        env = make_env(ConfigLoader(Path(args.config))) if args.config else make_env()
        while not _shutdown_requested:
            env.step()
    except KeyboardInterrupt:
        _shutdown_requested = True
    finally:
        if env is not None:
            try:
                env.close()
            except KeyboardInterrupt:
                _shutdown_requested = True
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
