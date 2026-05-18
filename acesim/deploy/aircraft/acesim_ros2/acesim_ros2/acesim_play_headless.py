from __future__ import annotations

import signal

from acesim.core.play import make_env
from acesim.env.base_env import BaseEnv


class ShutdownRequested(Exception):
    """Raised by the SIGTERM handler so launch shutdown exits quietly."""


_shutdown_requested = False


def _request_shutdown(_signum: int, _frame: object) -> None:
    global _shutdown_requested
    _shutdown_requested = True


def main() -> int:
    global _shutdown_requested
    _shutdown_requested = False
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)
    env: BaseEnv | None = None
    try:
        env = make_env()
        while not _shutdown_requested:
            env.step()
    except (KeyboardInterrupt, ShutdownRequested):
        _shutdown_requested = True
    finally:
        if env is not None:
            try:
                env.close()
            except (KeyboardInterrupt, ShutdownRequested):
                _shutdown_requested = True
    if _shutdown_requested:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
