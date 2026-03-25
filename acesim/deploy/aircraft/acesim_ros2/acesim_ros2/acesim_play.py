from __future__ import annotations

from acesim.core.play import make_env
from acesim.env.base_env import BaseEnv


def main() -> None:
    env: BaseEnv = make_env()
    try:
        env.run()
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
