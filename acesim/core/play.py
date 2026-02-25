import importlib
from pathlib import Path
from typing import Optional

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv


def main(config_path: Optional[Path] = None):
    if config_path:
        config_loader = ConfigLoader(config_path=config_path)
    else:
        config_loader = ConfigLoader()

    module_name, class_name = config_loader.get_sim_info()
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    env: BaseEnv = cls(config_loader)

    try:
        env.run()
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
