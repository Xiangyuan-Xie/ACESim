import importlib
from typing import Optional

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv


def make_env(config_loader: Optional[ConfigLoader] = None) -> BaseEnv:
    if not config_loader:
        config_loader = ConfigLoader()
    module_name, class_name = config_loader.get_sim_info()
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    return cls(config_loader)


if __name__ == "__main__":
    env: BaseEnv = make_env()
    try:
        env.run()
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
