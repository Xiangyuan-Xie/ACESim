from abc import ABC, abstractmethod

from acesim.config.config_loader import ConfigLoader


class BaseEnv(ABC):
    # === Lifecycle Interface ===
    def __init__(self, config_loader: ConfigLoader):
        self._config_loader = config_loader

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def close(self):
        pass
