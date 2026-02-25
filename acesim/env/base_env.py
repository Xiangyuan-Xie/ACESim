from abc import ABC, abstractmethod

from acesim.config.config_loader import ConfigLoader


class BaseEnv(ABC):
    def __init__(self, config_loader: ConfigLoader):
        self._config_loader = config_loader

    @abstractmethod
    def run(self, visualize=True):
        pass

    @abstractmethod
    def close(self):
        pass
