"""Common environment interface used by all simulator backends."""

from abc import ABC, abstractmethod

from acesim.config.config_loader import ConfigLoader


class BaseEnv(ABC):
    """Minimal lifecycle contract shared by all ACESim environments."""

    def __init__(self, config_loader: ConfigLoader):
        """Store the resolved configuration loader for subclasses."""

        self._config_loader = config_loader

    @abstractmethod
    def run(self):
        """Run the environment's viewer or main loop."""

    @abstractmethod
    def close(self):
        """Release backend resources owned by the environment."""
