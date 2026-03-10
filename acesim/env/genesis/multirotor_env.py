import platform

from acesim.config.config_loader import ConfigLoader
from acesim.env.genesis.genesis_env import GenesisEnv
from acesim.utils.px4_interface import PX4Interface


class MultirotorEnv(GenesisEnv):
    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        self._px4_interface = PX4Interface()
        if platform.system() == "Windows":
            print("[ACESim] Genesis backend initialized on Windows.")
        else:
            print("[ACESim] Genesis backend initialized on Linux.")

    def step(self):
        if not self._px4_interface.is_connected:
            self._px4_interface.update_connection_state()
        super().step()
