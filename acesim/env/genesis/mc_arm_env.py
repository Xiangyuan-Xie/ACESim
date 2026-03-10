from acesim.config.config_loader import ConfigLoader
from acesim.env.genesis.multirotor_env import MultirotorEnv


class MCArmEnv(MultirotorEnv):
    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
