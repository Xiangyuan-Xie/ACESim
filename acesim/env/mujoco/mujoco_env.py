from pathlib import Path

import mujoco
import mujoco.viewer

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv


class MujocoEnv(BaseEnv):
    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        asset_name = self._config_loader.get_asset_name()
        asset_path = str((Path(__file__).parent / "description" / asset_name / f"{asset_name}.xml").resolve())
        self._mj_model = mujoco.MjModel.from_xml_path(asset_path)
        self._mj_data = mujoco.MjData(self._mj_model)
        self._mj_model.opt.timestep = 0.001
        mujoco.mj_resetData(self._mj_model, self._mj_data)

        self._simulation_time_us = 0
        self._step_count = 0

    def run(self, visualize=True):
        if visualize:
            mujoco.set_mjcb_control(self.control)
            mujoco.viewer.launch(self._mj_model, self._mj_data)
        else:
            while True:
                mujoco.set_mjcb_control(self.control)
                mujoco.mj_step(self._mj_model, self._mj_data)

    def close(self):
        pass
