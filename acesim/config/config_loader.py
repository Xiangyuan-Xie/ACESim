from pathlib import Path
from typing import Any, Dict, Tuple

import tomli

_ENV_MAP = {
    "mujoco": {
        "mc_arm": ("acesim.env.mujoco.mc_arm_env", "MCArmEnv"),
    },
}

__all__ = ["ConfigLoader"]


class ConfigLoader:
    def __init__(self, config_path: Path = Path(__file__).parent / "default.toml"):
        config_path = Path(config_path).expanduser().resolve()
        with open(config_path, "rb") as f:
            self._config = tomli.load(f)

        asset_config_path = config_path.parent / self.get_sim_type() / f"{self.get_asset_name()}.toml"
        with open(asset_config_path, "rb") as f:
            self._asset_config = tomli.load(f)

    def get_sim_type(self) -> str:
        return self._config["basic"]["sim_type"]

    def get_env_type(self) -> str:
        return self._config["basic"]["env_type"]

    def get_sim_info(self) -> Tuple[str, str]:
        sim_type = self.get_sim_type()
        env_type = self.get_env_type()
        if sim_type in _ENV_MAP:
            if env_type in _ENV_MAP[sim_type]:
                return _ENV_MAP[sim_type][env_type]
            else:
                raise ValueError(f"Env type '{env_type}' not supported for sim type '{sim_type}'")
        else:
            raise ValueError(f"Sim type '{sim_type}' not supported")

    def get_scene_name(self) -> str:
        return self._config["basic"]["scene_name"]

    def get_asset_name(self) -> str:
        return self._config["basic"]["asset_name"]

    def get_asset_params(self) -> Dict[str, Any]:
        return self._asset_config["params"]

    def get_benchmark(self) -> str:
        return self._config["basic"]["benchmark"]
