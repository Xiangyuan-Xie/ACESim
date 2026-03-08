from abc import ABC, abstractmethod
from dataclasses import dataclass

from acesim.env.base_env import BaseEnv


@dataclass
class Metrics:
    total_time: float
    steps_per_second: float
    step_time_ms: float
    sim_rate: float


class BaseBenchmark(ABC):
    def __init__(self, env: BaseEnv):
        self.env = env
        self.steps = 0

    @abstractmethod
    def run(self) -> Metrics:
        pass

    def close(self):
        self.env.close()
