import importlib
from dataclasses import asdict

from acesim.config.config_loader import ConfigLoader
from acesim.core.play import make_env

_BENCHMARK_GROUPS = {
    "multirotor": ["acesim.benchmark.multirotor.hover:HoverBenchmark"],
}


def run_benchmark():
    config_loader = ConfigLoader()
    benchmark_group = _BENCHMARK_GROUPS[config_loader.get_benchmark()]
    for benchmark_name in benchmark_group:
        module_name, class_name = benchmark_name.split(":", 1)
        module = importlib.import_module(module_name)
        cls = getattr(module, class_name)
        benchmark = cls(env=make_env())
        metrics = benchmark.run()
        benchmark.close()
        print(f"{benchmark_name}: {asdict(metrics)}")


if __name__ == "__main__":
    run_benchmark()
