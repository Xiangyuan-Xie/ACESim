import subprocess
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib


ROOT = Path(__file__).resolve().parents[1]

PX4_ARM2X_MESSAGES = [
    "AmPosControlStatus.msg",
    "AmPolicyObservation.msg",
    "AmTestStatus.msg",
    "AmTestResult.msg",
]


def test_setup_py_exposes_project_metadata_for_legacy_setuptools() -> None:
    result = subprocess.run(
        [sys.executable, "setup.py", "--name", "--version"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.splitlines() == ["acesim", "0.1.0"]


def test_build_system_accepts_ubuntu_22_setuptools() -> None:
    build_system = tomllib.loads((ROOT / "pyproject.toml").read_text())["build-system"]

    assert "setuptools>=59.6.0" in build_system["requires"]


def test_pyproject_package_discovery_excludes_third_party_sources() -> None:
    setuptools_config = tomllib.loads((ROOT / "pyproject.toml").read_text())["tool"]["setuptools"]

    assert setuptools_config["packages"]["find"]["include"] == ["acesim", "acesim.*"]
    assert setuptools_config["packages"]["find"]["exclude"] == [
        "acesim.third_party",
        "acesim.third_party.*",
    ]


def test_sim_backends_are_optional_pyproject_dependencies() -> None:
    project_config = tomllib.loads((ROOT / "pyproject.toml").read_text())["project"]

    assert "mujoco" not in project_config["dependencies"]
    assert "genesis-world" not in project_config["dependencies"]
    assert project_config["optional-dependencies"]["mujoco"] == ["mujoco", "trimesh"]
    assert project_config["optional-dependencies"]["genesis"] == ["genesis-world"]
    assert project_config["optional-dependencies"]["all"] == ["mujoco", "genesis-world", "trimesh"]


def test_x500_arm2x_px4_messages_are_vendored() -> None:
    source_dir = ROOT / "acesim/third_party/aircraft/PX4-Autopilot/msg"
    vendored_dir = ROOT / "acesim/deploy/aircraft/px4_msgs/msg"

    for message_name in PX4_ARM2X_MESSAGES:
        source = source_dir / message_name
        vendored = vendored_dir / message_name

        assert vendored.exists()
        assert vendored.read_text() == source.read_text()


def test_acesim_ros2_exposes_x500_arm2x_benchmark_console_script() -> None:
    setup_py = ROOT / "acesim/deploy/aircraft/acesim_ros2/setup.py"

    assert "x500_arm2x_benchmark = acesim_ros2.benchmark.x500_arm2x:main" in setup_py.read_text()


def test_acesim_ros2_exposes_ace_follower_console_script() -> None:
    setup_py = ROOT / "acesim/deploy/aircraft/acesim_ros2/setup.py"

    assert "acesim_ace_follower = acesim_ros2.ace_follower:main" in setup_py.read_text()


def test_acesim_ros2_does_not_keep_legacy_x500_arm2x_benchmark_module() -> None:
    legacy_module = ROOT / "acesim/deploy/aircraft/acesim_ros2/acesim_ros2/x500_arm2x_benchmark.py"

    assert not legacy_module.exists()


def test_acesim_ros2_installs_launch_and_config_globs() -> None:
    setup_text = (ROOT / "acesim/deploy/aircraft/acesim_ros2/setup.py").read_text()

    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text


def test_acesim_ros2_does_not_keep_unused_px4_sim_config() -> None:
    config_path = ROOT / "acesim/deploy/aircraft/acesim_ros2/config/px4_sim_config.yaml"

    assert not config_path.exists()


def test_acesim_core_does_not_keep_broken_benchmark_entrypoint() -> None:
    benchmark_path = ROOT / "acesim/core/benchmark.py"

    assert not benchmark_path.exists()


def test_dynamic_params_tool_is_import_safe() -> None:
    result = subprocess.run(
        [sys.executable, "-c", "import acesim.tools.cal_dynamic_params"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout == ""
