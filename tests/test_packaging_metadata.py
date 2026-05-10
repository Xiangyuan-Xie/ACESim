import subprocess
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib


ROOT = Path(__file__).resolve().parents[1]


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
