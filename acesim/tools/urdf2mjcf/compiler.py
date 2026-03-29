import os
import shutil
import subprocess
from pathlib import Path

from .config import ConverterConfig


def find_mujoco_compile_binary(config: ConverterConfig) -> Path:
    if config.mujoco_bin:
        binary = Path(config.mujoco_bin)
        if binary.exists():
            return binary
        raise FileNotFoundError(f"Specified MuJoCo compile binary not found: {binary}")

    bin_path = shutil.which("compile")
    if bin_path:
        return Path(bin_path)

    home = os.environ.get("HOME", "/root")
    candidates = [
        Path(home) / ".mujoco" / "mujoco210" / "bin" / "compile",
        Path(home) / ".mujoco" / "bin" / "compile",
        Path("/usr/local/bin/compile"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        "MuJoCo 'compile' binary not found. Checked PATH and: " + ", ".join(str(candidate) for candidate in candidates)
    )


def compile_urdf_to_xml(binary: Path, urdf_path: Path, xml_path: Path) -> None:
    subprocess.check_call([str(binary), str(urdf_path), str(xml_path)])
