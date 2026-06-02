import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from .asset_context import AssetToolchainConfig


def find_mujoco_compile_binary(config: AssetToolchainConfig) -> Path:
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


def write_python_mujoco_compatible_urdf(urdf_path: Path, *, preserve_static_root: bool = True) -> Path:
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    if preserve_static_root:
        mujoco_elem = root.find("mujoco")
        if mujoco_elem is None:
            mujoco_elem = ET.Element("mujoco")
            root.insert(0, mujoco_elem)
        compiler_elem = mujoco_elem.find("compiler")
        if compiler_elem is None:
            compiler_elem = ET.SubElement(mujoco_elem, "compiler")
        compiler_elem.set("fusestatic", "false")

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename and filename.startswith("meshes/"):
            mesh.set("filename", filename.removeprefix("meshes/"))

    compatible_urdf_path = urdf_path.with_name(f"{urdf_path.stem}_python_mujoco_tmp{urdf_path.suffix}")
    tree.write(compatible_urdf_path, encoding="utf-8", xml_declaration=True)
    return compatible_urdf_path


def compile_urdf_to_xml_with_python(urdf_path: Path, xml_path: Path, *, preserve_static_root: bool = True) -> None:
    import mujoco

    compatible_urdf_path = write_python_mujoco_compatible_urdf(
        urdf_path,
        preserve_static_root=preserve_static_root,
    )
    try:
        model = mujoco.MjModel.from_xml_path(str(compatible_urdf_path))
        mujoco.mj_saveLastXML(str(xml_path), model)
    finally:
        compatible_urdf_path.unlink(missing_ok=True)


def compile_urdf_to_xml_with_available_backend(
    config: AssetToolchainConfig,
    urdf_path: Path,
    xml_path: Path,
) -> None:
    try:
        binary = find_mujoco_compile_binary(config)
    except FileNotFoundError:
        if config.mujoco_bin:
            raise
        compile_urdf_to_xml_with_python(urdf_path, xml_path, preserve_static_root=not config.floating)
        return

    compile_urdf_to_xml(binary, urdf_path, xml_path)
