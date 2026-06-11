import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from acesim.tools.sdf2urdf import (
    cleanup_manual_meshes_from_sdf,
    generate_manual_meshes_from_sdf,
    sync_manual_urdf_from_sdf,
)

from .asset_context import AssetPaths, AssetToolchainConfig
from .mesh_processing import clean_artifacts, process_urdf_collisions
from .mjcf_ops import euler_to_quat, fmt_floats, postprocess_xml
from .mujoco_compiler import compile_urdf_to_xml_with_available_backend, find_mujoco_compile_binary
from .runtime_handler_registry import runtime_handler_for_target
from .urdf_ops import calculate_min_z, parse_q0, preprocess_urdf
from .xml_ops import add_collision_exclusions, indent_xml, inject_xml, sort_attributes


class URDF2MJCFConverter:
    """Compile an existing URDF asset into runtime-ready MuJoCo MJCF.

    This stage intentionally consumes URDF as input. Any SDF truth sync happens
    before the compiler runs and is orchestrated at the workflow layer here,
    rather than inside lower-level MJCF helpers.
    """

    def __init__(
        self,
        target: str,
        floating: bool = False,
        decompose: bool = False,
        safety_margin: float = 0.05,
        q0: str = "",
        mujoco_bin: str | None = None,
        overwrite: bool = False,
    ):
        self.config = AssetToolchainConfig(
            target=target,
            floating=floating,
            decompose=decompose,
            safety_margin=safety_margin,
            q0=q0,
            mujoco_bin=mujoco_bin,
        )
        self.paths = AssetPaths.for_target(target)
        self.target = target
        self.floating = floating
        self.decompose = decompose
        self.safety_margin = safety_margin
        self.q0_str = q0
        self.mujoco_bin = mujoco_bin
        self.overwrite = overwrite
        self.base_dir = self.paths.base_dir
        self.urdf_path = self.paths.urdf_path
        self.mesh_dir = self.paths.mesh_dir
        self.xml_path = self.paths.xml_path
        self.initial_q = parse_q0(q0)
        self.family_handler = runtime_handler_for_target(target)

    @staticmethod
    def indent_xml(elem: ET.Element, level: int = 0) -> None:
        indent_xml(elem, level)

    @staticmethod
    def inject_xml(parent: ET.Element, xml_content: str, index: int = -1) -> None:
        inject_xml(parent, xml_content, index)

    @staticmethod
    def sort_attributes(elem: ET.Element) -> None:
        sort_attributes(elem)

    @staticmethod
    def add_collision_exclusions(root: ET.Element) -> None:
        add_collision_exclusions(root)

    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
        return euler_to_quat(roll, pitch, yaw)

    @staticmethod
    def _fmt_floats(values: list[float]) -> str:
        return fmt_floats(values)

    def clean_artifacts(self) -> None:
        clean_artifacts(self.paths)

    def process_urdf_collisions(
        self, urdf_path: str, mesh_dir: str, threshold: float = 0.2, resolution: int = 50
    ) -> str:
        return str(process_urdf_collisions(Path(urdf_path), Path(mesh_dir), threshold, resolution))

    def calculate_min_z(self, urdf_path: str) -> float:
        return calculate_min_z(Path(urdf_path), self.initial_q)

    def preprocess_urdf(self, urdf_path: str, height_offset: float = 0.0) -> str:
        return str(preprocess_urdf(Path(urdf_path), floating=self.floating, height_offset=height_offset))

    def postprocess_xml(self, xml_path: str, height_offset: float = 0.0) -> None:
        postprocess_xml(
            Path(xml_path),
            config=self.config,
            paths=self.paths,
            initial_q=self.initial_q,
            height_offset=height_offset,
        )

    def _find_mujoco_binary(self) -> str:
        return str(find_mujoco_compile_binary(self.config))

    def _confirm_overwrite(self) -> None:
        if not self.xml_path.exists():
            return
        if self.overwrite:
            self.clean_artifacts()
            return
        try:
            choice = input(f"Output file {self.xml_path} already exists. Overwrite? [y/N]: ").strip().lower()
        except EOFError as exc:
            raise RuntimeError(f"Output file {self.xml_path} already exists. Non-interactive mode detected.") from exc
        if choice != "y":
            raise RuntimeError("Operation aborted by user.")
        self.clean_artifacts()

    def run(self) -> None:
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF not found at {self.urdf_path}")

        self._confirm_overwrite()

        if self.mesh_dir.exists():
            # The SDF stage materializes source-driven meshes first, then the
            # runtime handler prepares any MJCF-only meshes for the target family.
            generate_manual_meshes_from_sdf(self.config, self.paths)
            sync_manual_urdf_from_sdf(self.config, self.paths)
            self.family_handler.prepare_runtime_assets(self.config, self.paths)

        processing_urdf_path = self.urdf_path
        if self.decompose:
            processing_urdf_path = process_urdf_collisions(self.urdf_path, self.mesh_dir)

        print("Calculating auto-height...")
        min_z = calculate_min_z(processing_urdf_path, self.initial_q)
        height_offset = -min_z + self.safety_margin
        print(f"Lowest point: {min_z:.4f}m. Applied offset: {height_offset:.4f}m (margin: {self.safety_margin}m)")

        tmp_urdf = preprocess_urdf(processing_urdf_path, floating=self.floating, height_offset=height_offset)
        print(f"Compiling to {self.xml_path}...")

        try:
            compile_urdf_to_xml_with_available_backend(self.config, tmp_urdf, self.xml_path)
        finally:
            tmp_urdf.unlink(missing_ok=True)
            if self.decompose and processing_urdf_path != self.urdf_path:
                processing_urdf_path.unlink(missing_ok=True)

        postprocess_xml(
            self.xml_path,
            config=self.config,
            paths=self.paths,
            initial_q=self.initial_q,
            height_offset=height_offset,
        )
        self.family_handler.cleanup_runtime_assets(self.config, self.paths)
        cleanup_manual_meshes_from_sdf(self.config, self.paths)
        print("\nCompilation and post-processing complete.")


def main(argv: list[str] | None = None) -> int:
    raw_argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description="Compile URDF to MuJoCo XML using URDF2MJCFConverter.")
    parser.add_argument("--tui", action="store_true", help="Launch the interactive terminal UI.")
    parser.add_argument("--target", type=str, default="ace_leader", help="Target robot name.")
    parser.add_argument("--mujoco-bin", type=str, default=None, help="Path to compile.exe.")
    parser.add_argument("--floating", action="store_true", help="Add floating joint.")
    parser.add_argument("--decompose", action="store_true", help="Perform convex decomposition.")
    parser.add_argument("--safety-margin", type=float, default=0.05, help="Extra height margin.")
    parser.add_argument("--q0", type=str, default="", help="Initial joint positions (key=val,key=val).")

    args = parser.parse_args(raw_argv)
    if args.tui or not raw_argv:
        from . import tui

        return tui.main()

    converter = URDF2MJCFConverter(
        target=args.target,
        floating=args.floating,
        decompose=args.decompose,
        safety_margin=args.safety_margin,
        q0=args.q0,
        mujoco_bin=args.mujoco_bin,
    )

    try:
        converter.run()
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0
