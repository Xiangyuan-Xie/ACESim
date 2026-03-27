"""Base Genesis environment with shared clock and temporary merged MJCF."""

import tempfile
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import genesis as gs

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv
from acesim.utils.simulation_clock import SimulationClock


class GenesisEnv(BaseEnv):
    """Base class for Genesis environments used in ACESim.

    Genesis currently consumes MJCF assets, so this base class merges the
    MuJoCo scene and asset XML into a temporary file, owns the Genesis scene
    lifecycle, and exposes the same shared simulation clock API used by the
    MuJoCo backend.
    """

    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        gs.init(backend=gs.cpu, logging_level="warning")

        scene_name = self._config_loader.get_scene_name()
        asset_name = self._config_loader.get_asset_name()
        mujoco_root = Path(__file__).resolve().parents[1] / "mujoco"
        scene_path = (mujoco_root / "scene" / f"{scene_name}.xml").resolve()
        asset_path = (mujoco_root / "asset" / asset_name / f"{asset_name}.xml").resolve()
        merged_xml = self._merge_scene_robot_xml(scene_path, asset_path)

        self._merged_xml_file = tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False, encoding="utf-8")
        self._merged_xml_file.write(merged_xml)
        self._merged_xml_file.flush()
        self._merged_xml_file.close()
        self._merged_xml_path = Path(self._merged_xml_file.name).resolve()

        self._sim_clock = SimulationClock()
        self._step_count = 0
        self._dt_s = 0.002
        self._scene = None
        self._robot = None
        self._scene_show_viewer = False

    @property
    def _simulation_time_us(self) -> int:
        """Return the current simulated time in microseconds."""

        return self._sim_clock.current_time_us

    @_simulation_time_us.setter
    def _simulation_time_us(self, value: int) -> None:
        """Reset the shared simulation clock to an absolute timestamp."""

        self._sim_clock.reset(value)

    def _advance_simulation_time_us(self, delta_us: int) -> int:
        """Advance the shared simulation clock by a microsecond delta."""

        return self._sim_clock.advance_us(delta_us)

    def _advance_simulation_time_seconds(self, dt_s: float) -> int:
        """Advance the shared simulation clock by seconds."""

        return self._sim_clock.advance_seconds(dt_s)

    def run(self):
        """Build a viewer scene if needed and keep stepping until interrupted."""

        if self._scene is None or not self._scene_show_viewer:
            self._scene = None
            self._robot = None
            self._scene = gs.Scene(
                sim_options=gs.options.SimOptions(dt=self._dt_s),
                show_viewer=True,
            )
            self._robot = self._scene.add_entity(gs.morphs.MJCF(file=str(self._merged_xml_path)))
            self._scene.build()
            self._scene_show_viewer = True
        try:
            while True:
                self.step()
        except KeyboardInterrupt:
            return

    def step(self):
        """Advance the Genesis simulation by one backend step."""

        if self._scene is None:
            self._scene = gs.Scene(
                sim_options=gs.options.SimOptions(dt=self._dt_s),
                show_viewer=False,
            )
            self._robot = self._scene.add_entity(gs.morphs.MJCF(file=str(self._merged_xml_path)))
            self._scene.build()
            self._scene_show_viewer = False
        self._step_count += 1
        self._advance_simulation_time_seconds(self._dt_s)
        self._scene.step()

    def close(self):
        """Release the scene, shared clock, and temporary merged MJCF file."""

        if self._scene is not None:
            self._scene = None
            self._robot = None
        self._sim_clock.close()
        merged_xml_path = getattr(self, "_merged_xml_path", None)
        if isinstance(merged_xml_path, Path) and merged_xml_path.exists():
            merged_xml_path.unlink()

    def _merge_scene_robot_xml(self, scene_path: Path, robot_path: Path) -> str:
        """Merge a scene XML tree with an asset XML tree into one MJCF string."""

        scene_root = ET.parse(scene_path).getroot()
        robot_root = ET.parse(robot_path).getroot()

        def merge_children(tag: str) -> None:
            robot_elem = robot_root.find(tag)
            if robot_elem is None:
                return
            scene_elem = scene_root.find(tag)
            if scene_elem is None:
                scene_root.append(deepcopy(robot_elem))
                return
            for child in list(robot_elem):
                scene_elem.append(deepcopy(child))

        def copy_if_missing(tag: str) -> None:
            if scene_root.find(tag) is not None:
                return
            robot_elem = robot_root.find(tag)
            if robot_elem is not None:
                scene_root.append(deepcopy(robot_elem))

        for tag in ["compiler", "option", "size", "default", "visual", "statistic", "extension"]:
            copy_if_missing(tag)

        compiler = scene_root.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(scene_root, "compiler")
        mesh_dir = (robot_path.parent / "meshes").resolve().as_posix()
        compiler.set("meshdir", mesh_dir)
        compiler.set("texturedir", mesh_dir)

        for tag in ["asset", "worldbody", "actuator", "sensor", "keyframe", "contact", "equality", "tendon"]:
            merge_children(tag)

        return ET.tostring(scene_root, encoding="unicode")
