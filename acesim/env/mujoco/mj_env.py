"""Base MuJoCo environment with merged-scene loading and shared sim clock."""

import json
import os
import time
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import mujoco
import mujoco.viewer

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv
from acesim.utils.sim_streams import ClockPublisher
from acesim.utils.simulation_clock import SimulationClock


class MJEnv(BaseEnv):
    """Base class for MuJoCo environments used in ACESim.

    The class merges a scene XML with an asset XML, owns the MuJoCo model/data
    pair, and exposes a shared simulation clock so PX4-facing code can consume a
    backend-independent time source.
    """

    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        mujoco.set_mjcb_control(None)
        scene_name = self._config_loader.get_scene_name()
        asset_name = self._config_loader.get_asset_name()
        scene_path = (Path(__file__).parent / "scene" / f"{scene_name}.xml").resolve()
        asset_path = (Path(__file__).parent / "asset" / asset_name / f"{asset_name}.xml").resolve()
        merged_xml = self._merge_scene_robot_xml(scene_path, asset_path)
        self._mj_model = mujoco.MjModel.from_xml_string(merged_xml)
        self._mj_data = mujoco.MjData(self._mj_model)
        self._mj_model.opt.timestep = 0.001
        initial_keyframe_id = self._initial_keyframe_id_for_model(self._mj_model)
        if initial_keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, initial_keyframe_id)
        else:
            mujoco.mj_resetData(self._mj_model, self._mj_data)
        mujoco.mj_forward(self._mj_model, self._mj_data)
        mujoco.set_mjcb_control(self._control)

        self._sim_clock = SimulationClock()
        self._clock_publisher = ClockPublisher()
        self._step_count = 0
        self._gui_profile_enabled = os.environ.get("ACESIM_GUI_PROFILE", "0") == "1"
        self._gui_profile_skip_s = 15.0
        self._gui_profile_report_period_s = 5.0
        self._gui_profile_wall_start_s: float | None = None
        self._gui_profile_stable_wall_start_s: float | None = None
        self._gui_profile_stable_sim_start_s: float | None = None
        self._gui_profile_next_report_s = 0.0
        self._publish_clock()

    @property
    def _simulation_time_us(self) -> int:
        """Return the current simulated time in microseconds."""

        return self._sim_clock.current_time_us

    @_simulation_time_us.setter
    def _simulation_time_us(self, value: int) -> None:
        """Reset the shared simulation clock to an absolute timestamp."""

        self._sim_clock.reset(value)
        self._publish_clock()

    def _advance_simulation_time_us(self, delta_us: int) -> int:
        """Advance the shared simulation clock by a microsecond delta."""

        timestamp_us = self._sim_clock.advance_us(delta_us)
        self._publish_clock()
        return timestamp_us

    def _advance_simulation_time_seconds(self, dt_s: float) -> int:
        """Advance the shared simulation clock by seconds."""

        timestamp_us = self._sim_clock.advance_seconds(dt_s)
        self._publish_clock()
        return timestamp_us

    def run(self):
        """Launch MuJoCo's interactive viewer."""

        self._before_interactive_viewer()
        try:
            mujoco.viewer.launch(self._mj_model, self._mj_data)
        finally:
            self._after_interactive_viewer()

    def step(self):
        """Advance the MuJoCo simulation by one backend step."""

        mujoco.mj_step(self._mj_model, self._mj_data)

    def close(self):
        """Release the shared simulation clock owned by the base backend."""

        mujoco.set_mjcb_control(None)
        self._clock_publisher.close()
        self._sim_clock.close()

    def _publish_clock(self) -> None:
        self._clock_publisher.publish(self._simulation_time_us)

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

    @staticmethod
    def _initial_keyframe_id_for_model(model: mujoco.MjModel) -> int:
        """Return the preferred initial keyframe, favoring scene-specific home poses."""

        scene_home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "scene_home")
        if scene_home_id >= 0:
            return scene_home_id
        if model.nkey > 0:
            return 0
        return -1

    def _control(self, model: mujoco.MjModel, data: mujoco.MjData):
        """MuJoCo control callback overridden by subclasses."""

    def _before_interactive_viewer(self) -> None:
        """Hook for environments that need viewer-specific callback behavior."""

    def _after_interactive_viewer(self) -> None:
        """Hook for environments that need to restore non-viewer behavior."""

    def _record_interactive_viewer_profile(self, sim_time_s: float) -> None:
        if not self._gui_profile_enabled:
            return

        now_s = time.monotonic()
        if self._gui_profile_wall_start_s is None:
            self._gui_profile_wall_start_s = now_s
            self._gui_profile_next_report_s = now_s + self._gui_profile_skip_s + self._gui_profile_report_period_s
            return

        elapsed_s = now_s - self._gui_profile_wall_start_s
        if elapsed_s < self._gui_profile_skip_s:
            return

        if self._gui_profile_stable_wall_start_s is None:
            self._gui_profile_stable_wall_start_s = now_s
            self._gui_profile_stable_sim_start_s = sim_time_s
            return

        if now_s < self._gui_profile_next_report_s:
            return

        stable_sim_start_s = self._gui_profile_stable_sim_start_s
        if stable_sim_start_s is None:
            return
        stable_wall_s = now_s - self._gui_profile_stable_wall_start_s
        stable_sim_s = sim_time_s - stable_sim_start_s
        realtime_factor = stable_sim_s / stable_wall_s if stable_wall_s > 0.0 else 0.0
        print(
            "ACESIM_GUI_PROFILE "
            + json.dumps(
                {
                    "elapsed_wall_s": elapsed_s,
                    "stable_wall_s": stable_wall_s,
                    "stable_sim_s": stable_sim_s,
                    "realtime_factor": realtime_factor,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        self._gui_profile_next_report_s = now_s + self._gui_profile_report_period_s
