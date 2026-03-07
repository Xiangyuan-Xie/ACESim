import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import mujoco
import mujoco.viewer

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv


class MujocoEnv(BaseEnv):
    def __init__(self, config_loader: ConfigLoader):
        super().__init__(config_loader)
        scene_name = self._config_loader.get_scene_name()
        asset_name = self._config_loader.get_asset_name()
        scene_path = (Path(__file__).parent / "scene" / f"{scene_name}.xml").resolve()
        asset_path = (Path(__file__).parent / "asset" / asset_name / f"{asset_name}.xml").resolve()
        merged_xml = self._merge_scene_robot_xml(scene_path, asset_path)
        self._mj_model = mujoco.MjModel.from_xml_string(merged_xml)
        self._mj_data = mujoco.MjData(self._mj_model)
        self._mj_model.opt.timestep = 0.001
        if self._mj_model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self._mj_model, self._mj_data, 0)
        else:
            mujoco.mj_resetData(self._mj_model, self._mj_data)
        mujoco.set_mjcb_control(self._control)

        self._simulation_time_us = 0
        self._step_count = 0

    def run(self):
        mujoco.viewer.launch(self._mj_model, self._mj_data)

    def step(self):
        mujoco.mj_step(self._mj_model, self._mj_data)

    def close(self):
        pass

    def _merge_scene_robot_xml(self, scene_path: Path, robot_path: Path) -> str:
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

    def _control(self, model: mujoco.MjModel, data: mujoco.MjData):
        pass
