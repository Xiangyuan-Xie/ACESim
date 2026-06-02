"""Base MuJoCo environment with merged-scene loading and shared sim clock."""

import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import mujoco
import mujoco.viewer

from acesim.config.config_loader import ConfigLoader
from acesim.env.base_env import BaseEnv
from acesim.utils.math import quat_mul, quat_rotate
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

        mujoco.viewer.launch(self._mj_model, self._mj_data)

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

        self._inject_static_landing_pad(scene_root, robot_root)
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

    def _inject_static_landing_pad(self, scene_root: ET.Element, robot_root: ET.Element) -> None:
        """Add a shared static landing pad to MuJoCo for UE-aligned contact."""

        if self._config_loader.get_asset_name() != "x500_arm2x":
            return
        worldbody = scene_root.find("worldbody")
        if worldbody is None:
            worldbody = ET.SubElement(scene_root, "worldbody")
        if worldbody.find("geom[@name='acesim_landing_pad']") is None:
            ET.SubElement(
                worldbody,
                "geom",
                name="acesim_landing_pad",
                type="cylinder",
                size="3.5 0.02",
                pos="0 0 0.02",
                rgba="0.52 0.53 0.50 1",
                group="0",
                contype="1",
                conaffinity="1",
                friction="1.0 0.005 0.0001",
            )

        keyframe = scene_root.find("keyframe")
        if keyframe is None:
            keyframe = ET.SubElement(scene_root, "keyframe")
        if keyframe.find("key[@name='scene_home']") is not None:
            return
        robot_home = robot_root.find("./keyframe/key[@name='home']")
        if robot_home is None:
            return
        qpos_values = robot_home.get("qpos", "").split()
        if len(qpos_values) < 3:
            return
        try:
            qpos_values[2] = f"{float(qpos_values[2]) + 0.048:.16g}"
        except ValueError:
            return
        scene_home_attrs = {"name": "scene_home", "qpos": " ".join(qpos_values)}
        mocap_pose = self._scene_home_mocap_pose(scene_root, qpos_values)
        if mocap_pose is not None:
            mpos, mquat = mocap_pose
            scene_home_attrs["mpos"] = " ".join(f"{value:.16g}" for value in mpos)
            scene_home_attrs["mquat"] = " ".join(f"{value:.16g}" for value in mquat)
        ET.SubElement(keyframe, "key", scene_home_attrs)

    @staticmethod
    def _parse_float_list(value: str | None, default: list[float]) -> list[float]:
        if not value:
            return default.copy()
        try:
            return [float(item) for item in value.split()]
        except ValueError:
            return default.copy()

    @classmethod
    def _scene_home_mocap_pose(
        cls,
        scene_root: ET.Element,
        qpos_values: list[str],
    ) -> tuple[list[float], list[float]] | None:
        if len(qpos_values) < 7:
            return None

        worldbody = scene_root.find("worldbody")
        if worldbody is None:
            return None
        base_body = next((body for body in worldbody.iter("body") if body.get("name") == "base_link"), None)
        if base_body is None:
            return None

        mocap_bodies = [body for body in worldbody.iter("body") if body.get("mocap") == "true"]
        if not mocap_bodies:
            return None

        try:
            base_pos = [float(value) for value in qpos_values[:3]]
            base_quat = [float(value) for value in qpos_values[3:7]]
        except ValueError:
            return None

        body_pose_to_base: dict[str, tuple[list[float], list[float]]] = {}

        def walk(body: ET.Element, parent_pos: list[float], parent_quat: list[float]) -> None:
            for child in body.findall("body"):
                child_pos = cls._parse_float_list(child.get("pos"), [0.0, 0.0, 0.0])
                child_quat = cls._parse_float_list(child.get("quat"), [1.0, 0.0, 0.0, 0.0])
                rotated = quat_rotate(parent_quat, child_pos)
                composed_pos = [
                    parent_pos[0] + rotated[0],
                    parent_pos[1] + rotated[1],
                    parent_pos[2] + rotated[2],
                ]
                composed_quat = quat_mul(parent_quat, child_quat)
                name = child.get("name")
                if name:
                    body_pose_to_base[name] = (composed_pos, composed_quat)
                walk(child, composed_pos, composed_quat)

        walk(base_body, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])

        mpos: list[float] = []
        mquat: list[float] = []
        for body in mocap_bodies:
            name = body.get("name", "")
            physical_pose = None
            if name.startswith("rotor_") and name.endswith("_vis"):
                physical_pose = body_pose_to_base.get(name.removesuffix("_vis"))
            local_pos, local_quat = physical_pose or (
                cls._parse_float_list(body.get("pos"), [0.0, 0.0, 0.0]),
                cls._parse_float_list(body.get("quat"), [1.0, 0.0, 0.0, 0.0]),
            )
            rotated_pos = quat_rotate(base_quat, local_pos) if physical_pose is not None else local_pos
            world_pos = (
                [base_pos[0] + rotated_pos[0], base_pos[1] + rotated_pos[1], base_pos[2] + rotated_pos[2]]
                if physical_pose is not None
                else local_pos
            )
            world_quat = quat_mul(base_quat, local_quat) if physical_pose is not None else local_quat
            mpos.extend(world_pos)
            mquat.extend(world_quat)

        return mpos, mquat

    def _control(self, model: mujoco.MjModel, data: mujoco.MjData):
        """MuJoCo control callback overridden by subclasses."""
