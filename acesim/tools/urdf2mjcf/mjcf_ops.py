import math
import re
import textwrap
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import mujoco

from acesim.utils.math import quat_mul, quat_rotate

from .config import ConverterConfig, ConverterPaths
from .px4_multirotor import rebuild_px4_multirotor
from .xml_utils import add_collision_exclusions, indent_xml, inject_xml, sort_attributes

XML_OPTION_TAG = textwrap.dedent("""
    <option
        density="1.225"
        timestep="0.001"
        integrator="implicit"
        viscosity="1.8e-5"
        cone="elliptic"
        impratio="10"
        magnetic="2.73e-5 0 -4.54e-5"
    />
    """).strip()

XML_SENSOR_BLOCK = textwrap.dedent("""
    <sensor>
        <framepos name="framepos" objtype="site" objname="base_link_origin" />
        <framequat name="framequat" objtype="site" objname="base_link_origin" />
        <framelinvel name="framelinvel" objtype="site" objname="base_link_origin" />
        <gyro name="gyro" site="base_link_origin" />
        <accelerometer name="accelerometer" site="base_link_origin" />
        <magnetometer name="magnetometer" site="base_link_origin" />
    </sensor>
    """).strip()

XML_ARM_ACTUATORS = textwrap.dedent("""
    <actuator>
        <position
            name="joint_1" joint="joint_1" kp="748.6" kv="0.547"
            forcerange="-4.905 4.905" ctrlrange="-2.6485 2.6485"
        />
        <position
            name="joint_2" joint="joint_2" kp="524.0" kv="0.727"
            forcerange="-3.43 3.43" ctrlrange="0 3.4907"
        />
        <position
            name="joint_3" joint="joint_3" kp="524.0" kv="0.727"
            forcerange="-3.43 3.43" ctrlrange="-2.6485 2.6485"
        />
        <position
            name="joint_4" joint="joint_4" kp="524.0" kv="0.727"
            forcerange="-3.43 3.43" ctrlrange="-3.1416 3.1416"
        />
        <position
            name="joint_5" joint="joint_5" kp="212.6" kv="0.133"
            forcerange="-1.3916 1.3916" ctrlrange="-1.723 0"
        />
        <position
            name="joint_gripper_left" joint="joint_gripper_left" kp="2000.0" kv="124.0"
            forcerange="-49.06 49.06" ctrlrange="-0.04225 0"
        />
        <position
            name="joint_gripper_right" joint="joint_gripper_right" kp="2000.0" kv="124.0"
            forcerange="-49.06 49.06" ctrlrange="0 0.04225"
        />
    </actuator>
    """).strip()

PX4_TARGETS = {"plane", "standard_vtol", "uuv_bluerov2_heavy"}
FIXEDWING_ACTUATOR_SPECS = {
    "plane": [
        ("rudder_ctrl", "rudder_joint", 40.0),
        ("left_flap_ctrl", "left_flap_joint", 40.0),
        ("right_flap_ctrl", "right_flap_joint", 40.0),
        ("left_elevon_ctrl", "left_elevon_joint", 40.0),
        ("right_elevon_ctrl", "right_elevon_joint", 40.0),
        ("elevator_ctrl", "elevator_joint", 40.0),
    ],
    "standard_vtol": [
        ("left_elevon_ctrl", "left_elevon_joint", 45.0),
        ("right_elevon_ctrl", "right_elevon_joint", 45.0),
        ("elevator_ctrl", "elevator_joint", 45.0),
    ],
}


def euler_to_quat(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def fmt_floats(values: list[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def parse_float_list(value: str | None, default: list[float]) -> list[float]:
    if not value:
        return default.copy()
    try:
        return [float(item) for item in value.split()]
    except Exception:
        return default.copy()


def find_body(parent: ET.Element, name: str) -> ET.Element | None:
    for body in parent.iter("body"):
        if body.get("name") == name:
            return body
    return None


def clear_body(body: ET.Element, *, keep_sites: bool = True) -> None:
    for child in list(body):
        if keep_sites and child.tag == "site":
            continue
        body.remove(child)


def ensure_child(root: ET.Element, tag: str) -> ET.Element:
    child = root.find(tag)
    if child is None:
        child = ET.SubElement(root, tag)
    return child


def append_position_actuator(
    actuator_elem: ET.Element,
    *,
    name: str,
    joint: str,
    joint_ranges: dict[str, str],
    kp: float,
    kv: float = 0.0,
) -> None:
    attrib = {
        "name": name,
        "joint": joint,
        "kp": fmt_floats([kp]),
        "ctrllimited": "true",
        "ctrlrange": joint_ranges.get(joint, "-0.78 0.78"),
    }
    if kv > 0.0:
        attrib["kv"] = fmt_floats([kv])
    actuator_elem.append(ET.Element("position", attrib))


def rebuild_body_maps(
    worldbody: ET.Element,
) -> tuple[dict[str, str | None], dict[str, ET.Element], dict[str, dict[str, list[float]]]]:
    body_parent: dict[str, str | None] = {}
    body_elem: dict[str, ET.Element] = {}
    body_transform: dict[str, dict[str, list[float]]] = {}

    def walk_bodies(elem: ET.Element, parent_name: str | None = None) -> None:
        for body in elem.findall("body"):
            name = body.get("name")
            if not name:
                continue
            body_parent[name] = parent_name
            body_elem[name] = body
            body_transform[name] = {
                "pos": parse_float_list(body.get("pos"), [0.0, 0.0, 0.0]),
                "quat": parse_float_list(body.get("quat"), [1.0, 0.0, 0.0, 0.0]),
            }
            walk_bodies(body, name)

    walk_bodies(worldbody)
    return body_parent, body_elem, body_transform


def set_home_height(root: ET.Element, height: float) -> None:
    worldbody = root.find("worldbody")
    base_body = find_body(worldbody, "base_link") if worldbody is not None else None
    if base_body is not None:
        pos = parse_float_list(base_body.get("pos"), [0.0, 0.0, 0.0])
        pos[2] = height
        base_body.set("pos", fmt_floats(pos))

    keyframe = root.find("keyframe")
    if keyframe is None:
        return
    for key in keyframe.findall("key"):
        if key.get("name") != "home":
            continue
        qpos = parse_float_list(key.get("qpos"), [])
        if len(qpos) >= 3:
            qpos[2] = height
            key.set("qpos", fmt_floats(qpos))
        return


def probe_floor_penetration(xml_path: Path) -> float:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF missing worldbody")
    if not any(geom.get("name") == "floor" for geom in worldbody.findall("geom")):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "size": "0 0 0.1",
                "pos": "0 0 0",
                "rgba": "0.2 0.2 0.2 1",
                "group": "0",
                "contype": "1",
                "conaffinity": "1",
            },
        )

    probe_path = xml_path.with_name(f".{xml_path.stem}_floor_probe.xml")
    indent_xml(root)
    tree.write(probe_path, encoding="utf-8", xml_declaration=True)
    try:
        model = mujoco.MjModel.from_xml_path(str(probe_path))
        data = mujoco.MjData(model)
        if model.nkey > 0:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        else:
            mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

        min_floor_dist = float("inf")
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            if geom1 == "floor" or geom2 == "floor":
                min_floor_dist = min(min_floor_dist, float(contact.dist))
        return min_floor_dist
    finally:
        probe_path.unlink(missing_ok=True)


def calibrate_home_height(root: ET.Element, xml_path: Path, config: ConverterConfig) -> None:
    if config.target not in {"x500", "iris", "typhoon_h480", "plane", "standard_vtol", "uuv_bluerov2_heavy"}:
        return
    keyframe = root.find("keyframe")
    if keyframe is None:
        return

    home = None
    for key in keyframe.findall("key"):
        if key.get("name") == "home":
            home = key
            break
    if home is None:
        return

    qpos = parse_float_list(home.get("qpos"), [])
    if len(qpos) < 3:
        return

    indent_xml(root)
    ET.ElementTree(root).write(xml_path, encoding="utf-8", xml_declaration=True)
    min_floor_dist = probe_floor_penetration(xml_path)
    if min_floor_dist != float("inf") and min_floor_dist < config.safety_margin:
        set_home_height(root, qpos[2] + (config.safety_margin - min_floor_dist))


def postprocess_xml(
    xml_path: Path,
    *,
    config: ConverterConfig,
    paths: ConverterPaths,
    initial_q: dict[str, float],
    height_offset: float = 0.0,
) -> None:
    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    body_parent: dict[str, str | None] = {}
    body_elem: dict[str, ET.Element] = {}
    body_transform: dict[str, dict[str, list[float]]] = {}
    rotor_visual_center_local: dict[str, list[float]] = {}

    if worldbody is not None:
        body_parent, body_elem, body_transform = rebuild_body_maps(worldbody)

    def compose_to_base(child_name: str, base_name: str = "base_link") -> list[float] | None:
        path: list[str] = []
        current: str | None = child_name
        while current is not None and current in body_parent:
            path.append(current)
            current = body_parent[current]
            if current == base_name:
                path.append(current)
                break
        if not path or path[-1] != base_name:
            return None

        acc_quat = [1.0, 0.0, 0.0, 0.0]
        acc_pos = [0.0, 0.0, 0.0]
        for i in range(len(path) - 1, 0, -1):
            node = path[i - 1]
            tf = body_transform.get(node, {"pos": [0.0, 0.0, 0.0], "quat": [1.0, 0.0, 0.0, 0.0]})
            rotated = quat_rotate(acc_quat, tf["pos"])
            acc_pos = [acc_pos[0] + rotated[0], acc_pos[1] + rotated[1], acc_pos[2] + rotated[2]]
            acc_quat = quat_mul(acc_quat, tf["quat"])
        base_quat = body_transform.get(base_name, {"quat": [1.0, 0.0, 0.0, 0.0]})["quat"]
        inv_base = [base_quat[0], -base_quat[1], -base_quat[2], -base_quat[3]]
        return quat_rotate(inv_base, acc_pos)

    def compose_to_world(body_name: str) -> tuple[list[float], list[float]] | None:
        path: list[str] = []
        current: str | None = body_name
        while current is not None and current in body_parent:
            path.append(current)
            current = body_parent[current]
        if not path:
            return None

        acc_quat = [1.0, 0.0, 0.0, 0.0]
        acc_pos = [0.0, 0.0, 0.0]
        for i in range(len(path) - 1, -1, -1):
            node = path[i]
            tf = body_transform.get(node, {"pos": [0.0, 0.0, 0.0], "quat": [1.0, 0.0, 0.0, 0.0]})
            rotated = quat_rotate(acc_quat, tf["pos"])
            acc_pos = [acc_pos[0] + rotated[0], acc_pos[1] + rotated[1], acc_pos[2] + rotated[2]]
            acc_quat = quat_mul(acc_quat, tf["quat"])
        return acc_pos, acc_quat

    def compose_rotation_to_base(child_name: str, base_name: str = "base_link") -> list[float] | None:
        path: list[str] = []
        current: str | None = child_name
        while current is not None and current in body_parent:
            path.append(current)
            current = body_parent[current]
            if current == base_name:
                path.append(current)
                break
        if not path or path[-1] != base_name:
            return None

        acc_quat = [1.0, 0.0, 0.0, 0.0]
        for i in range(len(path) - 1, 0, -1):
            node = path[i - 1]
            tf = body_transform.get(node, {"quat": [1.0, 0.0, 0.0, 0.0]})
            acc_quat = quat_mul(acc_quat, tf["quat"])
        return acc_quat

    def compose_rotated_offset_to_base(body_name: str, offset_local: list[float]) -> list[float] | None:
        offset_base = compose_to_base(body_name, "base_link")
        quat_base = compose_rotation_to_base(body_name, "base_link")
        if offset_base is None or quat_base is None:
            return None
        rotated_local = quat_rotate(quat_base, offset_local)
        return [
            offset_base[0] + rotated_local[0],
            offset_base[1] + rotated_local[1],
            offset_base[2] + rotated_local[2],
        ]

    def compose_visual_center_world(
        body_name: str, offset_local: list[float]
    ) -> tuple[list[float], list[float]] | None:
        world_pose = compose_to_world(body_name)
        if world_pose is None:
            return None
        world_pos, world_quat = world_pose
        rotated_local = quat_rotate(world_quat, offset_local)
        return (
            [
                world_pos[0] + rotated_local[0],
                world_pos[1] + rotated_local[1],
                world_pos[2] + rotated_local[2],
            ],
            world_quat,
        )

    def align_rotor_visual_bodies() -> None:
        if worldbody is None:
            return
        for vis_body in worldbody.findall("body"):
            vis_name = vis_body.get("name", "")
            match = re.fullmatch(r"rotor_(\d+)_vis", vis_name)
            if not match:
                continue
            physical_name = f"rotor_{match.group(1)}"
            center_local = rotor_visual_center_local.get(physical_name, [0.0, 0.0, 0.0])
            rotor_world_pose = compose_visual_center_world(physical_name, center_local)
            if rotor_world_pose is None:
                continue
            vis_body.set("pos", fmt_floats(rotor_world_pose[0]))
            vis_body.set("quat", fmt_floats(rotor_world_pose[1]))

    if worldbody is not None:
        seen = set()
        for geom in list(worldbody.findall("geom")):
            if geom.get("type") != "mesh":
                continue
            mesh = geom.get("mesh")
            if mesh in seen:
                worldbody.remove(geom)
            else:
                seen.add(mesh)

    if root.find("option") is None:
        compiler = root.find("compiler")
        idx = list(root).index(compiler) if compiler is not None else 0
        inject_xml(root, XML_OPTION_TAG, idx + 1, source="MuJoCo option tag")

    if root.find("sensor") is None:
        inject_xml(root, XML_SENSOR_BLOCK, source="PX4 sensor block")
    if root.find("actuator") is None:
        if config.target in PX4_TARGETS:
            root.append(ET.Element("actuator"))
        else:
            inject_xml(root, XML_ARM_ACTUATORS, source="arm actuators")

    if worldbody is not None:
        rebuild_px4_multirotor(root, worldbody, config, paths)
        body_parent, body_elem, body_transform = rebuild_body_maps(worldbody)

    if worldbody is not None:
        base_body = find_body(root, "base_link")
        if base_body is not None and not any(
            site.get("name") == "base_link_origin" for site in base_body.findall("site")
        ):
            base_body.append(
                ET.Element(
                    "site",
                    {"name": "base_link_origin", "type": "sphere", "size": "0.01", "rgba": "1 0 0 0.5", "pos": "0 0 0"},
                )
            )

        rotor_indices = sorted(
            {int(match.group(1)) for name in body_elem if (match := re.fullmatch(r"rotor_(\d+)", name))}
        )
        offsets: dict[int, list[float]] = {}
        rotor_site_pos_local: dict[str, list[float]] = {}
        for body in root.iter("body"):
            body_name = body.get("name")
            if body_name is None or not re.fullmatch(r"rotor_(\d+)", body_name):
                continue
            mesh_geoms = [geom for geom in body.findall("geom") if geom.get("mesh")]
            if not mesh_geoms:
                rotor_site_pos_local[body_name] = [0.0, 0.0, 0.0]
                continue
            accum = [0.0, 0.0, 0.0]
            for geom in mesh_geoms:
                pos = parse_float_list(geom.get("pos"), [0.0, 0.0, 0.0])
                accum = [accum[0] + pos[0], accum[1] + pos[1], accum[2] + pos[2]]
            rotor_site_pos_local[body_name] = [value / len(mesh_geoms) for value in accum]
        for rotor_name in [f"rotor_{idx}" for idx in rotor_indices]:
            if rotor_name in body_elem:
                offset = compose_rotated_offset_to_base(
                    rotor_name, rotor_site_pos_local.get(rotor_name, [0.0, 0.0, 0.0])
                )
                if offset is not None:
                    offsets[int(rotor_name.split("_")[1])] = offset
        if not offsets:
            for name in body_elem:
                if "rotor" not in name or name == "base_link":
                    continue
                offset = compose_rotated_offset_to_base(name, rotor_site_pos_local.get(name, [0.0, 0.0, 0.0]))
                match = re.fullmatch(r"rotor_(\d+)", name)
                if offset is not None and match:
                    offsets[int(match.group(1))] = offset
        if base_body is not None:
            for idx, vec in sorted(offsets.items()):
                site_name = f"rotor_offset_{idx}"
                if any(site.get("name") == site_name for site in base_body.findall("site")):
                    continue
                base_body.append(
                    ET.Element(
                        "site",
                        {
                            "name": site_name,
                            "type": "sphere",
                            "size": "0.005",
                            "rgba": "0 0 1 0.5",
                            "pos": f"{vec[0]:.6f} {vec[1]:.6f} {vec[2]:.6f}",
                        },
                    )
                )

        rotor_sites = {f"rotor_{idx}": f"rotor_joint_thrust{idx}" for idx in rotor_indices}
        for body in root.iter("body"):
            body_name = body.get("name")
            if body_name is None:
                continue
            if body_name not in rotor_sites:
                continue
            site_name = rotor_sites[body_name]
            site_pos = rotor_site_pos_local.get(body_name, [0.0, 0.0, 0.0])
            existing_site = next((site for site in body.findall("site") if site.get("name") == site_name), None)
            site_attrib = {
                "name": site_name,
                "type": "cylinder",
                "size": "0.01 0.005",
                "pos": fmt_floats(site_pos),
                "rgba": "1 0 0 0.5",
            }
            if existing_site is None:
                body.append(ET.Element("site", site_attrib))
            else:
                existing_site.attrib.update(site_attrib)

    if worldbody is not None:
        rotor_visual_geom_map: dict[str, list[ET.Element]] = {}
        for body in root.iter("body"):
            body_name = body.get("name", "")
            match = re.fullmatch(r"rotor_(\d+)", body_name)
            if not match:
                continue
            rotor_id = match.group(1)
            for geom in body.findall("geom"):
                mesh = geom.get("mesh")
                if mesh:
                    rotor_visual_geom_map.setdefault(rotor_id, []).append(deepcopy(geom))
            rotor_visual_center_local[body_name] = [0.0, 0.0, 0.0]
            if rotor_id in rotor_visual_geom_map and rotor_visual_geom_map[rotor_id]:
                accum = [0.0, 0.0, 0.0]
                for geom in rotor_visual_geom_map[rotor_id]:
                    pos = parse_float_list(geom.get("pos"), [0.0, 0.0, 0.0])
                    accum = [accum[0] + pos[0], accum[1] + pos[1], accum[2] + pos[2]]
                rotor_visual_center_local[body_name] = [value / len(rotor_visual_geom_map[rotor_id]) for value in accum]

        for rotor_id in sorted(rotor_visual_geom_map, key=int):
            vis_name = f"rotor_{rotor_id}_vis"
            existing = next((body for body in worldbody.findall("body") if body.get("name") == vis_name), None)
            rotor_name = f"rotor_{rotor_id}"
            center_local = rotor_visual_center_local.get(rotor_name, [0.0, 0.0, 0.0])
            rotor_world_pose = compose_visual_center_world(rotor_name, center_local)
            centered_geoms: list[ET.Element] = []
            for geom in rotor_visual_geom_map[rotor_id]:
                geom_pos = parse_float_list(geom.get("pos"), [0.0, 0.0, 0.0])
                centered_pos = [
                    geom_pos[0] - center_local[0],
                    geom_pos[1] - center_local[1],
                    geom_pos[2] - center_local[2],
                ]
                if all(abs(value) <= 1e-12 for value in centered_pos):
                    geom.attrib.pop("pos", None)
                else:
                    geom.set("pos", fmt_floats(centered_pos))
                centered_geoms.append(geom)
            if existing is None:
                attrs = {"name": vis_name, "mocap": "true"}
                if rotor_world_pose is not None:
                    attrs["pos"] = fmt_floats(rotor_world_pose[0])
                    attrs["quat"] = fmt_floats(rotor_world_pose[1])
                new_body = ET.Element("body", attrs)
                for geom in centered_geoms:
                    geom.set("group", "1")
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
                    new_body.append(geom)
                worldbody.append(new_body)
            else:
                for geom in list(existing.findall("geom")):
                    existing.remove(geom)
                if rotor_world_pose is not None:
                    existing.set("pos", fmt_floats(rotor_world_pose[0]))
                    existing.set("quat", fmt_floats(rotor_world_pose[1]))
                for geom in centered_geoms:
                    geom.set("group", "1")
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
                    existing.append(geom)
                existing.set("mocap", "true")

        for body in root.iter("body"):
            name = body.get("name", "")
            if not (name.startswith("rotor_") and name != "base_link" and re.match(r"rotor_[0-9]+$", name)):
                continue
            for joint in list(body.findall("joint")):
                body.remove(joint)
            for inertial in list(body.findall("inertial")):
                body.remove(inertial)
            if config.target not in {"x500", "iris", "typhoon_h480"}:
                for geom in [geom for geom in list(body.findall("geom")) if geom.get("mesh")]:
                    body.remove(geom)

    qpos_values: list[str] = []
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        joint_type = joint.get("type", "hinge")
        value = str(initial_q.get(joint_name, 0)) if joint_name else "0"

        if joint_type == "free" or joint_name == "floating_base_joint":
            qpos_values.extend(["0", "0", str(height_offset), "1", "0", "0", "0"])
        elif joint_type == "ball":
            qpos_values.extend(["0", "0", "0", "0"])
        else:
            qpos_values.append(value)

    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.Element("keyframe")
        root.append(keyframe)
    for key in list(keyframe.findall("key")):
        if key.get("name") == "home":
            keyframe.remove(key)
    keyframe.append(ET.Element("key", name="home", qpos=" ".join(qpos_values)))

    actuator_elem = root.find("actuator")
    if actuator_elem is not None:
        actual_joint_names = {joint.get("name") for joint in root.iter("joint") if joint.get("name")}
        joint_ranges: dict[str, str] = {
            joint_name: joint.get("range", "-0.78 0.78")
            for joint in root.iter("joint")
            for joint_name in [joint.get("name")]
            if joint_name is not None
        }
        for child in list(actuator_elem):
            joint_name = child.get("joint")
            if joint_name and joint_name not in actual_joint_names:
                actuator_elem.remove(child)

        if config.target in FIXEDWING_ACTUATOR_SPECS:
            actuator_elem.clear()
            for actuator_name, joint_name, kp in FIXEDWING_ACTUATOR_SPECS[config.target]:
                if joint_name not in actual_joint_names:
                    continue
                append_position_actuator(
                    actuator_elem,
                    name=actuator_name,
                    joint=joint_name,
                    joint_ranges=joint_ranges,
                    kp=kp,
                )
        elif config.target == "uuv_bluerov2_heavy":
            actuator_elem.clear()
        else:
            existing_joints = {child.get("joint") for child in actuator_elem if child.get("joint")}
            for joint in root.iter("joint"):
                joint_name = joint.get("name", "")
                joint_type = joint.get("type", "hinge")
                if joint_type == "free" or joint_name in existing_joints or joint_name == "floating_base_joint":
                    continue
                limited = joint.get("limited", "false") == "true"
                ctrl_range = joint.get("range", "0 0") if limited else "-3.14 3.14"
                is_gripper = "gripper" in joint_name
                actuator_elem.append(
                    ET.Element(
                        "position",
                        {
                            "name": joint_name,
                            "joint": joint_name,
                            "kp": "2000" if is_gripper else "500",
                            "kv": "124" if is_gripper else "50",
                            "ctrllimited": "true",
                            "ctrlrange": ctrl_range,
                            "forcelimited": "true",
                            "forcerange": "-100 100",
                        },
                    )
                )
            for child in list(actuator_elem):
                joint_name = child.get("joint", "")
                if joint_name.startswith("rotor_joint_"):
                    actuator_elem.remove(child)

    for geom in root.iter("geom"):
        if geom.get("name") == "floor":
            geom.set("group", "0")
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
            continue

        mesh_name = geom.get("mesh", "")
        geom_type = geom.get("type", "")
        parent_body = next((body for body in root.iter("body") if geom in list(body)), None)
        parent_name = parent_body.get("name", "") if parent_body is not None else ""

        if config.target in PX4_TARGETS:
            is_rotor_visual = parent_name.endswith("_vis") or mesh_name.startswith("rotor_")
            is_collision = geom_type != "mesh"
            if is_rotor_visual:
                is_collision = False
        else:
            is_collision = "_decomp_" in mesh_name
            if geom.get("group") in {"0", "3"}:
                is_collision = True
            elif geom.get("contype") == "1" or geom.get("conaffinity") == "1":
                if geom.get("group") != "1":
                    is_collision = True
            if mesh_name.startswith("rotor_"):
                is_collision = False

        geom.set("group", "3" if is_collision else "1")
        geom.set("contype", "1" if is_collision else "0")
        geom.set("conaffinity", "1" if is_collision else "0")
        if config.target in PX4_TARGETS and is_collision:
            geom.set("rgba", "0 0 0 0")

    add_collision_exclusions(root)
    sort_attributes(root)
    calibrate_home_height(root, xml_path, config)
    if worldbody is not None:
        body_parent, body_elem, body_transform = rebuild_body_maps(worldbody)
        align_rotor_visual_bodies()
    indent_xml(root)
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
