import math
import re
import textwrap
import xml.etree.ElementTree as ET
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

XML_ACTUATORS_SENSORS = textwrap.dedent("""
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
    <sensor>
        <framepos name="framepos" objtype="site" objname="base_link_origin" />
        <framequat name="framequat" objtype="site" objname="base_link_origin" />
        <framelinvel name="framelinvel" objtype="site" objname="base_link_origin" />
        <gyro name="gyro" site="base_link_origin" />
        <accelerometer name="accelerometer" site="base_link_origin" />
        <magnetometer name="magnetometer" site="base_link_origin" />
    </sensor>
    """).strip()


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
    if config.target not in {"x500", "iris", "typhoon_h480"}:
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

    if root.find("actuator") is None:
        inject_xml(root, XML_ACTUATORS_SENSORS, source="actuators and sensors")

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
        offsets: dict[str, list[float]] = {}
        for rotor_name in [f"rotor_{idx}" for idx in rotor_indices]:
            if rotor_name in body_elem:
                offset = compose_to_base(rotor_name, "base_link")
                if offset is not None:
                    offsets[rotor_name] = offset
        if not offsets:
            for name in body_elem:
                if "rotor" not in name or name == "base_link":
                    continue
                offset = compose_to_base(name, "base_link")
                if offset is not None:
                    offsets[name] = offset
        if base_body is not None:
            for idx, vec in enumerate(offsets.values(), start=1):
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
            if any(site.get("name") == site_name for site in body.findall("site")):
                continue
            body.append(
                ET.Element(
                    "site",
                    {"name": site_name, "type": "cylinder", "size": "0.01 0.005", "pos": "0 0 0", "rgba": "1 0 0 0.5"},
                )
            )

    if worldbody is not None:
        rotor_mesh_map: dict[str, str] = {}
        for geom in root.iter("geom"):
            mesh = geom.get("mesh")
            if mesh and mesh.startswith("rotor_"):
                match = re.match(r"(rotor_([0-9]+))", mesh)
                if match:
                    rotor_mesh_map[match.group(2)] = mesh

        asset = root.find("asset")
        asset_rotor_visual: dict[str, str] = {}
        if asset is not None:
            for mesh_elem in asset.findall("mesh"):
                name = mesh_elem.get("name", "")
                match = re.match(r"rotor_([0-9]+)$", name)
                if match:
                    asset_rotor_visual[match.group(1)] = name

        for rotor_id in sorted(rotor_mesh_map, key=int):
            mesh_name = asset_rotor_visual.get(rotor_id) or rotor_mesh_map[rotor_id]
            vis_name = f"rotor_{rotor_id}_vis"
            existing = next((body for body in worldbody.findall("body") if body.get("name") == vis_name), None)
            if existing is None:
                new_body = ET.Element("body", {"name": vis_name, "mocap": "true"})
                new_geom = ET.Element(
                    "geom",
                    {
                        "type": "mesh",
                        "mesh": mesh_name,
                        "rgba": "1 1 1 1",
                        "group": "1",
                        "contype": "0",
                        "conaffinity": "0",
                    },
                )
                new_body.append(new_geom)
                worldbody.append(new_body)
            else:
                for geom in existing.findall("geom"):
                    geom.set("type", "mesh")
                    geom.set("mesh", mesh_name)
                    geom.set("group", "1")
                    geom.set("contype", "0")
                    geom.set("conaffinity", "0")
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
                for geom in list(body.findall("geom")):
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
        for child in list(actuator_elem):
            joint_name = child.get("joint")
            if joint_name and joint_name not in actual_joint_names:
                actuator_elem.remove(child)

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

    add_collision_exclusions(root)
    sort_attributes(root)
    calibrate_home_height(root, xml_path, config)
    indent_xml(root)
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)
