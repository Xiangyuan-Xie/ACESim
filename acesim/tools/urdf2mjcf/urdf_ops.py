import re
from pathlib import Path

import trimesh

MUJOCO_COMPILER_TAG = """
<mujoco>
    <compiler
        angle="radian"
        meshdir="meshes/"
        texturedir="meshes/"
        balanceinertia="true"
        discardvisual="false"
    />
</mujoco>
""".strip()


def parse_q0(q0_str: str) -> dict[str, float]:
    initial_q: dict[str, float] = {}
    if not q0_str:
        return initial_q

    for pair in q0_str.split(","):
        if "=" not in pair:
            continue
        key, value = pair.split("=", 1)
        try:
            initial_q[key.strip()] = float(value)
        except ValueError as exc:
            raise ValueError(f"Invalid q0 value for {key.strip()}: {value}") from exc
    return initial_q


def calculate_min_z(urdf_path: Path, initial_q: dict[str, float]) -> float:
    try:
        import pinocchio as pin
    except ImportError as exc:
        raise ImportError("pinocchio is required to calculate the robot auto-height") from exc

    urdf_dir = urdf_path.parent
    package_dir = urdf_dir.parent
    model, collision_model, _ = pin.buildModelsFromUrdf(
        str(urdf_path),
        package_dirs=str(package_dir),
        root_joint=pin.JointModelFreeFlyer(),
    )
    data = model.createData()
    collision_data = collision_model.createData()

    q = pin.neutral(model)
    for name, value in initial_q.items():
        if not model.existJointName(name):
            continue
        joint_id = model.getJointId(name)
        idx_q = model.joints[joint_id].idx_q
        nq = model.joints[joint_id].nq
        if nq == 1:
            q[idx_q] = value

    pin.forwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, collision_model, collision_data)

    min_z = float("inf")
    found_geom = False
    for i, geom_obj in enumerate(collision_model.geometryObjects):
        placement = collision_data.oMg[i]
        transform = placement.homogeneous

        if geom_obj.meshPath and Path(geom_obj.meshPath).exists():
            mesh = trimesh.load(geom_obj.meshPath, force="mesh")
            mesh.apply_transform(transform)
            min_z = min(min_z, float(mesh.bounds[0][2]))
            found_geom = True
            continue

        min_z = min(min_z, float(placement.translation[2]))
        found_geom = True

    return min_z if found_geom else 0.0


def preprocess_urdf(urdf_path: Path, *, floating: bool, height_offset: float = 0.0) -> Path:
    content = urdf_path.read_text(encoding="utf-8")
    content = re.sub(r'filename="[^"]*meshes/', 'filename="meshes/', content)

    if floating:
        links = set(re.findall(r'<link\s+name="([^"]+)"', content))
        children = set(re.findall(r'<child\s+link="([^"]+)"', content))
        roots = list(links - children)
        if len(roots) == 1:
            floating_insert = f"""
  <link name="world_root_dummy_link"/>
  <joint name="floating_base_joint" type="floating">
    <origin xyz="0 0 {height_offset}" rpy="0 0 0"/>
    <parent link="world_root_dummy_link"/>
    <child link="{roots[0]}"/>
  </joint>
"""
            content = re.sub(r"(<robot[^>]*>)", r"\1" + floating_insert, content)

    if "<mujoco" not in content:
        content = re.sub(r"(<robot[^>]*>)", r"\1" + MUJOCO_COMPILER_TAG, content)

    tmp_path = urdf_path.with_name(f"{urdf_path.stem}_tmp.urdf")
    tmp_path.write_text(content, encoding="utf-8")
    return tmp_path
