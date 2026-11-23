"""Checks on the robot files. For us, not users; run on every push, on MuJoCo 3.17 and 4.0.1.

- every MJCF compiles
- each URDF describes the same robot as its MJCF: links, joints (type, axis, range, effort where
  the MJCF limits force), couplings, masses, inertias, link poses at the home pose, mesh files
- every referenced file exists, and every file in a `meshes/` folder is referenced
- the mesh rule: `.obj` + `.mtl` + one palette PNG per robot, named after it, nothing else
- each hand's `sdk_channels.yaml` names every actuated joint exactly once
- the controller data (`hands/o6/motor.yaml`, `arms/ar5_08/controller.yaml`) fits the robot files

The URDF is read here by its own rules (URDF spec), not through MuJoCo's URDF importer.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
MJCF = sorted([*ROOT.glob("workstations/*/*.xml"), *ROOT.glob("hands/*/*.xml")])
PAIRS = sorted(p for p in [*ROOT.glob("workstations/*/*.urdf"), *ROOT.glob("hands/*/*.urdf")])
HANDS = sorted(p.parent for p in ROOT.glob("hands/*/sdk_channels.yaml"))
ASSET_FILES = {".xml", ".urdf", ".yaml", ".obj", ".mtl", ".png"}


def rel(p):
    return str(p.relative_to(ROOT))


def compiled(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    if model.nkey and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home") >= 0:
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_kinematics(model, data)
    return model, data


# ---------------------------------------------------------------- URDF, read by the spec


def _vec(text, n=3):
    return np.array([float(x) for x in text.split()]) if text else np.zeros(n)


def _rot(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin(el):
    t = np.eye(4)
    o = el.find("origin") if el is not None else None
    if o is not None:
        t[:3, :3] = _rot(_vec(o.get("rpy")))
        t[:3, 3] = _vec(o.get("xyz"))
    return t


def _axis_rotation(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def read_urdf(path):
    robot = ET.parse(path).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}
    return links, joints


def urdf_poses(links, joints, q):
    """World pose of every link with joint positions q (by name); the root link at the origin."""
    by_child = {j.find("child").get("link"): j for j in joints.values()}
    (root,) = [name for name in links if name not in by_child]
    poses = {root: np.eye(4)}

    def pose(name):
        if name not in poses:
            j = by_child[name]
            t = pose(j.find("parent").get("link")) @ _origin(j)
            if j.get("type") in ("revolute", "continuous"):
                motion = np.eye(4)
                motion[:3, :3] = _axis_rotation(_vec(j.find("axis").get("xyz")), q[j.get("name")])
                t = t @ motion
            elif j.get("type") == "prismatic":
                t = t.copy()
                t[:3, 3] += t[:3, :3] @ (_vec(j.find("axis").get("xyz")) * q[j.get("name")])
            poses[name] = t
        return poses[name]

    for name in links:
        pose(name)
    return root, poses


# ---------------------------------------------------------------- MJCF compiles


@pytest.mark.parametrize("path", MJCF, ids=rel)
def test_mjcf_compiles(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    assert model.nbody > 1


# ---------------------------------------------------------------- URDF = MJCF


def mjcf_of(urdf):
    return urdf.with_suffix(".xml")


def body_name(model, b):
    return model.body(b).name or "world"


@pytest.mark.parametrize("urdf", PAIRS, ids=rel)
def test_urdf_matches_mjcf(urdf):
    model, data = compiled(mjcf_of(urdf))
    links, joints = read_urdf(urdf)
    moving = {n: j for n, j in joints.items() if j.get("type") != "fixed"}

    # The same links (bodies) and the same moving joints.
    bodies = {body_name(model, b) for b in range(model.nbody)}
    assert set(links) == bodies - ({"world"} - set(links))
    assert set(moving) == {model.joint(j).name for j in range(model.njnt)}

    # Joint by joint: type, range, effort where the MJCF limits force.
    for name, j in moving.items():
        mj = model.joint(name)
        hinge, slide = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
        assert {"revolute": hinge, "continuous": hinge, "prismatic": slide}[j.get("type")] == int(model.jnt_type[mj.id]), name
        limit = j.find("limit")
        if j.get("type") == "continuous":
            assert not model.jnt_limited[mj.id], name
        else:
            assert model.jnt_limited[mj.id], name
            np.testing.assert_allclose([float(limit.get("lower")), float(limit.get("upper"))], model.jnt_range[mj.id], atol=1e-9, err_msg=name)
        if model.jnt_actfrclimited[mj.id]:
            assert float(limit.get("effort")) == pytest.approx(model.jnt_actfrcrange[mj.id][1], abs=1e-9), name

    # Couplings: URDF mimic = MJCF joint equality (joint1 = offset + multiplier * joint2).
    mimic = {n: (j.find("mimic").get("joint"), float(j.find("mimic").get("multiplier", 1)), float(j.find("mimic").get("offset", 0)))
             for n, j in moving.items() if j.find("mimic") is not None}
    equality = {}
    for e in range(model.neq):
        if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT and model.eq_active0[e]:
            c = model.eq_data[e][:5]
            assert np.allclose(c[2:], 0), "nonlinear coupling has no URDF form"
            equality[model.joint(model.eq_obj1id[e]).name] = (model.joint(model.eq_obj2id[e]).name, c[1], c[0])
    assert mimic.keys() == equality.keys()
    for n in mimic:
        assert mimic[n][0] == equality[n][0], n
        np.testing.assert_allclose(mimic[n][1:], equality[n][1:], atol=1e-9, err_msg=n)

    # Link poses, axes, masses and inertias at the home pose (qpos0 if the MJCF has no home).
    q = {model.joint(j).name: data.qpos[model.jnt_qposadr[j]] for j in range(model.njnt)}
    root, poses = urdf_poses(links, joints, q)
    rb = model.body(root).id if root != "world" else 0
    base = np.eye(4)
    base[:3, :3], base[:3, 3] = data.xmat[rb].reshape(3, 3), data.xpos[rb]
    for name, t in poses.items():
        b = model.body(name).id if name != "world" else 0
        world = base @ t
        np.testing.assert_allclose(world[:3, 3], data.xpos[b], atol=1e-8, err_msg=f"{name} position")
        np.testing.assert_allclose(world[:3, :3], data.xmat[b].reshape(3, 3), atol=1e-8, err_msg=f"{name} orientation")

        inertial = links[name].find("inertial")
        if inertial is None:
            assert b == 0 or model.body_mass[b] == 0, f"{name}: URDF has no inertial"
            continue
        assert float(inertial.find("mass").get("value")) == pytest.approx(model.body_mass[b], rel=1e-9), name
        frame = world @ _origin(inertial)
        np.testing.assert_allclose(frame[:3, 3], data.xipos[b], atol=1e-8, err_msg=f"{name} centre of mass")
        i = inertial.find("inertia")
        tensor = np.array([[float(i.get(a)) for a in row] for row in
                           (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
        r_urdf, r_mj = frame[:3, :3], data.ximat[b].reshape(3, 3)
        np.testing.assert_allclose(r_urdf @ tensor @ r_urdf.T, r_mj @ np.diag(model.body_inertia[b]) @ r_mj.T,
                                   rtol=1e-6, atol=1e-12, err_msg=f"{name} inertia")

    for name, j in moving.items():
        mj = model.joint(name).id
        child = poses[j.find("child").get("link")]
        axis = (base @ child)[:3, :3] @ _vec(j.find("axis").get("xyz"))
        np.testing.assert_allclose(axis / np.linalg.norm(axis), data.xaxis[mj], atol=1e-8, err_msg=f"{name} axis")

    # The same mesh files.
    urdf_meshes = {(urdf.parent / m.get("filename")).resolve() for m in ET.parse(urdf).iter("mesh")}
    assert urdf_meshes == {f.resolve() for f in mjcf_references(mjcf_of(urdf)) if f.suffix == ".obj"}


# ---------------------------------------------------------------- files


def mjcf_references(path, seen=None):
    """Every file an MJCF loads, following <model>/<include> into other MJCF files."""
    seen = set() if seen is None else seen
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    texturedir = compiler.get("texturedir", "") if compiler is not None else ""
    out = set()
    for el in root.iter():
        f = el.get("file")
        if not f:
            continue
        if el.tag in ("model", "include"):
            child = (path.parent / f).resolve()
            out.add(child)
            if child not in seen:
                seen.add(child)
                out |= mjcf_references(child, seen)
        elif el.tag == "mesh":
            out.add((path.parent / meshdir / f).resolve())
        elif el.tag in ("texture", "hfield", "skin"):
            out.add((path.parent / texturedir / f).resolve())
    return out


def obj_references(path):
    out = set()
    for line in open(path):
        if line.startswith("mtllib "):
            mtl = (path.parent / line.split(None, 1)[1].strip()).resolve()
            out.add(mtl)
            for m in open(mtl):
                if m.startswith("map_Kd "):
                    out.add((mtl.parent / m.split(None, 1)[1].strip()).resolve())
    return out


def all_references():
    refs = set()
    for p in MJCF:
        refs |= mjcf_references(p)
    for p in PAIRS:
        refs |= {(p.parent / m.get("filename")).resolve() for m in ET.parse(p).iter("mesh")}
    for f in list(refs):
        if f.suffix == ".obj" and f.exists():
            refs |= obj_references(f)
    return refs


def test_every_reference_exists_and_every_mesh_is_used():
    refs = all_references()
    missing = sorted(str(f) for f in refs if not f.exists())
    assert not missing, missing
    files = {f.resolve() for f in [*ROOT.glob("workstations/*/meshes/**/*"), *ROOT.glob("hands/*/meshes/**/*")] if f.is_file()}
    unused = sorted(rel(f) for f in files - refs)
    assert not unused, unused


def test_mesh_rule():
    files = [f for f in [*ROOT.glob("workstations/**/*"), *ROOT.glob("hands/**/*")] if f.is_file()]
    others = sorted(rel(f) for f in files if f.suffix not in ASSET_FILES)
    assert not others, f"formats outside the mesh rule: {others}"

    for folder in sorted({f.parent for f in files if f.parent.name == "visual"}):
        # Named after the robot: tools that gather textures into one folder (USD converters)
        # would otherwise overwrite one robot's palette with another's.
        palette = f"{folder.parents[1].name}_palette.png"
        assert sorted(p.name for p in folder.glob("*.png")) == [palette], rel(folder)
        for obj in folder.glob("*.obj"):
            assert obj.name.endswith("_vis.obj"), rel(obj)
            text = obj.read_text()
            assert re.findall(r"^mtllib (.+)$", text, re.M) == [obj.stem + ".mtl"], rel(obj)
            assert len(re.findall(r"^usemtl ", text, re.M)) == 1, f"{rel(obj)}: one material per link"
            assert not re.search(r"^vn ", text, re.M), f"{rel(obj)}: no normals"
            faces = re.findall(r"^f (.+)$", text, re.M)
            assert faces and all(c.count("/") == 1 for f in faces for c in f.split()), f"{rel(obj)}: f v/vt only"
            mtl = (folder / (obj.stem + ".mtl")).read_text()
            assert re.findall(r"^map_Kd (.+)$", mtl, re.M) == [palette], rel(obj)
    for folder in sorted({f.parent for f in files if f.parent.name == "collision"}):
        for obj in folder.glob("*"):
            assert obj.name.endswith("_col.obj"), rel(obj)
            assert "mtllib" not in obj.read_text(), f"{rel(obj)}: collision meshes have no material"


# ---------------------------------------------------------------- SDK channels


@pytest.mark.parametrize("hand", HANDS, ids=rel)
@pytest.mark.parametrize("side", ["left", "right"])
def test_sdk_channels_name_each_actuated_joint_once(hand, side):
    spec = yaml.safe_load((hand / "sdk_channels.yaml").read_text())
    s = side[0]
    model = mujoco.MjModel.from_xml_path(str(hand / f"{hand.name}_{side}.xml"))
    driven = sorted(model.joint(model.actuator_trnid[u, 0]).name for u in range(model.nu))
    _, joints = read_urdf(hand / f"{hand.name}_{side}.urdf")
    urdf_driven = sorted(n for n, j in joints.items() if j.get("type") != "fixed" and j.find("mimic") is None)
    assert driven == urdf_driven
    lists = [k for k in ("channels", "legacy_channels", "slots") if k in spec]
    assert lists, "no channel list"
    for key in lists:
        names = [c.replace("{S}", s) for c in spec[key] if c != "reserved"]
        assert sorted(names) == driven, f"{key}: {sorted(set(names) ^ set(driven))}"


# ---------------------------------------------------------------- controller data


@pytest.mark.parametrize("side", ["left", "right"])
def test_o6_motor_fits_the_hand(side):
    """motor.yaml's shafts are the O6's driven joints; its torque budget fits each drive."""
    spec = yaml.safe_load((ROOT / "hands/o6/motor.yaml").read_text())
    shafts = [j.replace("{S}", side[0]) for j in spec["shafts"]]
    for path, prefix in ((ROOT / f"hands/o6/o6_{side}.xml", ""), (ROOT / "workstations/lw_v0/lw_v0.xml", f"{side}/hand_")):
        model = mujoco.MjModel.from_xml_path(str(path))
        driven = {model.joint(model.actuator_trnid[u, 0]).name for u in range(model.nu)}
        assert {prefix + s for s in shafts} <= driven and len(shafts) == len(set(shafts)) == 6
        for i, shaft in enumerate(shafts):
            joint = model.joint(prefix + shaft).id
            actuator = next(u for u in range(model.nu) if model.actuator_trnid[u, 0] == joint)
            # The motor can never push harder than the drive between shaft and finger.
            assert spec["advancing_cap"][i] <= model.jnt_actfrcrange[joint][1], shaft
            # The drive's damping includes the motor's output damping (any rest is passive).
            assert spec["output_damping"][i] <= -model.actuator_biasprm[actuator][2], shaft
    for key in ("advancing_cap", "output_damping", "viscosity"):
        assert len(spec[key]) == 6 and min(spec[key]) > 0, key
    assert spec["motor_stiffness"] > 0 and spec["speed_limit"] > 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_arm_controller_fits_the_arm(side):
    """controller.yaml names the AR5_08's seven joints; the wrist octagon is convex and its
    bounding box is the J6/J7 ranges in the robot files."""
    spec = yaml.safe_load((ROOT / "arms/ar5_08/controller.yaml").read_text())
    joints = [j.replace("{S}", side[0].upper()) for j in spec["joints"]]
    model = mujoco.MjModel.from_xml_path(str(ROOT / "workstations/lw_v0/lw_v0.xml"))
    for key in ("max_velocity", "max_acceleration", "max_jerk"):
        assert len(spec[key]) == len(joints) == 7 and min(spec[key]) > 0, key
    for j in joints:
        assert model.jnt_type[model.joint(f"{side}/arm_{j}").id] == int(mujoco.mjtJoint.mjJNT_HINGE)
    region = spec["wrist_region"]
    assert [j.replace("{S}", side[0].upper()) for j in region["joints"]] == joints[5:7]
    v = np.radians(np.asarray(region["vertices_deg"], float))
    edges = np.roll(v, -1, axis=0) - v
    nxt = np.roll(edges, -1, axis=0)
    turns = edges[:, 0] * nxt[:, 1] - edges[:, 1] * nxt[:, 0]  # 2-D cross product
    assert (turns < 0).all() or (turns > 0).all(), "the wrist region must be convex"
    for k, j in enumerate(joints[5:7]):
        lo, hi = model.jnt_range[model.joint(f"{side}/arm_{j}").id]
        np.testing.assert_allclose((v[:, k].min(), v[:, k].max()), (lo, hi), atol=1e-9, err_msg=j)


# ---------------------------------------------------------------- the data package (when installed)


def package_files():
    """What `linker-lw-v0` must hold: the files LW-v0's MJCF and URDF load, and the O6's SDK map."""
    ws = ROOT / "workstations/lw_v0"
    files = {*ws.glob("*.xml"), *ws.glob("*.urdf"), ROOT / "hands/o6/sdk_channels.yaml",
             ROOT / "hands/o6/motor.yaml", ROOT / "arms/ar5_08/controller.yaml"}
    for p in ws.glob("*.xml"):
        files |= mjcf_references(p)
    for p in ws.glob("*.urdf"):
        files |= {(p.parent / m.get("filename")).resolve() for m in ET.parse(p).iter("mesh")}
    for f in list(files):
        if f.suffix == ".obj":
            files |= obj_references(f)
    return {Path(f).resolve().relative_to(ROOT) for f in files}


def test_installed_package():
    """The installed package holds exactly LW-v0's files, byte for byte, and MuJoCo loads it."""
    import importlib.resources
    import importlib.util

    if importlib.util.find_spec("linker_lw_v0") is None:
        pytest.skip("linker-lw-v0 is not installed")
    installed = Path(str(importlib.resources.files("linker_lw_v0") / "workstations")).parent
    assert installed.resolve() != ROOT
    packaged = {p.relative_to(installed) for p in installed.rglob("*") if p.is_file() and "__pycache__" not in p.parts}
    assert packaged == package_files(), sorted(packaged ^ package_files())
    for p in packaged:
        assert (installed / p).read_bytes() == (ROOT / p).read_bytes(), p
    model = mujoco.MjModel.from_xml_path(str(installed / "workstations/lw_v0/lw_v0.xml"))
    assert model.nkey and model.key("home")
