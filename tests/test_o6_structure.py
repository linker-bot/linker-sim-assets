"""O6 1014 structure, native URDF/MJCF parity and actual mesh reflection."""

from pathlib import Path
import hashlib
import json
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"
# MjModel is re-exported dynamically by MuJoCo native bindings.
MjModel = getattr(mujoco, "MjModel")
MjData = getattr(mujoco, "MjData")
mj_forward = getattr(mujoco, "mj_forward")
O6 = ASSETS / "components/hands/linkerhand_o6"
REFLECT = np.diag([1.0, -1.0, 1.0])


def load_urdf(side):
    root = ET.parse(O6 / f"linkerhand_o6_{side}.urdf").getroot()
    for mesh in root.iter("mesh"):
        path = O6 / mesh.attrib["filename"]
        # MuJoCo URDF supports OBJ; this is the same normalized visual geometry.
        mesh.set("filename", str(path.with_suffix(".obj")))
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", fusestatic="false", discardvisual="false")
    return MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def pose(model, values):
    data = MjData(model)
    for name, value in values.items():
        data.qpos[model.joint(name).qposadr] = value
    mj_forward(model, data)
    return data


class O6StructureTests(unittest.TestCase):
    def test_sources_outputs_and_collision_budget(self):
        provenance = json.loads((O6 / "provenance_1014.json").read_text())
        for name, digest in provenance["outputs"].items():
            self.assertEqual(
                hashlib.sha256((O6 / name).read_bytes()).hexdigest(), digest, name
            )
        self.assertLess(
            sum(v["collision_faces"] for v in provenance["meshes"].values()), 10000
        )
        for side in ("left", "right"):
            root = ET.parse(O6 / f"linkerhand_o6_{side}.urdf").getroot()
            self.assertAlmostEqual(
                sum(
                    float(e.attrib["value"]) for e in root.findall("link/inertial/mass")
                ),
                0.36998,
            )
            for mesh in root.iter("mesh"):
                self.assertEqual(mesh.get("scale", "1 1 1"), "1 1 1")
            self.assertEqual(len(root.findall("joint")), 11)
            self.assertEqual(len(root.findall("joint/mimic")), 5)

    def test_baked_meshes_mirror_including_triangle_winding(self):
        for right in sorted((O6 / "meshes").glob("*/rh_*")):
            left = right.with_name(right.name.replace("rh_", "lh_", 1))
            a = trimesh.load(left, force="mesh", process=False)
            b = trimesh.load(right, force="mesh", process=False)
            assert isinstance(a, trimesh.Trimesh)
            assert isinstance(b, trimesh.Trimesh)
            # Reflection requires winding reversal; same point cloud alone is
            # insufficient because it could leave all left faces inside-out.
            np.testing.assert_allclose(a.vertices, b.vertices @ REFLECT, atol=2e-8)
            np.testing.assert_allclose(
                a.face_normals, b.face_normals @ REFLECT, atol=2e-5
            )

    def test_urdf_and_mjcf_match_all_links_axes_and_full_inertias(self):
        rng = np.random.default_rng(20261014)
        for side in ("left", "right"):
            model = MjModel.from_xml_path(str(O6 / f"linkerhand_o6_{side}.xml"))
            urdf = load_urdf(side)
            self.assertEqual(model.njnt, urdf.njnt)
            for _ in range(12):
                values = {
                    model.joint(j).name: float(rng.uniform(*model.jnt_range[j]))
                    for j in range(model.njnt)
                }
                a, b = pose(model, values), pose(urdf, values)
                for body in range(1, model.nbody):
                    other = urdf.body(model.body(body).name).id
                    np.testing.assert_allclose(a.xpos[body], b.xpos[other], atol=1e-9)
                    np.testing.assert_allclose(a.xmat[body], b.xmat[other], atol=1e-9)
                    np.testing.assert_allclose(a.xipos[body], b.xipos[other], atol=1e-9)
                    self.assertAlmostEqual(
                        model.body_mass[body], urdf.body_mass[other], places=12
                    )
                    ra, rb = a.ximat[body].reshape(3, 3), b.ximat[other].reshape(3, 3)
                    np.testing.assert_allclose(
                        ra @ np.diag(model.body_inertia[body]) @ ra.T,
                        rb @ np.diag(urdf.body_inertia[other]) @ rb.T,
                        atol=1e-12,
                    )
                for joint in range(model.njnt):
                    other = urdf.joint(model.joint(joint).name).id
                    np.testing.assert_allclose(
                        a.xaxis[joint], b.xaxis[other], atol=1e-9
                    )

    def test_multi_pose_mirror_includes_inertial_tensor_direction(self):
        left, right = [
            MjModel.from_xml_path(str(O6 / f"linkerhand_o6_{side}.xml"))
            for side in ("left", "right")
        ]
        rng = np.random.default_rng(20)
        for fraction in [np.zeros(6), np.ones(6), *rng.random((32, 6))]:
            values = {}
            for j, f in zip(right.actuator_trnid[:, 0], fraction):
                values[right.joint(j).name] = float(
                    right.jnt_range[j] @ np.array([1 - f, f])
                )
            for e, child in enumerate(right.eq_obj1id):
                master = right.joint(right.eq_obj2id[e]).name
                value = np.polynomial.polynomial.polyval(
                    values[master], right.eq_data[e, :5]
                )
                self.assertLessEqual(value, right.jnt_range[child, 1])
                values[right.joint(child).name] = value
            a = pose(left, {n.replace("rh_", "lh_", 1): v for n, v in values.items()})
            b = pose(right, values)
            for body in range(1, right.nbody):
                other = left.body(right.body(body).name.replace("rh_", "lh_", 1)).id
                np.testing.assert_allclose(
                    a.xpos[other], REFLECT @ b.xpos[body], atol=1e-10
                )
                np.testing.assert_allclose(
                    a.xipos[other], REFLECT @ b.xipos[body], atol=1e-10
                )
                ra, rb = a.ximat[other].reshape(3, 3), b.ximat[body].reshape(3, 3)
                np.testing.assert_allclose(
                    ra @ np.diag(left.body_inertia[other]) @ ra.T,
                    REFLECT @ rb @ np.diag(right.body_inertia[body]) @ rb.T @ REFLECT,
                    atol=1e-12,
                )

    def test_shared_flange_seats_both_palms_at_same_height(self):
        for suffix in ("", "_gemini335l"):
            for side in ("l", "r"):
                model = MjModel.from_xml_path(
                    str(ASSETS / f"units/ar5_08_o6{suffix}_{side}/workstation.xml")
                )
                data = pose(model, {})
                palm = model.body(f"hand_{side}h_hand_base_link").id
                seat = model.body("flange_hand_mount").id
                seat_in_palm = data.xmat[palm].reshape(3, 3).T @ (
                    data.xpos[seat] - data.xpos[palm]
                )
                np.testing.assert_allclose(seat_in_palm, [0, 0, 0.002], atol=1e-10)
