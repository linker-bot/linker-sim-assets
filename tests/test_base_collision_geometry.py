"""Collision simplification must preserve free space and native primitive poses."""

from pathlib import Path
import json
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from scripts.collision_geometry import Polytope, fit_collider, within_envelope

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"


def box(center, size):
    return trimesh.creation.box(
        extents=size, transform=trimesh.transformations.translation_matrix(center)
    )


class BaseCollisionGeometryTests(unittest.TestCase):
    def test_connected_parts_can_merge_but_empty_middle_must_remain_open(self):
        left = box([-0.015, 0, 0], [0.01, 0.02, 0.02])
        right = box([0.015, 0, 0], [0.01, 0.02, 0.02])
        hull = fit_collider([left, right], "mesh").envelope
        # Every hull vertex is on an old part; vertex-only checks miss this gap.
        old = [Polytope.make(m.vertices) for m in (left, right)]
        self.assertFalse(within_envelope(hull, old, 0.002))
        middle = box([0, 0, 0], [0.02, 0.02, 0.02])
        self.assertTrue(
            within_envelope(hull, old + [Polytope.make(middle.vertices)], 0.002)
        )
        self.assertFalse(within_envelope(hull, old, 0.002, budget=1))

    def test_tolerance_is_measured_against_original_not_previous_fit(self):
        original = Polytope.make(box([0, 0, 0], [0.01] * 3).vertices)
        first = Polytope.make(box([0.00075, 0, 0], [0.0115, 0.01, 0.01]).vertices)
        second = Polytope.make(box([0.0015, 0, 0], [0.013, 0.01, 0.01]).vertices)
        self.assertTrue(within_envelope(first, [original], 0.002))
        self.assertTrue(within_envelope(second, [first], 0.002))
        self.assertFalse(within_envelope(second, [original], 0.002))

    def test_native_urdf_and_mjcf_primitive_dimensions_and_poses_agree(self):
        count = 0
        for directory in sorted(
            (ASSETS / "components/bases").glob("*/variants/default")
        ):
            if not (directory / "collision_groups.yaml").exists():
                continue
            root = ET.parse(directory / "base.urdf").getroot()
            model = mujoco.MjModel.from_xml_path(str(directory / "base.mjcf"))
            for part in json.loads((directory / "collision_manifest.json").read_text())[
                "parts"
            ]:
                if part["shape"] == "mesh":
                    continue
                count += 1
                element = root.find(f".//collision[@name='{part['name']}']")
                assert element is not None
                origin = element.find("origin")
                assert origin is not None
                geom = model.geom(part["name"])
                np.testing.assert_allclose(
                    geom.pos, np.fromstring(origin.attrib["xyz"], sep=" "), atol=1e-12
                )
                rpy = np.fromstring(origin.attrib["rpy"], sep=" ")
                rotation = Rotation.from_quat(np.roll(geom.quat, -1)).as_matrix()
                np.testing.assert_allclose(
                    rotation, Rotation.from_euler("xyz", rpy).as_matrix(), atol=1e-12
                )
                primitive = element.find(f"geometry/{part['shape']}")
                assert primitive is not None
                if part["shape"] == "box":
                    self.assertEqual(geom.type, mujoco.mjtGeom.mjGEOM_BOX)
                    np.testing.assert_allclose(
                        geom.size, np.fromstring(primitive.attrib["size"], sep=" ") / 2
                    )
                else:
                    self.assertEqual(geom.type, mujoco.mjtGeom.mjGEOM_CYLINDER)
                    np.testing.assert_allclose(
                        geom.size[:2],
                        [
                            float(primitive.attrib["radius"]),
                            float(primitive.attrib["length"]) / 2,
                        ],
                    )
        self.assertGreater(count, 0)


if __name__ == "__main__":
    unittest.main()
