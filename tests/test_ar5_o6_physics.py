"""Compile the assembly configuration, including scope and import boundaries."""

import unittest
import xml.etree.ElementTree as ET
from math import pi
from pathlib import Path

import mujoco
import numpy as np

from linker_robot_assets.composer.ar5_o6_physics import apply_ar5_o6_physics
from linker_robot_assets.composer.schemas import ComponentMeta, SchemaError
from linker_robot_assets.composer.urdf_ops import compile_component

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"
CONFIGURED = (
    "units/ar5_08_o6_l",
    "units/ar5_08_o6_r",
    "units/ar5_08_o6_gemini335l_l",
    "units/ar5_08_o6_gemini335l_r",
    "workstations/ar5_08_o6_bench_bimanual",
    "workstations/ar5_o6_bench_bimanual",
)


def load(name):
    return mujoco.MjModel.from_xml_path(str(ASSETS / name / "workstation.xml"))


class AR5O6PhysicsTests(unittest.TestCase):
    def test_compiled_physics_and_torque_interface(self):
        for name in CONFIGURED:
            with self.subTest(assembly=name):
                m = load(name)
                self.assertAlmostEqual(m.opt.timestep, 1 / 480)
                self.assertEqual(m.opt.impratio, 100)
                self.assertEqual(m.opt.cone, mujoco.mjtCone.mjCONE_ELLIPTIC)
                self.assertEqual((m.opt.iterations, m.opt.ls_iterations), (100, 50))
                np.testing.assert_array_equal(m.opt.gravity, [0, 0, -9.81])
                active = set(m.actuator_trnid[:, 0])
                for jid in range(m.njnt):
                    joint = m.joint(jid).name
                    arm = "_joint_" in joint
                    yaw = joint.endswith("_thumb_cmc_yaw")
                    dof = m.jnt_dofadr[jid]
                    self.assertAlmostEqual(m.dof_armature[dof], 0.01 if arm else 0.001)
                    self.assertEqual(m.dof_frictionloss[dof], 0)
                    self.assertEqual(
                        m.dof_damping[dof],
                        0 if arm or yaw else 4 if jid in active else 0.4,
                    )
                # No implicit position servo and no independent follower motor.
                np.testing.assert_array_equal(m.actuator_gainprm[:, 0], 1)
                np.testing.assert_array_equal(m.actuator_biasprm, 0)
                self.assertFalse(np.isin(m.eq_obj1id, m.actuator_trnid[:, 0]).any())
                for aid, jid in enumerate(m.actuator_trnid[:, 0]):
                    joint = m.joint(jid).name
                    cap = (
                        200
                        if "_joint_" in joint
                        else 10
                        if joint.endswith("_yaw")
                        else 60
                    )
                    np.testing.assert_array_equal(
                        m.actuator_ctrlrange[aid], [-cap, cap]
                    )
                    np.testing.assert_array_equal(
                        m.actuator_forcerange[aid], [-cap, cap]
                    )
                for row in m.eq_solref:
                    np.testing.assert_allclose(row, [0.002, 1])
                for row in m.eq_solimp:
                    np.testing.assert_allclose(row, [0.999, 0.999, 0.001, 0.5, 2])
                for bid in range(1, m.nbody):
                    body = m.body(bid).name
                    attached = body.startswith(
                        ("arm_", "hand_", "flange_", "bracket_", "camera_")
                    )
                    self.assertEqual(m.body_gravcomp[bid], int(attached), body)

    def test_contact_materials_are_limited_to_colliders(self):
        for name in CONFIGURED:
            with self.subTest(assembly=name):
                m = load(name)
                for gid in range(m.ngeom):
                    if not (m.geom_contype[gid] or m.geom_conaffinity[gid]):
                        continue
                    hand = m.body(m.geom_bodyid[gid]).name.startswith("hand_")
                    np.testing.assert_allclose(
                        m.geom_friction[gid], [4 if hand else 1, 0, 0]
                    )
                    np.testing.assert_allclose(m.geom_solref[gid], [0.004, 1])
                    np.testing.assert_allclose(
                        m.geom_solimp[gid], [0.95 if hand else 0.9, 0.95, 0.001, 0.5, 2]
                    )
                    self.assertEqual(m.geom_condim[gid], 4)
                    self.assertEqual(m.geom_gap[gid], 0)
                    self.assertEqual(m.geom_margin[gid], 0)

    def test_ar5_limits_in_every_generated_model_and_component(self):
        for path in ASSETS.rglob("*"):
            if path.suffix not in {".xml", ".urdf", ".mjcf"}:
                continue
            root = ET.parse(path).getroot()
            joints = (
                root.findall("joint")
                if path.suffix == ".urdf"
                else root.findall(".//worldbody//joint")
            )
            for joint in joints:
                name = joint.get("name", "")
                if "AR5_5_" not in name or not name.endswith(("_joint_6", "_joint_7")):
                    continue
                with self.subTest(path=path, joint=name):
                    if path.suffix == ".urdf":
                        limit = joint.find("limit")
                        assert limit is not None
                        bounds = [float(limit.attrib[k]) for k in ("lower", "upper")]
                    else:
                        bounds = list(map(float, joint.attrib["range"].split()))
                    np.testing.assert_allclose(
                        bounds, [-pi / 3, pi / 3], rtol=0, atol=1e-14
                    )

    def test_unselected_combinations_keep_previous_physics(self):
        for name in (
            "units/o6_r",
            "workstations/a7_o6_dc",
            "units/ar5_08_l6_r",
            "units/ar5_08_r",
        ):
            with self.subTest(assembly=name):
                m = load(name)
                self.assertEqual(m.opt.impratio, 1)
                np.testing.assert_array_equal(m.body_gravcomp, 0)
                if "ar5_08" in name:
                    self.assertAlmostEqual(m.dof_damping[0], 14.22378)
                if name == "units/o6_r":
                    np.testing.assert_allclose(m.dof_armature, 3.4707e-8)
                    np.testing.assert_allclose(m.dof_damping, 4)

    def test_profile_rejects_an_unadapted_hand(self):
        parts = []
        for role, component in (
            ("arm", "arms/ar5_08"),
            ("hand", "hands/linkerhand_l6"),
        ):
            meta = ComponentMeta.load(ASSETS / "components" / component / "meta.yaml")
            parts.append(
                compile_component(
                    role=role,
                    meta=meta,
                    variant=meta.variants["right"],
                    workstation_dir=ASSETS,
                )
            )
        with self.assertRaisesRegex(SchemaError, "only AR5"):
            apply_ar5_o6_physics(ET.Element("mujoco"), parts)


if __name__ == "__main__":
    unittest.main()
