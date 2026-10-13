"""Regress all five hand families, including different passive-joint counts."""

import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np
import yaml

from linker_robot_assets.composer.compose import compose, resolve_paths
from linker_robot_assets.composer.mjcf_profiles import COUPLED_HANDS
from linker_robot_assets.composer.schemas import ComponentMeta, SchemaError
from linker_robot_assets.hand_coupling import coupled_mjcf
from linker_robot_assets.validate_hand_coupling import probe

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"


def variants():
    for name in COUPLED_HANDS:
        component = ASSETS / "components/hands" / f"linkerhand_{name}"
        meta = ComponentMeta.load(component / "meta.yaml")
        for side, variant in meta.variants.items():
            yield name, component, meta, side, variant


class HandCouplingTests(unittest.TestCase):
    def test_original_assets_reproduce_lag_for_every_hand_and_side(self):
        for name, component, _, side, variant in variants():
            with self.subTest(hand=name, side=side):
                model = mujoco.MjModel.from_xml_path(str(component / variant.mjcf))
                result = probe(model)
                self.assertTrue(result["completed"])
                self.assertTrue(result["active_joint_motion"])
                self.assertTrue(all(result["eq_active"]))
                self.assertFalse(any(result["warnings"]))
                self.assertGreater(result["final_error_rad"], 0.5)
                self.assertFalse(result["passed"])

    def test_all_generated_hands_follow_open_close_cycles(self):
        for name, component, _, side, _ in variants():
            with self.subTest(hand=name, side=side):
                model = mujoco.MjModel.from_xml_path(
                    str(component / "coupled" / f"{side}.xml")
                )
                result = probe(model, cycle=True)
                self.assertTrue(result["passed"], result)
                self.assertAlmostEqual(result["time"], 4.0, places=8)
                self.assertLess(result["final_error_rad"], 0.001)

    def test_composer_keeps_every_hand_profile_and_motor_semantics(self):
        # L30 has no pre-generated solo unit, but is a supported component:
        # compose a real custom solo recipe for each of the ten variants.
        for name, component, _, side, variant in variants():
            with (
                self.subTest(hand=name, side=side),
                tempfile.TemporaryDirectory() as directory,
            ):
                recipe = {
                    "schema_version": 1,
                    "name": f"{name}_{side}_coupled",
                    "components": {
                        "hand": {
                            "component": f"hands/linkerhand_{name}",
                            "variant": side,
                        }
                    },
                    "freeze_base": "hand",
                    "mujoco": {"coupling_profile": "hand_coupled"},
                }
                path = Path(directory)
                (path / "recipe.yaml").write_text(yaml.safe_dump(recipe))
                paths = resolve_paths(path, ASSETS)
                composed = compose(paths)
                paths.out_mjcf.write_text(composed.mjcf_text)
                model = mujoco.MjModel.from_xml_path(str(paths.out_mjcf))
                original = mujoco.MjModel.from_xml_path(str(component / variant.mjcf))
                np.testing.assert_array_equal(model.actuator_gainprm[:, 0], 1.0)
                np.testing.assert_array_equal(model.actuator_biasprm, 0.0)
                np.testing.assert_array_equal(model.eq_data, original.eq_data)
                self.assertEqual(model.nu, original.nu)
                result = probe(
                    model,
                    cycle=True,
                    motor_gains=(
                        original.actuator_gainprm[:, 0],
                        -original.actuator_biasprm[:, 2],
                    ),
                )
                self.assertTrue(result["passed"], result)

    def test_generated_assets_preserve_physics_and_have_no_drift(self):
        expected_counts = {"l6": 5, "o6": 5, "l20lite": 10, "l25": 5, "l30": 4}
        for name, component, meta, side, variant in variants():
            with self.subTest(hand=name, side=side):
                path = component / "coupled" / f"{side}.xml"
                self.assertEqual(path.read_text(), coupled_mjcf(component, side))
                original = mujoco.MjModel.from_xml_path(str(component / variant.mjcf))
                model = mujoco.MjModel.from_xml_path(str(path))
                self.assertEqual(model.neq, expected_counts[name])
                self.assertEqual(model.nu, len(meta.actuated_joints))
                for field in (
                    "dof_damping",
                    "dof_frictionloss",
                    "dof_armature",
                    "body_mass",
                    "body_inertia",
                    "body_ipos",
                    "body_pos",
                    "body_quat",
                    "jnt_range",
                    "jnt_axis",
                    "eq_data",
                    "actuator_gainprm",
                    "actuator_biasprm",
                    "actuator_ctrlrange",
                    "actuator_trnid",
                    "geom_contype",
                    "geom_conaffinity",
                    "geom_friction",
                    "exclude_signature",
                ):
                    np.testing.assert_array_equal(
                        getattr(model, field), getattr(original, field), err_msg=field
                    )
                self.assertAlmostEqual(model.opt.timestep, 0.0000625)
                if name == "l30":
                    thumb = model.joint(f"{side[0]}h_thumb_dip").id
                    self.assertIn(thumb, model.actuator_trnid[:, 0])
                    self.assertNotIn(thumb, model.eq_obj1id)

    def test_mixed_hand_scene_and_o6_only_compatibility_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            recipe = {
                "name": "mixed",
                "components": {
                    "o6": {"component": "hands/linkerhand_o6", "variant": "left"},
                    "l25": {"component": "hands/linkerhand_l25", "variant": "right"},
                    "lite": {
                        "component": "hands/linkerhand_l20lite",
                        "variant": "left",
                    },
                },
                "freeze_base": "o6",
                "mounts": [
                    {
                        "child": "l25:wrist_mount",
                        "parent": "o6:wrist_mount",
                        "xyz": [1, 0, 0],
                    },
                    {
                        "child": "lite:wrist_mount",
                        "parent": "o6:wrist_mount",
                        "xyz": [-1, 0, 0],
                    },
                ],
            }
            for profile, count in (("hand_coupled", 20), ("o6_coupled", 5)):
                recipe["mujoco"] = {"coupling_profile": profile}
                (path / "recipe.yaml").write_text(yaml.safe_dump(recipe))
                paths = resolve_paths(path, ASSETS)
                paths.out_mjcf.write_text(compose(paths).mjcf_text)
                model = mujoco.MjModel.from_xml_path(str(paths.out_mjcf))
                self.assertEqual(model.neq, 20)
                self.assertEqual(
                    int(np.count_nonzero(model.eq_solref[:, 0] == 0.000125)), count
                )
                self.assertAlmostEqual(model.opt.timestep, 0.0000625)

    def test_profile_requires_supported_hand_and_compatible_timestep(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            for name, step, message in (
                ("ar5_08_r", 0.0000625, "supported hand"),
                ("l25_r", 0.002, "timestep"),
            ):
                recipe = yaml.safe_load(
                    (ASSETS / "units" / name / "recipe.yaml").read_text()
                )
                recipe["mujoco"] = {
                    "coupling_profile": "hand_coupled",
                    "timestep": step,
                }
                (path / "recipe.yaml").write_text(yaml.safe_dump(recipe))
                with self.assertRaisesRegex(SchemaError, message):
                    compose(resolve_paths(path, ASSETS))

    def test_l6_lag_is_not_caused_by_its_existing_self_contact(self):
        component = ASSETS / "components/hands/linkerhand_l6"
        for side in ("left", "right"):
            model = mujoco.MjModel.from_xml_path(
                str(component / f"variants/{side}/hand.mjcf")
            )
            model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
            result = probe(model)
            self.assertTrue(result["completed"])
            self.assertEqual(result["max_contacts"], 0)
            self.assertGreater(result["final_error_rad"], 0.5)


if __name__ == "__main__":
    unittest.main()
