"""O6 coupling must survive composition and real actuator-driven dynamics."""

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

from linker_robot_assets.composer.compose import compose, resolve_paths
from linker_robot_assets.composer.mjcf_profiles import apply_mujoco_options
from linker_robot_assets.composer.schemas import MujocoOptions, SchemaError
from linker_robot_assets.o6_coupling import coupled_mjcf
from linker_robot_assets.validate_hand_coupling import probe
from linker_robot_assets.validate_o6_contact import build_fixture, probe_contact

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"
O6 = ASSETS / "components/hands/linkerhand_o6"


def load(path):
    return mujoco.MjModel.from_xml_path(str(path))


class O6CouplingTests(unittest.TestCase):
    def test_original_right_hand_reproduces_customer_report(self):
        model = load(O6 / "linkerhand_o6_right.xml")
        data = mujoco.MjData(model)
        data.ctrl[:] = [0, 0.3, 0.8, 0.8, 0.8, 0.8]
        for _ in range(2500):
            mujoco.mj_step(model, data)
            self.assertFalse(data.warning.number.any(), "Warning or automatic reset")
        self.assertAlmostEqual(
            data.time, 5.0, places=9, msg="Simulation must not reset"
        )
        np.testing.assert_array_equal(data.eq_active, [1] * 5)
        self.assertEqual(data.ncon, 0)
        active = data.joint("rh_index_mcp_pitch").qpos[0]
        passive = data.joint("rh_index_dip").qpos[0]
        self.assertAlmostEqual(active, 0.8, places=3)
        self.assertGreater(abs(passive - 0.89 * active), 0.6)

    def test_generated_files_have_no_drift_and_keep_physical_parameters(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                path = O6 / "coupled" / f"{side}.xml"
                self.assertEqual(path.read_text(), coupled_mjcf(O6, side))
                original = load(O6 / f"linkerhand_o6_{side}.xml")
                corrected = load(path)
                self.assertEqual(corrected.nu, 6)
                self.assertEqual(corrected.neq, 5)
                self.assertAlmostEqual(corrected.opt.timestep, 0.002)
                np.testing.assert_array_equal(corrected.actuator_gainprm[:, 0], 600)
                np.testing.assert_array_equal(corrected.actuator_biasprm[:, 2], -4.5)
                for field in (
                    "dof_damping",
                    "dof_frictionloss",
                    "dof_armature",
                    "body_mass",
                    "body_inertia",
                    "body_ipos",
                    "actuator_trnid",
                    "jnt_range",
                    "eq_data",
                    "geom_contype",
                    "geom_conaffinity",
                ):
                    np.testing.assert_array_equal(
                        getattr(original, field),
                        getattr(corrected, field),
                        err_msg=field,
                    )

    def test_both_component_hands_follow_open_close_cycle(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                self._exercise(load(O6 / "coupled" / f"{side}.xml"), cycle=True)

    def test_hold_reaches_target_and_settles(self):
        self._exercise(load(O6 / "coupled/right.xml"), cycle=False)

    def test_composed_solo_hands_keep_native_servo_and_follow_cycle(self):
        for side in ("l", "r"):
            with self.subTest(side=side), tempfile.TemporaryDirectory() as directory:
                paths = self._recipe(
                    Path(directory), f"o6_{side}", {"coupling_profile": "o6_coupled"}
                )
                result = compose(paths)
                paths.out_mjcf.write_text(result.mjcf_text)
                model = load(paths.out_mjcf)
                self.assertAlmostEqual(model.opt.timestep, 0.002)
                np.testing.assert_array_equal(model.actuator_gainprm[:, 0], 600)
                np.testing.assert_array_equal(model.actuator_biasprm[:, 2], -4.5)
                self._exercise(model, cycle=True)

    def test_recipe_profile_applies_to_both_hands_in_full_workstation(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self._recipe(
                Path(directory),
                "ar5_08_o6_bench_bimanual",
                {"coupling_profile": "o6_coupled", "timestep": 0.001},
                subdir="workstations",
            )
            result = compose(paths)
            paths.out_mjcf.write_text(result.mjcf_text)
            model = load(paths.out_mjcf)
            self.assertAlmostEqual(model.opt.timestep, 0.001)
            self.assertEqual(model.neq, 10)
            np.testing.assert_allclose(model.eq_solref[:, 0], -1e7)
            np.testing.assert_allclose(model.eq_solimp[:, :2], 0.9999)
            # The generated manifest fingerprints the selected recipe/output.
            self.assertEqual(result.manifest.artifacts.mjcf_sha256, result.mjcf_sha256)

    def test_scene_timestep_is_explicit_and_does_not_change_other_equalities(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self._recipe(Path(directory), "l6_r", {"timestep": 0.001})
            root = ET.fromstring(compose(paths).mjcf_text)
            self.assertEqual(root.find("option").get("timestep"), "0.001")
            self.assertTrue(
                all(e.get("solref") is None for e in root.findall("equality/joint"))
            )
        # Equalities of other components must survive even in a mixed scene.
        root = ET.parse(O6 / "linkerhand_o6_right.xml").getroot()
        other = ET.SubElement(
            root.find("equality"),
            "joint",
            joint1="other_distal",
            joint2="other_proximal",
            solref="0.02 1",
        )
        names = [
            e.get("joint1") for e in root.findall("equality/joint") if e is not other
        ]
        apply_mujoco_options(
            root, MujocoOptions(coupling_profile="o6_coupled"), [names]
        )
        self.assertEqual(
            other.attrib,
            {"joint1": "other_distal", "joint2": "other_proximal", "solref": "0.02 1"},
        )

    def test_invalid_settings_and_incompatible_profiles_fail_loudly(self):
        for settings in (
            None,
            [],
            {"time_step": 0.002},
            {"timestep": 0},
            {"timestep": -1},
            {"timestep": float("nan")},
            {"timestep": float("inf")},
            {"timestep": True},
            {"timestep": "0.001"},
            {"timestep": None},
            {"coupling_profile": None},
            {"coupling_profile": "typo"},
        ):
            with self.subTest(settings=settings), self.assertRaises(SchemaError):
                MujocoOptions.from_dict(settings, "test.mujoco")
        for unit, settings, message in (
            ("o6_r", {"coupling_profile": "o6_coupled", "timestep": 0.004}, "timestep"),
            ("l6_r", {"coupling_profile": "o6_coupled"}, "linkerhand_o6"),
        ):
            with self.subTest(unit=unit), tempfile.TemporaryDirectory() as directory:
                paths = self._recipe(Path(directory), unit, settings)
                with self.assertRaisesRegex(SchemaError, message):
                    compose(paths)
        root = ET.parse(O6 / "linkerhand_o6_right.xml").getroot()
        names = [e.get("joint1") for e in root.findall("equality/joint")]
        root.find("equality").remove(root.find("equality/joint"))
        with self.assertRaisesRegex(SchemaError, "declared"):
            apply_mujoco_options(
                root, MujocoOptions(coupling_profile="o6_coupled"), [names]
            )

    def _recipe(self, directory, name, settings, subdir="units"):
        recipe = yaml.safe_load((ASSETS / subdir / name / "recipe.yaml").read_text())
        recipe["mujoco"] = settings
        (directory / "recipe.yaml").write_text(yaml.safe_dump(recipe, sort_keys=False))
        return resolve_paths(directory, ASSETS)

    def _exercise(self, model, *, cycle):
        result = probe(model, cycle=cycle, duration=8, steady=True)
        self.assertTrue(result["passed"], result)
        self.assertAlmostEqual(result["time"], 8.0, places=8)
        self.assertTrue(all(result["eq_active"]))
        self.assertLess(result["tail_velocity_rad_s"], 0.01)

    def test_contact_and_recontact_settle_on_both_sides(self):
        for side in ("left", "right"):
            for finger in ("thumb", "index"):
                for recontact in (False, True):
                    with self.subTest(side=side, finger=finger, recontact=recontact):
                        model, pose, _ = build_fixture(side, finger)
                        result = probe_contact(model, pose, recontact=recontact)
                        self.assertTrue(result["completed"], result)
                        self.assertFalse(any(result["warnings"]), result)
                        self.assertGreater(result["force_mean"], 20.0)
                        self.assertLess(result["force_std"], 0.1, result)
                        self.assertLess(result["max_velocity"], 0.01, result)
                        self.assertLess(result["coupling_error"], 0.001, result)
                        self.assertEqual(result["contact_fraction"], 1.0)


if __name__ == "__main__":
    unittest.main()
