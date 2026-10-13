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
                self.assertAlmostEqual(corrected.opt.timestep, 0.0000625)
                for field in (
                    "dof_damping",
                    "dof_frictionloss",
                    "dof_armature",
                    "body_mass",
                    "body_inertia",
                    "body_ipos",
                    "actuator_gainprm",
                    "actuator_biasprm",
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

    def test_step_command_has_bounded_transient(self):
        self._exercise(load(O6 / "coupled/right.xml"), cycle=False)

    def test_composed_solo_hands_keep_motors_and_follow_cycle(self):
        for side in ("l", "r"):
            with self.subTest(side=side), tempfile.TemporaryDirectory() as directory:
                paths = self._recipe(
                    Path(directory), f"o6_{side}", {"coupling_profile": "o6_coupled"}
                )
                result = compose(paths)
                paths.out_mjcf.write_text(result.mjcf_text)
                model = load(paths.out_mjcf)
                self.assertAlmostEqual(model.opt.timestep, 0.0000625)
                np.testing.assert_array_equal(model.actuator_gainprm[:, 0], 1)
                np.testing.assert_array_equal(model.actuator_biasprm, 0)
                self._exercise(model, cycle=True, motors=True)

    def test_recipe_profile_applies_to_both_hands_in_full_workstation(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self._recipe(
                Path(directory),
                "ar5_08_o6_bench_bimanual",
                {"coupling_profile": "o6_coupled", "timestep": 0.00003125},
                subdir="workstations",
            )
            result = compose(paths)
            paths.out_mjcf.write_text(result.mjcf_text)
            model = load(paths.out_mjcf)
            self.assertAlmostEqual(model.opt.timestep, 0.00003125)
            self.assertEqual(model.neq, 10)
            np.testing.assert_allclose(model.eq_solref[:, 0], 0.000125)
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
            ("o6_r", {"coupling_profile": "o6_coupled", "timestep": 0.002}, "timestep"),
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

    def _exercise(self, model, *, cycle, motors=False):
        """No qpos writes: commands only, with checks at every integration step."""
        data = mujoco.MjData(model)
        active = model.jnt_qposadr[model.eq_obj2id]
        passive = model.jnt_qposadr[model.eq_obj1id]
        commanded = model.jnt_qposadr[model.actuator_trnid[:, 0]]
        targets = np.array([0.0, 0.3, 0.8, 0.8, 0.8, 0.8])
        peak_error = 0.0
        max_contact = 0
        duration = 4.0 if cycle else 2.0
        for i in range(round(duration / model.opt.timestep)):
            if cycle:
                scale = (
                    min(data.time, 1.0) if data.time < 2 else max(3.0 - data.time, 0.0)
                )
            else:
                scale = 1.0
            target = targets * scale
            # Composed models expose torque motors. Apply the same P law as
            # the component position servos, without powering passive DOFs.
            data.ctrl[:] = 200 * (target - data.qpos[commanded]) if motors else target
            mujoco.mj_step(model, data)
            self.assertFalse(data.warning.number.any(), "Warning or automatic reset")
            self.assertTrue(np.isfinite(data.qpos).all())
            residual = data.qpos[passive] - model.eq_data[:, 1] * data.qpos[active]
            peak_error = max(peak_error, float(abs(residual).max()))
            max_contact = max(max_contact, data.ncon)
            self.assertLess(float(np.max(model.jnt_range[:, 0] - data.qpos)), 0.001)
            self.assertLess(float(np.max(data.qpos - model.jnt_range[:, 1])), 0.001)
            if i == round(1.9 / model.opt.timestep):
                # A broken implementation must not pass by keeping both
                # active and passive joints near zero.
                self.assertGreater(float(data.qpos[active[1:]].min()), 0.7)
                self.assertGreater(float(data.qpos[passive[1:]].min()), 0.65)
        self.assertAlmostEqual(data.time, duration, places=9)
        self.assertEqual(max_contact, 0)
        np.testing.assert_array_equal(data.eq_active, [1] * 5)
        self.assertLess(peak_error, 0.005 if cycle else 0.05)
        self.assertLess(float(abs(residual).max()), 0.001)
        expected = np.zeros(6) if cycle else targets
        np.testing.assert_allclose(data.qpos[commanded], expected, atol=0.005)


if __name__ == "__main__":
    unittest.main()
