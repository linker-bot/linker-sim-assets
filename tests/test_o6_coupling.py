"""O6 coupling must survive composition and real actuator-driven dynamics."""

import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np

from linker_robot_assets.composer.compose import compose, resolve_paths
from linker_robot_assets.validate_hand_coupling import probe
from linker_robot_assets.validate_o6_contact import build_fixture, probe_contact

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"
O6 = ASSETS / "components/hands/linkerhand_o6"


def load(path):
    return mujoco.MjModel.from_xml_path(str(path))


class O6CouplingTests(unittest.TestCase):
    def test_customer_ctrl_and_step_sequence_now_moves_dip(self):
        model = load(O6 / "linkerhand_o6_right.xml")
        data = mujoco.MjData(model)
        data.ctrl[:] = [0, 0.3, 0.8, 0.8, 0.8, 0.8]
        for _ in range(2500):
            mujoco.mj_step(model, data)
            self.assertFalse(data.warning.number.any())
        self.assertAlmostEqual(data.time, 5.0, places=9)
        self.assertTrue(data.eq_active.all())
        active = data.joint("rh_index_mcp_pitch").qpos[0]
        passive = data.joint("rh_index_dip").qpos[0]
        self.assertAlmostEqual(active, 0.8, delta=0.005)
        self.assertAlmostEqual(passive, 0.89 * active, delta=0.001)

    def test_default_component_contract(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                model = load(O6 / f"linkerhand_o6_{side}.xml")
                self.assertEqual(model.nu, 6)
                self.assertEqual(model.neq, 5)
                self.assertAlmostEqual(model.opt.timestep, 0.002)
                np.testing.assert_array_equal(model.actuator_gainprm[:, 0], 600)
                np.testing.assert_array_equal(model.actuator_biasprm[:, 2], -4.5)
                np.testing.assert_array_equal(model.eq_solref, [[-1e7, -63000]] * 5)
                # The fix tunes constraints/servos, not physical joint dynamics.
                np.testing.assert_allclose(model.dof_damping, 4.0)
                np.testing.assert_allclose(model.dof_frictionloss, 0.5)
                np.testing.assert_allclose(model.dof_armature, 3.4707e-08)
                self.assertFalse(
                    np.isin(model.eq_obj1id, model.actuator_trnid[:, 0]).any()
                )
                caps = [1, 4 if side == "left" else 3.8148, 2.55, 2.55, 2.55, 2.55]
                np.testing.assert_allclose(model.actuator_forcerange[:, 1], caps)
                np.testing.assert_allclose(
                    model.actuator_forcerange[:, 0], -np.array(caps)
                )

    def test_default_hands_follow_open_close_cycle(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                self._exercise(load(O6 / f"linkerhand_o6_{side}.xml"), cycle=True)

    def test_default_hands_settle_while_holding(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                self._exercise(load(O6 / f"linkerhand_o6_{side}.xml"), cycle=False)

    def test_default_composition_preserves_torque_interface_and_coupling(self):
        # The shared Isaac assets deliberately use external drives. Replacing
        # component XMLs must not silently turn their torque inputs into angles.
        for side in ("l", "r"):
            with self.subTest(side=side), tempfile.TemporaryDirectory() as directory:
                path = Path(directory)
                (path / "recipe.yaml").write_bytes(
                    (ASSETS / f"units/o6_{side}/recipe.yaml").read_bytes()
                )
                paths = resolve_paths(path, ASSETS)
                result = compose(paths)
                paths.out_mjcf.write_text(result.mjcf_text)
                model = load(paths.out_mjcf)
                np.testing.assert_array_equal(model.actuator_gainprm[:, 0], 1)
                np.testing.assert_array_equal(model.actuator_biasprm, 0)
                np.testing.assert_array_equal(model.eq_solref, [[-1e7, -63000]] * 5)
                gains = result.manifest.default_gains["hand"]
                for cycle in (False, True):
                    result = probe(
                        model,
                        motor_gains=(gains.stiffness, gains.damping),
                        duration=8,
                        steady=True,
                        cycle=cycle,
                    )
                    self.assertTrue(result["passed"], result)

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
