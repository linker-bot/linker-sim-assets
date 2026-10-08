"""Regress physical installation, added load and the known O6 motion restriction."""

from pathlib import Path
import unittest

import mujoco
import numpy as np


ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"


def model(name):
    return mujoco.MjModel.from_xml_path(
        str(ASSETS / "units" / name / "workstation.xml")
    )


def forward_hand(m, d, side, yaw, pitch):
    d.qpos[:] = m.qpos0
    d.qpos[m.joint(f"hand_{side}h_thumb_cmc_yaw").qposadr] = yaw
    d.qpos[m.joint(f"hand_{side}h_thumb_cmc_pitch").qposadr] = pitch
    for index, kind in enumerate(m.eq_type):
        if kind == mujoco.mjtEq.mjEQ_JOINT:
            a, b = m.eq_obj1id[index], m.eq_obj2id[index]
            d.qpos[m.jnt_qposadr[a]] = np.polynomial.polynomial.polyval(
                d.qpos[m.jnt_qposadr[b]], m.eq_data[index, :5]
            )
    mujoco.mj_forward(m, d)


def attachment_contacts(m, d):
    result = []
    for contact in d.contact:
        names = [m.body(m.geom_bodyid[g]).name for g in contact.geom]
        if (
            contact.dist < -1e-6
            and any(n.startswith("hand_") for n in names)
            and any(n.startswith(("camera_", "bracket_", "flange_")) for n in names)
        ):
            result.append((names, contact.dist))
    return result


class CameraAssemblyTests(unittest.TestCase):
    def test_added_mass_and_actuation_identity(self):
        for hand in ("l6", "o6"):
            for side in ("l", "r"):
                with self.subTest(hand=hand, side=side):
                    plain = model(f"ar5_08_{hand}_{side}")
                    mounted = model(f"ar5_08_{hand}_gemini335l_{side}")
                    arm, fingers = model(f"ar5_08_{side}"), model(f"{hand}_{side}")
                    expected_mass = arm.body_mass.sum() + fingers.body_mass.sum()
                    self.assertAlmostEqual(
                        plain.body_mass.sum() - expected_mass, 0.0445, places=9
                    )
                    self.assertAlmostEqual(
                        mounted.body_mass.sum() - expected_mass, 0.2065, places=9
                    )
                    self.assertEqual(plain.nu, arm.nu + fingers.nu)
                    self.assertEqual(mounted.nu, plain.nu)
                    self.assertEqual(
                        [mounted.joint(j).name for j in mounted.actuator_trnid[:, 0]],
                        [plain.joint(j).name for j in plain.actuator_trnid[:, 0]],
                    )
                    np.testing.assert_array_equal(plain.jnt_range, mounted.jnt_range)
                    np.testing.assert_array_equal(
                        plain.dof_armature, mounted.dof_armature
                    )

    def test_optical_frames_keep_stereo_baseline_and_rgb_offset(self):
        for side in ("l", "r"):
            m = model(f"ar5_08_o6_gemini335l_{side}")
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            depth = m.body("camera_depth_optical").id
            rgb = m.body("camera_rgb_optical").id
            right = m.body("camera_right_ir_optical").id
            self.assertAlmostEqual(
                np.linalg.norm(d.xpos[depth] - d.xpos[right]), 0.095, places=9
            )
            self.assertAlmostEqual(
                np.linalg.norm(d.xpos[depth] - d.xpos[rgb]), 0.02375, places=9
            )
            # Positive optical X must run from left IR toward right IR.
            delta = d.xmat[depth].reshape(3, 3).T @ (d.xpos[right] - d.xpos[depth])
            np.testing.assert_allclose(delta, [0.095, 0, 0], atol=1e-10)

    def test_o6_straight_thumb_contact_is_not_filtered(self):
        for side in ("l", "r"):
            m = model(f"ar5_08_o6_gemini335l_{side}")
            d = mujoco.MjData(m)
            forward_hand(m, d, side, 0.3, 0)
            contacts = attachment_contacts(m, d)
            self.assertTrue(any("camera_camera_link" in names for names, _ in contacts))

    def test_o6_bending_allows_a_combined_route_without_changing_limits(self):
        # These are two verified examples, not a task controller or a universal
        # safe-angle table. The left and right mounting/hand geometry differs.
        for side, pitch, yaw in (("l", 0.5, 0.26), ("r", 0.57, 1.2)):
            m = model(f"ar5_08_o6_gemini335l_{side}")
            d = mujoco.MjData(m)
            route = [(0, p) for p in np.linspace(0, pitch, 51)]
            route += [(y, pitch) for y in np.linspace(0, yaw, 101)]
            for y, p in route:
                with self.subTest(side=side, yaw=y, pitch=p):
                    forward_hand(m, d, side, y, p)
                    self.assertEqual(attachment_contacts(m, d), [])

    def test_top_module_optics_and_inertia_are_in_the_mesh_frame(self):
        m = model("workstation_zed2i")
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        self.assertEqual(m.nu, 0)
        self.assertAlmostEqual(m.body_mass.sum(), 0.290483237157481, places=12)
        a, b = m.body("sensor_left_optical").id, m.body("sensor_right_optical").id
        np.testing.assert_allclose(
            d.xmat[a].reshape(3, 3).T @ (d.xpos[b] - d.xpos[a]), [0.12, 0, 0], atol=1e-8
        )
        # Independently measured uniform-density CAD COM; the legacy Y value
        # was positive, outside the entire negative-Y stand mesh.
        np.testing.assert_allclose(
            m.body_ipos[m.body("sensor_camera_base_link").id],
            [0.0538775743, -0.0338328750, 0.0404391874],
            atol=1.2e-5,
        )


if __name__ == "__main__":
    unittest.main()
