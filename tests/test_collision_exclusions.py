"""Installation seams must not hide collisions with distant component links."""

from pathlib import Path
from contextlib import redirect_stderr, redirect_stdout
import io
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import yaml

from linker_robot_assets.composer.compose import compose, resolve_paths
from linker_robot_assets.composer.schemas import CollisionExclusion, Recipe, SchemaError
from linker_robot_assets.validate_workstation import validate


ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"


class CollisionExclusionTests(unittest.TestCase):
    def test_invalid_declarations_are_rejected(self):
        valid = {"body1": "arm:tool", "body2": "hand:palm", "reason": "Mount seam"}
        for patch in (
            {"body1": "tool"},
            {"body2": "hand:"},
            {"body2": "arm:palm"},
            {"reason": " "},
            {"body1": None},
            {"extra": "typo"},
        ):
            with self.subTest(patch=patch), self.assertRaises(SchemaError):
                CollisionExclusion.from_dict(valid | patch, "test")

    def test_missing_body_and_duplicate_resolved_pairs_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = self._fixture(Path(directory))
            recipe = yaml.safe_load(paths.recipe.read_text())
            for body in ("arm:missing", "unknown:tool"):
                recipe["collision_exclusions"][0]["body1"] = body
                paths.recipe.write_text(yaml.safe_dump(recipe))
                with self.subTest(body=body), self.assertRaises(SchemaError):
                    compose(paths)
            recipe["collision_exclusions"][0]["body1"] = "arm:tool"
            recipe["collision_exclusions"].append(
                {"body1": "hand:palm", "body2": "arm:tool", "reason": "Duplicate"}
            )
            paths.recipe.write_text(yaml.safe_dump(recipe))
            with self.assertRaisesRegex(SchemaError, "duplicate"):
                Recipe.load(paths.recipe)

    def test_distal_contact_remains_active_and_only_declared_seam_is_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            result = compose(self._fixture(Path(directory)))
            assert result.mjcf_text is not None
            root = ET.fromstring(result.mjcf_text)
            self.assertEqual(
                [e.attrib for e in root.findall("contact/exclude")],
                [{"body1": "arm_tool", "body2": "hand_palm"}],
            )
            model = mujoco.MjModel.from_xml_string(result.mjcf_text)
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            contacts = {
                frozenset(model.body(model.geom_bodyid[g]).name for g in c.geom)
                for c in data.contact
            }
            self.assertIn(frozenset(("arm_base", "hand_finger")), contacts)
            self.assertNotIn(frozenset(("arm_tool", "hand_palm")), contacts)

    def test_all_assets_pass_full_validation(self):
        for path in sorted(ASSETS.glob("*/**/recipe.yaml")):
            with self.subTest(asset=path.parent.name):
                output = io.StringIO()
                with redirect_stdout(output), redirect_stderr(output):
                    status = validate(path.parent, None)
                self.assertEqual(status, 0, output.getvalue())

    def _fixture(self, directory):
        components = directory / "components"
        for role, root_name, tip, joint, kind in (
            ("arm", "base", "tool", "hinge", "arm"),
            ("hand", "palm", "finger", "slide", "hand"),
        ):
            component = components / role
            component.mkdir(parents=True)
            offset = "0 0 0.5" if role == "arm" else "0 0 -0.5"
            (component / "model.xml").write_text(f'''<mujoco>
              <compiler angle="radian" eulerseq="XYZ"/>
              <default><default class="shape_collision"><geom type="sphere" size="0.1"/></default></default>
              <worldbody><body name="{root_name}">
                <geom class="shape_collision"/>
                <body name="{tip}" pos="{offset}">
                  <joint name="joint" type="{joint}"/>
                  <geom class="shape_collision"/>
                </body>
              </body></worldbody>
            </mujoco>''')
            (component / "model.urdf").write_text(f'''<robot name="{role}">
              <link name="{root_name}"/><link name="{tip}"/>
              <joint name="joint" type="continuous"><parent link="{root_name}"/><child link="{tip}"/>
                <origin xyz="{offset}"/><axis xyz="0 0 1"/></joint>
            </robot>''')
            metadata = {
                "kind": kind,
                "name": role,
                "root_link": root_name,
                "variants": {
                    "default": {
                        "urdf": "model.urdf",
                        "mjcf": "model.xml",
                        "meshdir": ".",
                    }
                },
                "mount_frames": {"root": {"parent": root_name}, "tip": {"parent": tip}},
                "actuated_joints": ["joint"],
            }
            if role == "arm":
                metadata["ee_frame"] = "tip"
            (component / "meta.yaml").write_text(yaml.safe_dump(metadata))
        unit = directory / "units/test"
        unit.mkdir(parents=True)
        (unit / "recipe.yaml").write_text(
            yaml.safe_dump(
                {
                    "name": "test",
                    "components": {
                        "arm": {"component": "arm"},
                        "hand": {"component": "hand"},
                    },
                    "freeze_base": "arm",
                    "mounts": [{"child": "hand:root", "parent": "arm:tip"}],
                    "collision_exclusions": [
                        {
                            "body1": "arm:tool",
                            "body2": "hand:palm",
                            "reason": "Mount seam",
                        }
                    ],
                },
                sort_keys=False,
            )
        )
        return resolve_paths(unit, directory)


if __name__ == "__main__":
    unittest.main()
