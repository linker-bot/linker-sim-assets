"""Import the pinned O6 1014_v4 geometry; preserve the PR #18 solver/drive policy.

Requires the hardware delivery and an unmodified O6 component from a8c0d475.
Only the changed thumb metacarpal gets a new convex collider. Other colliders
come from the baseline right hand, transformed to the new frames and mirrored.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from trimesh.exchange.obj import export_obj

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "src/linker_robot_assets/assets/components/hands/linkerhand_o6"
SOURCE_HASHES = {
    "left": "6d4ec7c22f8d8ceda3781fe8fc6ef23e9db4bb17982c6872686c75371f79ded6",
    "right": "849e2d8600d537a30da9895c7b256af252fafe0cc62009a389321472e7881698",
}
BASELINE_HASHES = {
    "linkerhand_o6_left.urdf": "09864cfe9168fd058dfe3c5f496fd3592f012b8f874e19788986c07cef06065e",
    "linkerhand_o6_right.urdf": "cbd15e10d563637d3c0fe6f9e65031b07af8316ab3e399664b427e2dd183a662",
    "linkerhand_o6_left.xml": "3d50feb0597d15a600f2be9ceedfdfe3f288a603a2a22cf0ac0f084f8cc45f85",
    "linkerhand_o6_right.xml": "a421903f8f8ed498f33909d2b86e3b182a5b7e51abbcbf30acc198aa58250caa",
}
REFLECT = np.diag([1.0, -1.0, 1.0, 1.0])


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values) -> str:
    return " ".join(f"{v:.15g}" for v in values)


def origin(element: ET.Element) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler(
        "xyz", np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    ).as_matrix()
    result[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    return result


def required(parent: ET.Element, path: str) -> ET.Element:
    element = parent.find(path)
    if element is None:
        raise ValueError(f"Missing required XML element: {path}")
    return element


def frames(root: ET.Element) -> dict[str, np.ndarray]:
    children = {required(j, "child").attrib["link"] for j in root.findall("joint")}
    result = {
        link.attrib["name"]: np.eye(4)
        for link in root.findall("link")
        if link.attrib["name"] not in children
    }
    todo = list(root.findall("joint"))
    while todo:
        pending = []
        for joint in todo:
            parent = required(joint, "parent").attrib["link"]
            if parent not in result:
                pending.append(joint)
                continue
            result[required(joint, "child").attrib["link"]] = result[parent] @ origin(
                required(joint, "origin")
            )
        if len(pending) == len(todo):
            raise ValueError("Unresolved URDF tree")
        todo = pending
    return result


def write_xml(root: ET.Element, path: Path) -> None:
    ET.indent(root, space="  ")
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")


def tensor(element: ET.Element) -> np.ndarray:
    xx, yy, zz, xy, xz, yz = (
        float(element.attrib[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    )
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def build(source_dir: Path, baseline_dir: Path) -> None:
    for name, expected in BASELINE_HASHES.items():
        if sha(baseline_dir / name) != expected:
            raise ValueError(f"Unexpected baseline revision: {name}")
    sources = {}
    supplied = {}
    for side, expected in SOURCE_HASHES.items():
        path = source_dir / f"o6_{side}/linkerhand_o6_{side}--1014_v4.urdf"
        if sha(path) != expected:
            raise ValueError(f"Unexpected {side} source URDF revision")
        supplied[side] = ET.parse(path).getroot()
        sources[str(path.relative_to(source_dir))] = sha(path)
    baseline = {
        side: ET.parse(baseline_dir / f"linkerhand_o6_{side}.urdf").getroot()
        for side in supplied
    }
    right = supplied["right"]
    old_frames = frames(baseline["right"])
    new_frames = frames(right)
    anchor_shift = np.eye(4)
    anchor_shift[:3, 3] = (
        new_frames["thumb_metacarpals_base2"][:3, 3]
        - old_frames["rh_thumb_metacarpals_base2"][:3, 3]
    )
    meshes = {}
    transformations = {}
    baseline_sources = {}
    for link in right.findall("link"):
        name = link.attrib["name"]
        mesh_path = (
            source_dir
            / "o6_right"
            / required(link, "visual/geometry/mesh").attrib["filename"]
        )
        sources[str(mesh_path.relative_to(source_dir))] = sha(mesh_path)
        visual = trimesh.load(mesh_path, force="mesh", process=False)
        if not isinstance(visual, trimesh.Trimesh):
            raise ValueError(f"Expected triangle mesh: {name}")
        # The thumb anchor moves physically. Remove that change before deriving
        # the old-to-new LOCAL coordinate transform for the reused colliders.
        transform = np.eye(4)
        if name.startswith("thumb_"):
            transform = (
                np.linalg.inv(new_frames[name])
                @ anchor_shift
                @ old_frames[f"rh_{name}"]
            )
        transformations[name] = transform.tolist()
        old_collision = baseline_dir / f"meshes/collision/rh_{name}_col.obj"
        baseline_sources[str(old_collision.relative_to(baseline_dir))] = sha(
            old_collision
        )
        if name == "thumb_metacarpals":
            collision = visual.convex_hull
            method = "Convex hull of 1014 thumb metacarpal; fills concavities, no hidden mesh decimation."
            # Every source vertex must lie inside every supporting plane.
            from scipy.spatial import ConvexHull

            hull = ConvexHull(np.asarray(visual.vertices))
            maximum = max(
                float(np.max(visual.vertices @ plane[:3] + plane[3]))
                for plane in hull.equations
            )
            if maximum > 1e-9:
                raise ValueError("New thumb collider does not enclose the source")
        else:
            collision = trimesh.load(old_collision, force="mesh", process=False)
            if not isinstance(collision, trimesh.Trimesh):
                raise ValueError(f"Expected baseline triangle mesh: {name}")
            collision.apply_transform(transform)
            method = "Reuse baseline right simplified collider in the corresponding 1014 local frame."
        meshes[name] = {
            "source_visual_faces": len(visual.faces),
            "collision_faces": len(collision.faces),
            "collision_method": method,
        }
        for side in ("left", "right"):
            prefix = side[0] + "h_"
            vis, col = visual.copy(), collision.copy()
            if side == "left":
                # trimesh applies inverse-transpose normals and reverses winding
                # for a reflection. URDF/MJCF then use only positive unit scale.
                vis.apply_transform(REFLECT)
                col.apply_transform(REFLECT)
            vis.export(COMPONENT / f"meshes/visual/{prefix}{name}_vis.glb")
            (COMPONENT / f"meshes/visual/{prefix}{name}_vis.obj").write_text(
                export_obj(vis, include_color=False, include_texture=False)
            )
            (COMPONENT / f"meshes/collision/{prefix}{name}_col.obj").write_text(
                export_obj(col, include_color=False, include_texture=False)
            )

    for side in ("left", "right"):
        prefix = side[0] + "h_"
        urdf = deepcopy(supplied[side])
        # Supplied left tensors copy right values. Correct the Y reflection;
        # masses and COMs already mirror exactly in the supplied link frames.
        for link in urdf.findall("link"):
            name = link.attrib["name"]
            if side == "left":
                inertia = required(link, "inertial/inertia")
                for key in ("ixy", "iyz"):
                    inertia.set(key, f"{-float(inertia.attrib[key]):.15g}")
            link.set("name", prefix + name)
            for kind in ("visual", "collision"):
                element = required(link, kind)
                required(element, "origin").set("xyz", "0 0 0")
                required(element, "origin").set("rpy", "0 0 0")
                suffix, extension = (
                    ("vis", "glb") if kind == "visual" else ("col", "obj")
                )
                mesh = required(element, "geometry/mesh")
                mesh.attrib.clear()
                mesh.set(
                    "filename", f"meshes/{kind}/{prefix}{name}_{suffix}.{extension}"
                )
                if kind == "visual":
                    material = element.find("material")
                    if material is not None:
                        element.remove(material)
                    ET.SubElement(
                        element,
                        "material",
                        name="BlackAccent"
                        if name == "thumb_metacarpals_base2"
                        else "silver",
                    )
        for joint in urdf.findall("joint"):
            name = joint.attrib["name"]
            old = required(baseline[side], f"joint[@name='{prefix}{name}']")
            joint.set("name", prefix + name)
            for tag in ("parent", "child"):
                element = required(joint, tag)
                element.set("link", prefix + element.attrib["link"])
            mimic = joint.find("mimic")
            if mimic is not None:
                mimic.set("joint", prefix + mimic.attrib["joint"])
            for attr in ("effort", "velocity"):
                required(joint, "limit").set(attr, required(old, "limit").attrib[attr])
            dynamics = joint.find("dynamics")
            if dynamics is not None:
                joint.remove(dynamics)
        write_xml(urdf, COMPONENT / f"linkerhand_o6_{side}.urdf")
        baseline_xml = baseline_dir / f"linkerhand_o6_{side}.xml"
        baseline_sources[baseline_xml.name] = sha(baseline_xml)
        mjcf = ET.parse(baseline_xml).getroot()
        for link in urdf.findall("link"):
            body = required(mjcf, f".//body[@name='{link.attrib['name']}']")
            old_inertial = required(body, "inertial")
            body.remove(old_inertial)
            inertial = required(link, "inertial")
            values = tensor(required(inertial, "inertia"))
            ET.SubElement(
                body,
                "inertial",
                mass=required(inertial, "mass").attrib["value"],
                pos=required(inertial, "origin").attrib["xyz"],
                fullinertia=vector(values[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]),
            )
        for joint in urdf.findall("joint"):
            name = joint.attrib["name"]
            body = required(
                mjcf, f".//body[@name='{required(joint, 'child').attrib['link']}']"
            )
            for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis"):
                body.attrib.pop(key, None)
            trans = origin(required(joint, "origin"))
            body.set("pos", vector(trans[:3, 3]))
            q = Rotation.from_matrix(trans[:3, :3]).as_quat(scalar_first=True)
            body.set("quat", vector(q))
            axis = required(body, f"joint[@name='{name}']")
            axis.set("axis", required(joint, "axis").attrib["xyz"])
            limit = required(joint, "limit")
            axis.set("range", f"{limit.attrib['lower']} {limit.attrib['upper']}")
            act = mjcf.find(f"actuator/position[@joint='{name}']")
            if act is not None:
                act.set("ctrlrange", axis.attrib["range"])
            mimic = joint.find("mimic")
            if mimic is not None:
                eq = required(mjcf, f"equality/joint[@joint1='{name}']")
                eq.set(
                    "polycoef",
                    f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
                )
        write_xml(mjcf, COMPONENT / f"linkerhand_o6_{side}.xml")
    for side in ("left", "right"):
        path = baseline_dir / f"linkerhand_o6_{side}.urdf"
        baseline_sources[path.name] = sha(path)
    outputs = [
        *COMPONENT.glob("*.urdf"),
        *COMPONENT.glob("*.xml"),
        *COMPONENT.glob("meshes/visual/*"),
        *COMPONENT.glob("meshes/collision/*"),
    ]
    provenance = {
        "revision": "1014_v4",
        "baseline_commit": "a8c0d475d6d56c03c67cb6ab33c398f5b7463c02",
        "hardware_sources": sources,
        "baseline_sources": baseline_sources,
        "mass_kg_each": 0.36998,
        "inertia_policy": "Use hardware masses/COM/right tensors; reflect left ixy/iyz. These supplied values are not independently measured here.",
        "controller_policy": "Keep PR #18 component gains, force caps, options, passive dynamics and constraint/contact settings; no application gain changes.",
        "mesh_policy": "Metres; bake left Y reflection with winding correction; GLB visual and OBJ visual/collision; no negative runtime mesh scales.",
        "baseline_right_to_new_local": transformations,
        "meshes": meshes,
        "outputs": {str(p.relative_to(COMPONENT)): sha(p) for p in sorted(outputs)},
    }
    (COMPONENT / "provenance_1014.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            {
                "collision_faces_each": sum(
                    m["collision_faces"] for m in meshes.values()
                ),
                "mass_kg_each": 0.36998,
            }
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--baseline-dir", type=Path, required=True)
    args = parser.parse_args()
    build(args.source_dir, args.baseline_dir)
