"""Build conservative CAD-part hulls instead of one hull around an entire stand.

Run with the authoring versions recorded in each collision_manifest.json. This
tool only replaces collision elements and adds meshes; visual and inertial
elements, joints and installation frames are preserved.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import scipy
import trimesh
from trimesh.exchange.stl import export_stl


ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"
BASES = ("a7_torso", "a7_lite_torso", "p7_torso", "bench_table")
# Some legacy STL shells are open planar patches. A 0.1 mm prism keeps those
# surfaces collidable without inflating the whole stand into its bounding hull.
PLANAR_THICKNESS_M = 0.0001
CONTAINMENT_TOLERANCE_M = 1e-8


def invariant(root: ET.Element) -> str:
    """Fingerprint every XML fact except collision-only geometry definitions."""
    for parent in root.iter():
        for child in list(parent):
            if (
                child.tag == "collision"
                or child.tag == "geom"
                and child.get("class", "").endswith("_collision")
                or child.tag == "mesh"
                and child.get("name", "").startswith("base_collision_")
            ):
                parent.remove(child)
    for child in root.iter():
        child.text = None
        child.tail = None
    return hashlib.sha256(ET.tostring(root)).hexdigest()


def collision_hulls(mesh: trimesh.Trimesh) -> list[trimesh.Trimesh]:
    hulls = []
    for part in mesh.split(only_watertight=False):
        if part.area == 0:
            continue
        vertices = np.unique(part.vertices, axis=0)
        centered = vertices - vertices.mean(axis=0)
        rank = np.linalg.matrix_rank(centered, tol=1e-8)
        if rank < 3:
            _, _, axes = np.linalg.svd(centered, full_matrices=True)
            for normal in axes[rank:]:
                vertices = np.vstack(
                    (
                        vertices + normal * PLANAR_THICKNESS_M / 2,
                        vertices - normal * PLANAR_THICKNESS_M / 2,
                    )
                )
        if np.linalg.matrix_rank(vertices - vertices.mean(axis=0), tol=1e-8) < 3:
            raise ValueError("source contains a degenerate line/point shell")
        hull = trimesh.convex.convex_hull(vertices)
        equations = ConvexHull(hull.vertices).equations
        hulls.append((hull, equations))
    # Remove a part only when ALL its vertices lie inside another convex part.
    # This does not remove outer CAD surfaces or join disconnected structural
    # pieces across the free space in which the robot moves.
    kept = []
    for hull, equations in sorted(hulls, key=lambda item: -item[0].volume):
        if any(
            np.all(hull.bounds[0] >= other.bounds[0] - CONTAINMENT_TOLERANCE_M)
            and np.all(hull.bounds[1] <= other.bounds[1] + CONTAINMENT_TOLERANCE_M)
            and np.max(hull.vertices @ planes[:, :3].T + planes[:, 3])
            <= CONTAINMENT_TOLERANCE_M
            for other, planes in kept
        ):
            continue
        kept.append((hull, equations))
    return [hull for hull, _ in kept]


def build(name: str) -> None:
    directory = ASSETS / "components/bases" / name / "variants/default"
    source_name = "workstation" if name == "bench_table" else "base_link"
    body_name = "workstation_link" if name == "bench_table" else "base_link"
    source = directory / "meshes" / f"{source_name}.STL"
    urdf_path, mjcf_path = directory / "base.urdf", directory / "base.mjcf"
    before = {p.name: invariant(ET.parse(p).getroot()) for p in (urdf_path, mjcf_path)}
    mesh = trimesh.load_mesh(source)
    hulls = collision_hulls(mesh)
    output = directory / "meshes/collision"
    output.mkdir(exist_ok=True)
    files = []
    for index, hull in enumerate(hulls):
        filename = f"base_part_{index:03d}.stl"
        content = export_stl(hull)
        (output / filename).write_bytes(content)
        files.append({"file": filename, "sha256": hashlib.sha256(content).hexdigest()})
    obsolete = set(output.glob("base_part_*.stl")) - {output / f["file"] for f in files}
    for path in obsolete:
        path.unlink()
    urdf = urdf_path.read_text()
    match = re.search(rf'<link name="{body_name}">.*?</link>', urdf, re.DOTALL)
    if match is None:
        raise ValueError(f"missing URDF body {body_name}")
    link = re.sub(r"\s*<collision\b.*?</collision>", "", match[0], flags=re.DOTALL)
    elements = "".join(
        f'    <collision><geometry><mesh filename="meshes/collision/{f["file"]}"/></geometry></collision>\n'
        for f in files
    )
    link = re.sub(r"\s*</link>", "\n" + elements + "  </link>", link)
    urdf = urdf[: match.start()] + link + urdf[match.end() :]
    mjcf = mjcf_path.read_text()
    mjcf = re.sub(
        r'^[ \t]*<mesh name="base_collision_\d+"[^>]*/>[ \t]*\n?',
        "",
        mjcf,
        flags=re.MULTILINE,
    )
    definitions = "".join(
        f'    <mesh name="base_collision_{i:03d}" file="collision/{f["file"]}"/>\n'
        for i, f in enumerate(files)
    )
    mjcf = mjcf.replace("  </asset>", definitions + "  </asset>")
    pattern = rf'^[ \t]*<geom class="base_{name}_collision" mesh="(?:torso_base_link|workstation|base_collision_\d+)"[ \t]*/>[ \t]*\n?'
    matches = list(re.finditer(pattern, mjcf, re.MULTILINE))
    if not matches:
        raise ValueError(f"missing MJCF collision element for {name}")
    elements = "".join(
        f'        <geom class="base_{name}_collision" mesh="base_collision_{i:03d}"/>\n'
        for i in range(len(files))
    )
    mjcf = re.sub(pattern, "", mjcf, flags=re.MULTILINE)
    # Insert at the original first collider offset after removing all previous
    # generated colliders, so rerunning does not accumulate new geometry.
    offset = matches[0].start()
    mjcf = mjcf[:offset] + elements + mjcf[offset:]
    after = {
        "base.urdf": invariant(ET.fromstring(urdf)),
        "base.mjcf": invariant(ET.fromstring(mjcf)),
    }
    if before != after:
        raise ValueError("collision generation modified a non-collision XML fact")
    urdf_path.write_text(urdf)
    mjcf_path.write_text(mjcf)
    report = {
        "source_mesh": source.name,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "method": "convex hull per connected CAD shell; contained hulls removed",
        "planar_thickness_m": PLANAR_THICKNESS_M,
        "containment_tolerance_m": CONTAINMENT_TOLERANCE_M,
        "authoring_versions": {
            "trimesh": trimesh.__version__,
            "numpy": np.__version__,
            "scipy": scipy.__version__,
        },
        "unchanged_non_collision_xml_sha256": before,
        "source_surface_area_m2": float(mesh.area),
        "original_whole_hull_volume_m3": float(mesh.convex_hull.volume),
        "sum_part_hull_volume_m3": sum(float(h.volume) for h in hulls),
        "parts": files,
    }
    (directory / "collision_manifest.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(f"{name}: {len(files)} convex parts; non-collision XML unchanged", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*", choices=BASES)
    args = parser.parse_args()
    for name in args.names or BASES:
        build(name)
