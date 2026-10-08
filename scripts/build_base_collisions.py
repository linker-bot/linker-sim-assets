"""Build named, conservative colliders from pinned semantic CAD assemblies.

Run with the authoring versions recorded in each collision_manifest.json. This
tool only replaces collision elements and adds meshes; visual and inertial
elements, joints and installation frames are preserved.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import scipy
import trimesh
import yaml

from collision_geometry import Polytope, fit_collider, within_envelope
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


def sha(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def numbers(values) -> str:
    return " ".join(f"{value:.15g}" for value in values)


def build(name: str, check: bool = False) -> None:
    directory = ASSETS / "components/bases" / name / "variants/default"
    source_name = "workstation" if name == "bench_table" else "base_link"
    body_name = "workstation_link" if name == "bench_table" else "base_link"
    source = directory / "meshes" / f"{source_name}.STL"
    config_path = directory / "collision_groups.yaml"
    config = yaml.safe_load(config_path.read_text())
    if sha(source.read_bytes()) != config["source_sha256"]:
        raise ValueError(f"{name}: source changed; review semantic shell assignments")
    source_mesh = trimesh.load_mesh(source)
    raw_hulls = [export_stl(hull) for hull in collision_hulls(source_mesh)]
    if (
        len(raw_hulls) != config["source_hull_count"]
        or sha("".join(sha(raw) for raw in raw_hulls).encode())
        != config["source_hulls_sha256"]
    ):
        raise ValueError(
            f"{name}: source hull ordering changed; review assignments/toolchain"
        )
    # Reproduce the exact float32 hulls previously emitted by this tool.
    hulls = [trimesh.load_mesh(io.BytesIO(raw), file_type="stl") for raw in raw_hulls]
    originals = [Polytope.make(hull.vertices) for hull in hulls]
    assigned = [i for g in config["groups"] for p in g["parts"] for i in p["shells"]]
    if sorted(assigned) != list(range(len(hulls))):
        raise ValueError("semantic groups must assign every source hull exactly once")
    names = [g["name"] for g in config["groups"]]
    if len(set(names)) != len(names) or any(
        not re.fullmatch(r"[a-z][a-z0-9_]*", n) for n in names
    ):
        raise ValueError("semantic group names must be unique identifiers")
    outputs = {}
    urdf_elements, mjcf_elements, definitions, parts = [], [], [], []
    for group in config["groups"]:
        tolerance = group["max_added_distance_m"]
        reference = [originals[i] for p in group["parts"] for i in p["shells"]]
        for index, spec in enumerate(group["parts"], 1):
            suffix = f"_{index:02d}" if len(group["parts"]) > 1 else ""
            label = f"{name}_{group['name']}{suffix}"
            fitted = fit_collider(
                [hulls[i] for i in spec["shells"]],
                spec["shape"],
                spec.get("oriented", False),
                spec.get("axis", 2),
            )
            # Checking the full ORIGINAL assembly avoids tolerance accumulation.
            members = [originals[i] for i in spec["shells"]]
            if not within_envelope(
                fitted.envelope, members, tolerance
            ) and not within_envelope(fitted.envelope, reference, tolerance):
                raise ValueError(
                    f"{label}: added envelope exceeds {tolerance} m (or is unproven)"
                )
            record = {
                "name": label,
                "group": group["name"],
                "source_shells": spec["shells"],
                "shape": fitted.kind,
                "max_added_distance_m": tolerance,
            }
            collision = ET.Element("collision", name=label)
            geometry = ET.SubElement(collision, "geometry")
            geom = ET.Element(
                "geom", name=label, attrib={"class": f"base_{name}_collision"}
            )
            if fitted.kind == "mesh":
                assert fitted.mesh is not None
                filename = f"{label}.stl"
                content = export_stl(fitted.mesh)
                outputs[directory / "meshes/collision" / filename] = content
                mesh_name = f"base_collision_{label}"
                definitions.append(
                    f'    <mesh name="{mesh_name}" file="collision/{filename}"/>\n'
                )
                ET.SubElement(geometry, "mesh", filename=f"meshes/collision/{filename}")
                geom.set("mesh", mesh_name)
                record.update(file=filename, sha256=sha(content))
            else:
                collision.insert(
                    0,
                    ET.Element(
                        "origin", xyz=numbers(fitted.position), rpy=numbers(fitted.rpy)
                    ),
                )
                geom.set("type", fitted.kind)
                geom.set("pos", numbers(fitted.position))
                geom.set("quat", numbers(fitted.quaternion_wxyz))
                if fitted.kind == "box":
                    ET.SubElement(geometry, "box", size=numbers(fitted.size))
                    geom.set("size", numbers(fitted.size / 2))
                else:
                    radius, length = fitted.size
                    ET.SubElement(
                        geometry,
                        "cylinder",
                        radius=numbers([radius]),
                        length=numbers([length]),
                    )
                    geom.set("size", numbers([radius, length / 2]))
                record.update(
                    position_m=fitted.position.tolist(),
                    rpy_rad=fitted.rpy.tolist(),
                    size_m=fitted.size.tolist(),
                )
            urdf_elements.append(
                "    " + ET.tostring(collision, encoding="unicode") + "\n"
            )
            mjcf_elements.append(
                "        " + ET.tostring(geom, encoding="unicode") + "\n"
            )
            parts.append(record)
    urdf_path, mjcf_path = directory / "base.urdf", directory / "base.mjcf"
    before = {p.name: invariant(ET.parse(p).getroot()) for p in (urdf_path, mjcf_path)}
    urdf = urdf_path.read_text()
    match = re.search(rf'<link name="{body_name}">.*?</link>', urdf, re.DOTALL)
    if match is None:
        raise ValueError(f"missing URDF body {body_name}")
    link = re.sub(r"\s*<collision\b.*?</collision>", "", match[0], flags=re.DOTALL)
    link = re.sub(r"\s*</link>", "\n" + "".join(urdf_elements) + "  </link>", link)
    urdf = urdf[: match.start()] + link + urdf[match.end() :]
    mjcf = mjcf_path.read_text()
    mjcf = re.sub(
        r'^[ \t]*<mesh name="base_collision_[^"]+"[^>]*/>[ \t]*\n?',
        "",
        mjcf,
        flags=re.MULTILINE,
    )
    mjcf = mjcf.replace("  </asset>", "".join(definitions) + "  </asset>")
    pattern = rf'^[ \t]*<geom\b(?=[^>]*class="base_{name}_collision")[^>]*/>[ \t]*\n?'
    matches = list(re.finditer(pattern, mjcf, re.MULTILINE))
    if not matches:
        raise ValueError(f"missing MJCF collision element for {name}")
    mjcf = re.sub(pattern, "", mjcf, flags=re.MULTILINE)
    offset = matches[0].start()
    mjcf = mjcf[:offset] + "".join(mjcf_elements) + mjcf[offset:]
    after = {
        "base.urdf": invariant(ET.fromstring(urdf)),
        "base.mjcf": invariant(ET.fromstring(mjcf)),
    }
    if before != after:
        raise ValueError("collision generation modified a non-collision XML fact")
    outputs[urdf_path], outputs[mjcf_path] = urdf.encode(), mjcf.encode()
    report = {
        "source_mesh": source.name,
        "source_sha256": config["source_sha256"],
        "groups_sha256": sha(config_path.read_bytes()),
        "source_hull_count": len(hulls),
        "source_hulls_sha256": config["source_hulls_sha256"],
        "method": "semantic convex unions and conservative native primitives",
        "planar_thickness_m": PLANAR_THICKNESS_M,
        "containment_tolerance_m": CONTAINMENT_TOLERANCE_M,
        "envelope_validation": "entire fitted solid within original group union offset; inconclusive fits rejected",
        "authoring_versions": {
            "trimesh": trimesh.__version__,
            "numpy": np.__version__,
            "scipy": scipy.__version__,
        },
        "unchanged_non_collision_xml_sha256": before,
        "collision_count": len(parts),
        "mesh_file_count": sum(p["shape"] == "mesh" for p in parts),
        "parts": parts,
    }
    outputs[directory / "collision_manifest.json"] = (
        json.dumps(report, indent=2) + "\n"
    ).encode()
    # Remove only generated meshes listed in the previous manifest, never sources.
    previous = json.loads((directory / "collision_manifest.json").read_text())
    obsolete = {
        directory / "meshes/collision" / p["file"]
        for p in previous["parts"]
        if "file" in p
    } - outputs.keys()
    if check:
        drift = [
            str(p)
            for p, data in outputs.items()
            if not p.is_file() or p.read_bytes() != data
        ]
        if drift or any(p.exists() for p in obsolete):
            raise ValueError(f"{name}: generated collision drift: {drift}")
    else:
        for path, content in outputs.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        for path in obsolete:
            path.unlink(missing_ok=True)
    print(
        f"{name}: {len(hulls)} -> {len(parts)} colliders, {report['mesh_file_count']} mesh files; envelope/non-collision checks passed",
        flush=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify geometry and generated output without writes",
    )
    parser.add_argument("names", nargs="*", choices=BASES)
    args = parser.parse_args()
    for name in args.names or BASES:
        build(name, check=args.check)
