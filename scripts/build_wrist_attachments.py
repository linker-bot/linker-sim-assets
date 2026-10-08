"""Author the shared AR5 flange and optional Gemini 335L from metre-scale meshes.

With --source-dir, ingest the six reviewed millimetre CAD files once. Without it,
regenerate descriptions/colliders from the committed normalized visual meshes.
Original CAD is never modified. Optical frames and inertias are nominal estimates,
not device calibration. See the component provenance files for their sources.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import scipy
from scipy.spatial.transform import Rotation
import trimesh
import yaml

ASSETS = Path(__file__).resolve().parents[1] / "src/linker_robot_assets/assets"
COMPONENTS = ASSETS / "components"
FLANGE = COMPONENTS / "attachments/ar5_08_l6_o6_flange"
BRACKET = COMPONENTS / "attachments/ar5_08_gemini335l_bracket"
CAMERA = COMPONENTS / "sensors/orbbec_gemini335l"
FRAME_MASS = 1e-9
SECTORS = 24
INPUTS = {
    "法兰装配.STL": "9fce31dd61d60c39c5c91db87070c3fd7d5674b0a67bc686cad9defdb8bc4ab8",
    "335L.STL": "01057f50fb9908be9ae9cee71306e402b42bbcd29d5d395d8b8183aa4d43879c",
    "O6-腕部相机-奥比中光335L-左手.STL": "9d20940ddefa84e2c2fbe84a37670b75f2eef8eb7047e87c091c78152f308eea",
    "O6-腕部相机-奥比中光335L-右手.STL": "9b7e2e956ae729da273e228320598f5c5ddd5edf0128117935051e6616b449a9",
    "法兰装配+相机支架装配体文件.STL": "9ccb581139a4e3ce6cc4560a1326451fad76e88fd3316766200ced6132c32d11",
    "法兰装配+相机支架装配体文件2.STL": "85efbcf81d8d3d9403dfabbd95f0646a06fe3481d4ef5baf3abcf3e772269d79",
}
# Review established the large seating face at CAD Y=-32.2 mm, coaxial
# center X=-7.6, Z=4.499946 mm. Its locating boss enters the arm's 2 mm recess.
CAD_TO_FLANGE = np.array(
    [[1, 0, 0, 0.0076], [0, 0, -1, 0.004499946], [0, 1, 0, 0.0322], [0, 0, 0, 1]],
    dtype=float,
)
# Nominal vendor camera_link is the depth/left-IR origin: X forward, Y left,
# Z up. Register the vendor shell to the user CAD; RGB is 23.75 mm to its right.
CAD_TO_CAMERA = np.array(
    [[0, 0, 1, 0.00408], [1, 0, 0, -0.0475], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float
)


def fmt(values) -> str:
    return " ".join(f"{float(value):.12g}" for value in np.asarray(values).reshape(-1))


def transform(rotation=None, position=(0, 0, 0)):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = rotation
    result[:3, 3] = position
    return result


def cad_camera_mount(side: str):
    # Rigid registration of the separate camera to each supplied assembly.
    rotation = np.array(
        [
            [0, np.cos(np.deg2rad(20)), -np.sin(np.deg2rad(20))],
            [0, np.sin(np.deg2rad(20)), np.cos(np.deg2rad(20))],
            [1, 0, 0],
        ]
    )
    assembly = transform(
        rotation,
        (
            -0.054429933205,
            0.009041398536,
            0.024499946756 if side == "right" else -0.015500053266,
        ),
    )
    return CAD_TO_FLANGE @ assembly @ np.linalg.inv(CAD_TO_CAMERA)


def cad_bracket_mount(side: str):
    return CAD_TO_FLANGE @ transform(
        Rotation.from_euler("x", np.pi / 2).as_matrix(),
        (-0.0076, -0.0112, 0.004499946 if side == "right" else 0.076499946),
    )


def ingest(source: Path):
    for name, expected in INPUTS.items():
        if hashlib.sha256((source / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"unreviewed CAD input: {name}")
    flange = trimesh.load_mesh(source / "法兰装配.STL")
    parts = sorted(
        flange.split(only_watertight=False), key=lambda part: part.volume, reverse=True
    )
    if len(parts) != 2 or not all(part.is_watertight for part in parts):
        raise ValueError("flange must contain the two reviewed closed solids")
    for mesh, name in zip(parts, ("flange_plate", "flange_adapter"), strict=True):
        mesh.apply_scale(0.001)
        mesh.apply_transform(CAD_TO_FLANGE)
        target = FLANGE / "variants/default/meshes" / f"{name}.stl"
        target.parent.mkdir(parents=True, exist_ok=True)
        mesh.export(target)
    for side, chinese in [("left", "左"), ("right", "右")]:
        mesh = trimesh.load_mesh(source / f"O6-腕部相机-奥比中光335L-{chinese}手.STL")
        mesh.apply_scale(0.001)
        mesh.apply_transform(cad_bracket_mount(side))
        target = BRACKET / f"variants/{side}/meshes/bracket.stl"
        target.parent.mkdir(parents=True, exist_ok=True)
        mesh.export(target)
    mesh = trimesh.load_mesh(source / "335L.STL")
    mesh.apply_scale(0.001)
    mesh.apply_transform(CAD_TO_CAMERA)
    target = CAMERA / "variants/default/meshes/gemini335l.stl"
    target.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(target)


def sector_hulls(mesh):
    """Preserve the central mounting recess instead of filling it with one hull.

    Small fastener holes within a sector are conservatively closed. Their mating
    bodies have explicit seam exclusions; this is not a screw-insertion model.
    """
    result = []
    for index in range(SECTORS):
        a, b = np.array([index, index + 1]) * 2 * np.pi / SECTORS
        part = mesh.slice_plane([0, 0, 0], [-np.sin(a), np.cos(a), 0])
        part = part.slice_plane([0, 0, 0], [np.sin(b), -np.cos(b), 0])
        if len(part.vertices) < 4:
            continue
        if (
            np.linalg.matrix_rank(part.vertices - part.vertices.mean(axis=0), tol=1e-9)
            < 3
        ):
            continue
        result.append(part.convex_hull)
    return result


def load_normalized_mesh(path):
    mesh = trimesh.load_mesh(path, process=False)
    # Default 1e-8 metre welding merges distinct small fillet vertices in the
    # flange adapter. Match coincident STL vertices without collapsing that seam.
    mesh.merge_vertices(digits_vertex=10)
    return mesh


def inertia_for_meshes(meshes, mass):
    volumes = np.array([mesh.volume for mesh in meshes])
    if not all(mesh.is_watertight for mesh in meshes) or np.any(volumes <= 0):
        raise ValueError(
            "uniform-density inertia requires closed positive-volume solids"
        )
    masses = volumes / volumes.sum() * mass
    center = np.average([mesh.center_mass for mesh in meshes], axis=0, weights=masses)
    inertia = np.zeros((3, 3))
    for mesh, part_mass in zip(meshes, masses, strict=True):
        delta = mesh.center_mass - center
        inertia += mesh.moment_inertia * (part_mass / mesh.volume) + part_mass * (
            np.eye(3) * np.dot(delta, delta) - np.outer(delta, delta)
        )
    return center, inertia


def write_xml(path, element):
    ET.indent(element, space="  ")
    path.write_text(ET.tostring(element, encoding="unicode") + "\n")


def write_component(
    directory, variant, root_name, visual_names, mass, frames, mounts, *, camera=False
):
    folder = directory / "variants" / variant
    meshes = [
        load_normalized_mesh(folder / "meshes" / f"{name}.stl") for name in visual_names
    ]
    frame_mass = len(frames) * FRAME_MASS
    if camera:
        bounds = meshes[0].bounds
        center = bounds.mean(axis=0)
        box = trimesh.creation.box(
            extents=np.diff(bounds, axis=0)[0], transform=transform(position=center)
        )
        # A box fills the shell's slanted corners and falsely intersects the O6
        # thumb at its normal open pose. The CAD hull remains conservative while
        # retaining those corners (about 1 mm clearance in the reviewed mount).
        hulls = [meshes[0].convex_hull]
        center, inertia = inertia_for_meshes([box], mass - frame_mass)
        inertia_method = "known total mass; conservative shell bounding box; COM/inertia not measured"
    else:
        center, inertia = inertia_for_meshes(meshes, mass - frame_mass)
        hulls = [hull for mesh in meshes for hull in sector_hulls(mesh)]
        inertia_method = (
            "known total mass; uniform density across CAD closed solids; not measured"
        )
    collision_dir = folder / "meshes/collision"
    collision_dir.mkdir(exist_ok=True)
    for index, hull in enumerate(hulls):
        hull.export(collision_dir / f"part_{index:03}.stl")
    collision_files = [f"collision/part_{index:03}.stl" for index in range(len(hulls))]
    for obsolete in set(collision_dir.glob("part_*.stl")) - {
        folder / "meshes" / file for file in collision_files
    }:
        obsolete.unlink()
    urdf = ET.Element("robot", name=directory.name)
    root_link = ET.SubElement(urdf, "link", name=root_name)
    mjcf = ET.Element("mujoco", model=directory.name)
    ET.SubElement(
        mjcf,
        "compiler",
        meshdir="meshes",
        discardvisual="false",
        autolimits="true",
        angle="radian",
        eulerseq="XYZ",
    )
    ET.SubElement(mjcf, "option")
    defaults = ET.SubElement(mjcf, "default")
    for name, attrs in [
        ("visual", {"contype": "0", "conaffinity": "0", "group": "1"}),
        ("collision", {"contype": "1", "conaffinity": "1", "group": "3"}),
    ]:
        ET.SubElement(
            ET.SubElement(defaults, "default", {"class": name}),
            "geom",
            attrib={"type": "mesh", **attrs},
        )
    assets = ET.SubElement(mjcf, "asset")
    body = ET.SubElement(ET.SubElement(mjcf, "worldbody"), "body", name=root_name)
    add_inertia(root_link, body, mass - frame_mass, center, inertia)
    for name in visual_names:
        ET.SubElement(assets, "mesh", name=name, file=f"{name}.stl")
        visual = ET.SubElement(root_link, "visual")
        ET.SubElement(
            ET.SubElement(visual, "geometry"), "mesh", filename=f"meshes/{name}.stl"
        )
        ET.SubElement(
            ET.SubElement(visual, "material", name=f"{name}_material"),
            "color",
            rgba="0.25 0.28 0.32 1",
        )
        ET.SubElement(
            body, "geom", {"class": "visual", "mesh": name, "rgba": "0.25 0.28 0.32 1"}
        )
    for index, file in enumerate(collision_files):
        name = f"collision_{index:03}"
        ET.SubElement(assets, "mesh", name=name, file=file)
        ET.SubElement(
            ET.SubElement(ET.SubElement(root_link, "collision"), "geometry"),
            "mesh",
            filename=f"meshes/{file}",
        )
        ET.SubElement(body, "geom", {"class": "collision", "mesh": name})
    bodies = {root_name: body}
    for name, parent, matrix in frames:
        link = ET.SubElement(urdf, "link", name=name)
        joint = ET.SubElement(urdf, "joint", name=f"{name}_joint", type="fixed")
        rpy = Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz")
        ET.SubElement(joint, "origin", xyz=fmt(matrix[:3, 3]), rpy=fmt(rpy))
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        child = ET.SubElement(
            bodies[parent], "body", name=name, pos=fmt(matrix[:3, 3]), euler=fmt(rpy)
        )
        bodies[name] = child
        add_inertia(link, child, FRAME_MASS, np.zeros(3), np.eye(3) * 1e-12)
    for frame, parent in mounts.items():
        ET.SubElement(bodies[parent], "site", name=frame, pos="0 0 0", size="0.001")
    write_xml(folder / "component.urdf", urdf)
    write_xml(folder / "component.mjcf", mjcf)
    metadata = {
        "schema_version": 1,
        "mass_kg": mass,
        "virtual_frame_mass_kg": frame_mass,
        "inertia_method": inertia_method,
        "length_unit": "m",
        "collision_method": "camera CAD convex hull; slanted shell corners retained"
        if camera
        else "24 angular CAD sectors, convex hull per solid sector",
        "collision_part_count": len(hulls),
        "collision_limits": "conservative approximation; small fastener holes may be filled; not a fastener insertion model",
        "visual_sha256": {
            name: hashlib.sha256(
                (folder / "meshes" / f"{name}.stl").read_bytes()
            ).hexdigest()
            for name in visual_names
        },
        "collision_sha256": {
            file: hashlib.sha256((folder / "meshes" / file).read_bytes()).hexdigest()
            for file in collision_files
        },
        "authoring_versions": {
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "trimesh": trimesh.__version__,
        },
        "source": source_provenance(directory, variant),
    }
    (folder / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")


def source_provenance(directory, variant):
    common = {
        "description": "Hardware-owner supplied CAD; assembly reviewed against arm/hand mounting faces",
        "input_length_unit": "mm",
        "scale_to_m": 0.001,
        "input_sha256": INPUTS,
        "transforms_apply_after_scaling": True,
    }
    if directory == FLANGE:
        return common | {
            "cad_to_component": CAD_TO_FLANGE.tolist(),
            "mass_source": "hardware owner: 44.5 g total for the two flange parts",
            "mount": "large seating face at tool0; 2 mm pilot enters arm recess; small hand seat at +22.5 mm",
        }
    if directory == BRACKET:
        return common | {
            "cad_to_component": cad_bracket_mount(variant).tolist(),
            "component_to_camera_link": cad_camera_mount(variant).tolist(),
            "mass_source": "hardware owner: 29 g per bracket",
        }
    return common | {
        "cad_to_component": CAD_TO_CAMERA.tolist(),
        "mass_source": "Orbbec Gemini 335L product specification: 133 g",
        "product_url": "https://www.orbbec.com/products/stereo-vision-camera/gemini-335l/",
        "nominal_frames_url": "https://github.com/orbbec/OrbbecSDK_ROS2/blob/8e7cad2bfa2c4a6ac4e779be99c64e72166043af/orbbec_description/urdf/gemini_335_L_336_L.urdf.xacro",
        "vendor_shell_registration": {
            "nearest_vertex_rms_m": 0.0002145,
            "nearest_vertex_median_m": 0.0000108,
            "depth_center_in_cad_mm": [47.5, 0, -4.08],
            "vendor_mesh_committed": False,
        },
        "calibration_status": "nominal CAD/vendor reference; not device calibration",
        "optical_axes": "OpenCV: X right, Y down, Z forward",
    }


def add_inertia(link, body, mass, center, inertia):
    item = ET.SubElement(link, "inertial")
    ET.SubElement(item, "origin", xyz=fmt(center), rpy="0 0 0")
    ET.SubElement(item, "mass", value=fmt([mass]))
    ET.SubElement(
        item,
        "inertia",
        attrib={
            key: fmt([inertia[i, j]])
            for key, i, j in [
                ("ixx", 0, 0),
                ("iyy", 1, 1),
                ("izz", 2, 2),
                ("ixy", 0, 1),
                ("ixz", 0, 2),
                ("iyz", 1, 2),
            ]
        },
    )
    ET.SubElement(
        body,
        "inertial",
        pos=fmt(center),
        mass=fmt([mass]),
        fullinertia=fmt(
            [
                inertia[0, 0],
                inertia[1, 1],
                inertia[2, 2],
                inertia[0, 1],
                inertia[0, 2],
                inertia[1, 2],
            ]
        ),
    )


def write_meta(directory, kind, variants, root, mounts):
    data = {
        "schema_version": 1,
        "kind": kind,
        "name": directory.name,
        "variants": {
            side: {
                "vars": {},
                "urdf": f"variants/{side}/component.urdf",
                "mjcf": f"variants/{side}/component.mjcf",
                "meshdir": f"variants/{side}/meshes",
            }
            for side in variants
        },
        "root_link": root,
        "mount_frames": {name: {"parent": parent} for name, parent in mounts.items()},
    }
    (directory / "meta.yaml").write_text(yaml.safe_dump(data, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path)
    args = parser.parse_args()
    if args.source_dir is not None:
        ingest(args.source_dir)
    mounts = {
        "arm_mount": "flange_link",
        "hand_mount": "hand_mount",
        "bracket_mount": "flange_link",
    }
    write_component(
        FLANGE,
        "default",
        "flange_link",
        ["flange_plate", "flange_adapter"],
        0.0445,
        [("hand_mount", "flange_link", transform(position=(0, 0, 0.0225)))],
        mounts,
    )
    write_meta(FLANGE, "attachment", ["default"], "flange_link", mounts)
    mounts = {"flange_mount": "bracket_link", "camera_mount": "camera_mount"}
    for side in ("left", "right"):
        write_component(
            BRACKET,
            side,
            "bracket_link",
            ["bracket"],
            0.029,
            [("camera_mount", "bracket_link", cad_camera_mount(side))],
            mounts,
        )
    write_meta(BRACKET, "attachment", ["left", "right"], "bracket_link", mounts)
    optical = transform(
        Rotation.from_euler("xyz", [-np.pi / 2, 0, -np.pi / 2]).as_matrix()
    )
    frames = [
        ("depth_optical", "camera_link", optical),
        (
            "rgb_frame",
            "camera_link",
            transform(
                Rotation.from_euler("x", 0.0010157).as_matrix(), (0, -0.02375, 0)
            ),
        ),
        ("rgb_optical", "rgb_frame", optical),
        ("right_ir_frame", "camera_link", transform(position=(0, -0.095, 0))),
        ("right_ir_optical", "right_ir_frame", optical),
    ]
    mounts = {
        "base_mount": "camera_link",
        "depth_optical": "depth_optical",
        "rgb_optical": "rgb_optical",
        "right_ir_optical": "right_ir_optical",
    }
    write_component(
        CAMERA,
        "default",
        "camera_link",
        ["gemini335l"],
        0.133,
        frames,
        mounts,
        camera=True,
    )
    write_meta(CAMERA, "sensor", ["default"], "camera_link", mounts)


if __name__ == "__main__":
    main()
