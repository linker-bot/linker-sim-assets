"""Reproduce fixed-base, frictionless O6 fingertip/plate tests from shipped assets.

    python -m linker_robot_assets.validate_o6_contact --output contact.json

Recorded fixture poses originate from the DemoGrasp 2026-10-08 diagnostics.
Geometry and numerical parameters are loaded from the current coupled assets.
The thumb yaw is fixtured; flexion and its passive joint remain dynamic.
"""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from . import asset_root
from .composer.o6_profile import O6_CONTACT_SOLIMP, O6_CONTACT_SOLREF


def build_fixture(side, finger, *, assets_root=None):
    component = (assets_root or asset_root()) / "components/hands/linkerhand_o6"
    path = component / "coupled" / f"{side}.xml"
    root = ET.parse(path).getroot()
    root.find("compiler").set("meshdir", str(component / "meshes"))
    poses = json.loads(
        (Path(__file__).parent / "validation_data/o6_contact_poses.json").read_text()
    )
    ref = poses[f"{side}_{finger}"]
    prefix = f"{side[0]}h_"
    hand = root.find(f"worldbody/body[@name='{prefix}hand_base_link']")
    hand.set("pos", " ".join(map(str, ref["root_position"])))
    hand.set("quat", " ".join(map(str, ref["root_quaternion"])))
    for body in list(hand):
        if body.tag == "body" and not body.get("name", "").startswith(
            prefix + finger + "_"
        ):
            hand.remove(body)
    if finger == "thumb":
        yaw = hand.find(f".//body[@name='{prefix}thumb_metacarpals_base2']")
        for key in ("euler", "axisangle", "xyaxes", "zaxis"):
            yaw.attrib.pop(key, None)
        yaw.set("quat", " ".join(map(str, ref["thumb_yaw_fixture_quaternion"])))
        yaw.remove(yaw.find("joint"))
    joints = {j.get("name") for j in hand.iter("joint")}
    bodies = {b.get("name") for b in hand.iter("body")}
    for tag in ("actuator", "equality", "contact"):
        parent = root.find(tag)
        for item in list(parent):
            keep = (
                item.get("joint") in joints
                if tag == "actuator"
                else item.get("joint1") in joints and item.get("joint2") in joints
                if tag == "equality"
                else item.get("body1") in bodies and item.get("body2") in bodies
            )
            if not keep:
                parent.remove(item)
    for geom in hand.iter("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
    tip = hand.find(f".//body[@name='{prefix}{finger}_distal']")
    for geom in tip.findall("geom"):
        if "collision" in geom.get("class", ""):
            geom.set("contype", "1")
            geom.set("conaffinity", "2")
            geom.set("friction", "0 0 0")
            geom.set("condim", "1")
    normal = np.array(ref["normal"])
    tangent = np.cross(
        normal, [0.0, 0.0, 1.0] if abs(normal[2]) < 0.9 else [0.0, 1.0, 0.0]
    )
    tangent /= np.linalg.norm(tangent)
    rotation = np.column_stack([tangent, np.cross(normal, tangent), normal])
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.reshape(-1))
    ET.SubElement(
        root.find("worldbody"),
        "geom",
        name="load_cell",
        type="plane",
        size=".05 .05 .001",
        pos=" ".join(map(str, ref["point_m"])),
        quat=" ".join(map(str, quat)),
        contype="2",
        conaffinity="1",
        condim="1",
        friction="0 0 0",
        solref=O6_CONTACT_SOLREF,
        solimp=O6_CONTACT_SOLIMP,
    )
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    pose = {
        name.removeprefix(f"hand_{side}_"): value
        for name, value in ref["joints"].items()
    }
    return model, pose, hashlib.sha256(xml.encode()).hexdigest()


def probe_contact(model, pose, *, duration=30.0, recontact=False):
    data = mujoco.MjData(model)
    for j in range(model.njnt):
        data.qpos[model.jnt_qposadr[j]] = pose[model.joint(j).name]
    child, parent = model.eq_obj1id[0], model.eq_obj2id[0]
    c, p = model.jnt_qposadr[[child, parent]]
    ratio = model.eq_data[0, 1]
    initial = max(0.015, data.qpos[p] - 0.12)
    goal = min(
        model.jnt_range[parent, 1] - 0.01,
        model.jnt_range[child, 1] / ratio - 0.01,
        data.qpos[p] + 0.25,
    )
    data.qpos[p], data.qpos[c] = initial, ratio * initial
    plate = model.geom("load_cell").id
    trace = []
    wrench = np.zeros(6)
    dt = model.opt.timestep
    for _ in range(round(duration / dt)):
        u = np.clip((data.time - 0.5) / 2, 0.0, 1.0)
        scale = u**3 * (10 + u * (-15 + 6 * u))
        if recontact and 8 <= data.time < 14:
            scale = max(0.0, 1 - (data.time - 8) / 2)
        elif recontact and data.time >= 14:
            scale = min(1.0, (data.time - 14) / 2)
        data.ctrl[:] = initial + (goal - initial) * scale
        mujoco.mj_step(model, data)
        if data.warning.number.any() or not np.isfinite(data.qpos).all():
            break
        force = penetration = 0.0
        for ci, contact in enumerate(data.contact):
            if plate in contact.geom:
                mujoco.mj_contactForce(model, data, ci, wrench)
                force += max(0.0, wrench[0])
                penetration = max(penetration, -contact.dist)
        trace.append(
            [
                data.time,
                force,
                abs(data.qpos[c] - ratio * data.qpos[p]),
                abs(data.qvel).max(),
                penetration,
                data.qpos[p],
                data.qpos[c],
            ]
        )
    trace = np.asarray(trace)
    tail = trace[trace[:, 0] >= duration - 5]
    completed = len(trace) == round(duration / dt) and bool(np.isfinite(trace).all())
    result = {
        "completed": completed,
        "recontact": recontact,
        "time": float(data.time),
        "timestep": dt,
        "warnings": data.warning.number.tolist(),
    }
    if len(tail):
        result.update(
            force_mean=float(tail[:, 1].mean()),
            force_std=float(tail[:, 1].std()),
            coupling_error=float(tail[:, 2].max()),
            max_velocity=float(tail[:, 3].max()),
            penetration_mean=float(tail[:, 4].mean()),
            angle_ptp=float(np.ptp(tail[:, 5:], axis=0).max()),
            contact_fraction=float(np.mean(tail[:, 1] > 0)),
        )
    result["passed"] = bool(
        completed
        and not any(result["warnings"])
        and len(tail)
        and result["force_mean"] > 1.0
        and result["force_std"] < 0.1
        and result["max_velocity"] < 0.01
        and result["coupling_error"] < 0.001
        and result["contact_fraction"] == 1.0
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--recontact", action="store_true")
    args = parser.parse_args()
    rows = []
    for side in ("left", "right"):
        for finger in ("thumb", "index"):
            model, pose, digest = build_fixture(side, finger)
            r = dict(
                side=side,
                finger=finger,
                fixture_sha256=digest,
                **probe_contact(model, pose, recontact=args.recontact),
            )
            rows.append(r)
            print(side, finger, r)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps({"mujoco": mujoco.__version__, "rows": rows}, indent=2) + "\n"
    )
    return 0 if all(r["passed"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
