"""Transfer O6 numerical settings; retain each other hand's physics/URDF caps."""

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import mujoco
import numpy as np

from linker_robot_assets import asset_root
from linker_robot_assets.composer.o6_profile import configure_o6_solver
from linker_robot_assets.composer.schemas import ComponentMeta

OUT = Path.cwd() / "hand_2ms_results"
HANDS = ("l6", "l20lite", "l25", "l30")
mujoco.set_mju_user_warning(lambda message: None)


def build(hand, side, mode):
    comp = asset_root() / "components/hands" / f"linkerhand_{hand}"
    meta = ComponentMeta.load(comp / "meta.yaml")
    variant = meta.variants[side]
    source = comp / variant.mjcf
    root = ET.parse(source).getroot()
    root.find("compiler").set("meshdir", str(comp / variant.meshdir))
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", ".002")
    configure_o6_solver(option)
    for eq in root.findall("equality/joint"):
        eq.set("solref", "-1e7 -63000")
        eq.set("solimp", "0.9999 0.9999 0.001 0.5 2")
    for joint in root.findall(".//worldbody//joint"):
        joint.set("solreflimit", ".000125 1")
        joint.set("solimplimit", "0.9999 0.9999 0.001 0.5 2")
    for geom in root.findall(".//worldbody//geom"):
        geom.set("solref", ".004 1")
        geom.set("solimp", ".95 .95 .001 .5 2")
        geom.set("margin", "0")
        geom.set("gap", "0")
    urdf = ET.parse(comp / variant.urdf)
    caps = {
        j.get("name"): float(j.find("limit").get("effort"))
        for j in urdf.findall("joint")
        if j.find("limit") is not None
    }
    for act in root.findall("actuator/position"):
        cap = caps[act.get("joint")]
        act.set("kp", "600")
        act.set("kv", "4.5")
        # Match the existing composer: nonpositive URDF effort is not a
        # usable finite force range. Leave that authored actuator uncapped.
        if cap > 0:
            act.set("forcelimited", "true")
            act.set("forcerange", f"{-cap} {cap}")
    if mode == "payload":
        body = root.find(f".//body[@name='{side[0]}h_index_distal']")
        payload = ET.SubElement(body, "body", name="test_payload", pos=".006 0 .035")
        ET.SubElement(
            payload, "inertial", pos="0 0 0", mass=".05", diaginertia="5e-7 5e-7 5e-7"
        )
    if mode.startswith("contact-"):
        finger = mode.removeprefix("contact-")
        eqs = [
            e
            for e in root.findall("equality/joint")
            if f"_{finger}_" in e.get("joint1")
        ]
        kept = {e.get(k) for e in eqs for k in ("joint1", "joint2")}
        # Fixture all unrelated joints at zero, analogous to fixing O6 thumb yaw.
        for body in root.findall(".//worldbody//body"):
            for joint in list(body.findall("joint")):
                if joint.get("name") not in kept:
                    body.remove(joint)
            for geom in body.findall("geom"):
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
        for eq in list(root.find("equality")):
            if eq not in eqs:
                root.find("equality").remove(eq)
        for act in list(root.find("actuator")):
            if act.get("joint") not in kept:
                root.find("actuator").remove(act)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        data = mujoco.MjData(model)
        parents = np.unique(model.eq_obj2id)
        assert len(parents) == 1
        parent = int(parents[0])
        upper = min(
            [model.jnt_range[parent, 1]]
            + [
                model.jnt_range[c, 1] / r
                for c, r in zip(model.eq_obj1id, model.eq_data[:, 1])
            ]
        )
        angle = min(0.4, 0.4 * upper)
        data.qpos[model.jnt_qposadr[parent]] = angle
        data.qpos[model.jnt_qposadr[model.eq_obj1id]] = model.eq_data[:, 1] * angle
        mujoco.mj_forward(model, data)

        def depth(j):
            b = int(model.jnt_bodyid[j])
            n = 0
            while b:
                n += 1
                b = int(model.body_parentid[b])
            return n

        child = int(max(model.eq_obj1id, key=depth))
        body_id = int(model.jnt_bodyid[child])
        body = root.find(f".//body[@name='{model.body(body_id).name}']")
        collision = [
            g for g in body.findall("geom") if "collision" in g.get("class", "")
        ]
        if not collision:
            collision = [g for g in body.findall("geom") if "col" in g.get("mesh", "")]
        assert collision, (hand, side, finger)
        mesh_names = {g.get("mesh") for g in collision}
        ids = [
            i
            for i in range(model.ngeom)
            if model.geom_bodyid[i] == body_id
            and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
            and model.mesh(int(model.geom_dataid[i])).name in mesh_names
        ]
        vertices = []
        for gi in ids:
            mesh = int(model.geom_dataid[gi])
            adr = model.mesh_vertadr[mesh]
            count = model.mesh_vertnum[mesh]
            v = (
                model.mesh_vert[adr : adr + count] @ data.geom_xmat[gi].reshape(3, 3).T
                + data.geom_xpos[gi]
            )
            vertices.append(v)
        vertices = np.concatenate(vertices)
        point = vertices.mean(axis=0)
        jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacp, jacr, point, body_id)
        velocity = jacp[:, model.jnt_dofadr[parent]].copy()
        for c, ratio in zip(model.eq_obj1id, model.eq_data[:, 1]):
            velocity += ratio * jacp[:, model.jnt_dofadr[c]]
        normal = -velocity / np.linalg.norm(velocity)
        point += normal * (np.min(vertices @ normal) - point @ normal)
        tangent = np.cross(
            normal, [0.0, 0.0, 1.0] if abs(normal[2]) < 0.9 else [0.0, 1.0, 0.0]
        )
        tangent /= np.linalg.norm(tangent)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(
            quat,
            np.column_stack([tangent, np.cross(normal, tangent), normal]).reshape(-1),
        )
        for geom in collision:
            geom.set("contype", "1")
            geom.set("conaffinity", "2")
            geom.set("friction", "0 0 0")
            geom.set("condim", "1")
        ET.SubElement(
            root.find("worldbody"),
            "geom",
            name="test_plate",
            type="plane",
            size=".1 .1 .001",
            pos=" ".join(map(str, point)),
            quat=" ".join(map(str, quat)),
            contype="2",
            conaffinity="1",
            friction="0 0 0",
            condim="1",
            solref=".004 1",
            solimp=".95 .95 .001 .5 2",
        )
    else:
        angle = None
    folder = OUT / "models" / mujoco.__version__
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{hand}-{side}-{mode}.xml"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    m = mujoco.MjModel.from_xml_path(str(path))
    assert m.opt.timestep == 0.002
    np.testing.assert_array_equal(m.eq_solref, np.tile([-1e7, -63000], (m.neq, 1)))
    np.testing.assert_array_equal(m.actuator_gainprm[:, 0], 600.0)
    np.testing.assert_array_equal(m.actuator_biasprm[:, 2], -4.5)
    np.testing.assert_array_equal(m.eq_active0, True)
    if not mode.startswith("contact-"):
        before = mujoco.MjModel.from_xml_path(str(source))
        for field in (
            "dof_damping",
            "dof_frictionloss",
            "dof_armature",
            "eq_data",
            "jnt_range",
            "actuator_trnid",
            "geom_contype",
            "geom_conaffinity",
        ):
            np.testing.assert_array_equal(getattr(m, field), getattr(before, field))

        def exclusions(model):
            return {
                tuple(
                    sorted(
                        (
                            model.body(int(sig) // 65536).name,
                            model.body(int(sig) % 65536).name,
                        )
                    )
                )
                for sig in model.exclude_signature
            }

        assert exclusions(m) == exclusions(before)
        if mode != "payload":
            for field in ("body_mass", "body_inertia", "body_ipos"):
                np.testing.assert_array_equal(getattr(m, field), getattr(before, field))
    return m, path, angle


def trial(hand, side, mode, duration=30):
    m, path, reference_angle = build(hand, side, mode)
    d = mujoco.MjData(m)
    parents, children = m.jnt_qposadr[m.eq_obj2id], m.jnt_qposadr[m.eq_obj1id]
    goals = np.zeros(m.nq)
    for parent in np.unique(m.eq_obj2id):
        upper = min(
            [m.jnt_range[parent, 1]]
            + [
                m.jnt_range[m.eq_obj1id[e], 1] / m.eq_data[e, 1]
                for e in np.flatnonzero(m.eq_obj2id == parent)
            ]
        )
        goals[m.jnt_qposadr[parent]] = (
            0.98 * upper if mode == "limit" else min(0.6, 0.5 * upper)
        )
    commanded = m.jnt_qposadr[m.actuator_trnid[:, 0]]
    contact = mode.startswith("contact-")
    if contact:
        initial = max(0.01, reference_angle - 0.15)
        d.qpos[parents] = initial
        d.qpos[children] = m.eq_data[:, 1] * initial
        goals[parents] = reference_angle + 0.2
        plate = m.geom("test_plate").id
    mujoco.mj_forward(m, d)
    trace = []
    peak = np.zeros(len(parents))
    wrench = np.zeros(6)
    for i in range(round(duration / 0.002)):
        t = d.time
        if contact:
            u = min(1.0, max(0.0, (t - 0.5) / 2))
            d.ctrl[:] = initial + (goals[commanded] - initial) * (
                u**3 * (10 + u * (-15 + 6 * u))
            )
        else:
            scale = min(t, 1.0)
            if mode == "cycle" and t >= 10:
                scale = max(0.0, 1 - (t - 10) / 2)
            d.ctrl[:] = goals[commanded] * scale
        mujoco.mj_step(m, d)
        if d.warning.number.any() or not np.isfinite(d.qpos).all():
            break
        force = penetration = 0.0
        if contact:
            for ci, c in enumerate(d.contact):
                if plate in c.geom:
                    mujoco.mj_contactForce(m, d, ci, wrench)
                    force += max(0.0, wrench[0])
                    penetration = max(penetration, -c.dist)
        error = abs(d.qpos[children] - m.eq_data[:, 1] * d.qpos[parents]).max()
        peak = np.maximum(peak, d.qpos[parents])
        trace.append([d.time, error, abs(d.qvel).max(), force, penetration, *d.qpos])
    a = np.asarray(trace)
    tail = a[a[:, 0] >= duration - 5]
    r = {
        "hand": hand,
        "side": side,
        "mode": mode,
        "model": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "completed": bool(len(a) == round(duration / 0.002) and np.isfinite(a).all()),
        "warnings": d.warning.number.tolist(),
        "timestep": 0.002,
        "cap": m.actuator_forcerange.tolist(),
        "neq": m.neq,
        "moved": bool(np.all(peak > 0.1)),
        "parent_motion_from_initial": float(peak.min() - (initial if contact else 0)),
    }
    r["uncapped_actuators"] = [
        m.actuator(i).name for i in range(m.nu) if not m.actuator_forcelimited[i]
    ]
    r["time_s"] = float(d.time)
    if len(tail):
        r.update(
            coupling_error=float(tail[:, 1].max()),
            max_velocity=float(tail[:, 2].max()),
            angle_ptp=float(np.ptp(tail[:, 5:], axis=0).max()),
            force_mean=float(tail[:, 3].mean()),
            force_std=float(tail[:, 3].std()),
            penetration=float(tail[:, 4].mean()),
        )
        r["steady"] = bool(
            r["completed"] and r["max_velocity"] < 0.01 and r["coupling_error"] < 0.001
        )
        if contact:
            r["steady"] &= bool(r["force_mean"] > 1 and r["force_std"] < 0.1)
        else:
            r["steady"] &= r["moved"]
    else:
        r["steady"] = False
    folder = OUT / "traces" / mujoco.__version__
    folder.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(folder / f"{hand}-{side}-{mode}.npz", trace=a)
    return r


def task(args):
    global OUT
    case, output_dir = args
    OUT = Path(output_dir)
    return trial(*case)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--kind", choices=("motion", "contact"), required=True)
    p.add_argument("--hand", action="append", choices=HANDS)
    p.add_argument("--output-dir", type=Path, default=OUT)
    args = p.parse_args()
    OUT = args.output_dir.resolve()
    OUT.mkdir(parents=True, exist_ok=True)
    cases = []
    for h in args.hand or HANDS:
        for s in ("left", "right"):
            modes = (
                ("hold", "cycle", "limit", "payload")
                if args.kind == "motion"
                else tuple(
                    f"contact-{f}"
                    for f in (
                        ("index", "middle", "ring", "pinky")
                        if h == "l30"
                        else ("thumb", "index", "middle", "ring", "pinky")
                    )
                )
            )
            cases.extend((h, s, mode) for mode in modes)
    rows = []
    with ProcessPoolExecutor(4) as pool:
        for future in as_completed([pool.submit(task, (c, str(OUT))) for c in cases]):
            r = future.result()
            rows.append(r)
            label = f"{len(rows)}/{len(cases)} {r['hand']}/{r['side']}/{r['mode']}"
            if r["completed"]:
                print(
                    f"{label}: steady={r['steady']}, speed={r['max_velocity']:.6g}, "
                    f"force={r['force_mean']:.6g}, std={r['force_std']:.6g}, "
                    f"eq={r['coupling_error']:.6g}",
                    flush=True,
                )
            else:
                print(f"{label}: aborted, warnings={r['warnings']}", flush=True)
            (OUT / f"{args.kind}-{mujoco.__version__}.json").write_text(
                json.dumps(rows, indent=2) + "\n"
            )

    raise SystemExit(0 if all(r["steady"] for r in rows) else 1)
