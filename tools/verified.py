"""The physics of a verified assembly, frozen: a summary of its compiled model in the world frame.

    python tools/verified.py lw_v0     # write tests/verified/lw_v0.json

`tests/test_assets.py` compares each assembly with its file on every push, so a component change
that alters a verified assembly fails until it is accepted: rewrite the file with this tool, in
a change that is a new major version (or make a new assembly; README "Versions"). Materials and
the names of meshes, materials and classes are left out: a colour may be corrected in 1.x.

The summary, at the home keyframe, by name: bodies (pose, mass, centre of mass, inertia tensor,
gravity compensation), joints (anchor, axis, limits, armature, damping, friction, stiffness),
drives, couplings, contact exclusions, collision geoms (pose, shape, contact settings; a mesh by
the centroid, second moments and extent of its vertices), visual geoms (their meshes the same
way), sites, cameras, solver options; and the controller data next to the parts (each part
folder's `.yaml` files, read).
"""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
OPTIONS = ("timestep", "impratio", "tolerance", "ls_tolerance", "noslip_tolerance", "gravity", "wind", "magnetic",
           "density", "viscosity", "o_margin", "o_solref", "o_solimp", "o_friction", "integrator", "cone", "jacobian",
           "solver", "iterations", "ls_iterations", "noslip_iterations", "disableflags", "enableflags")


def _l(x):
    return np.asarray(x, dtype=float).tolist()


def _name(m, objtype, i):
    return mujoco.mj_id2name(m, objtype, i) if i >= 0 else None


def _pose(pos, mat):
    return {"pos": _l(pos), "mat": _l(np.asarray(mat).reshape(3, 3))}


def _mesh(m, d, g):
    """A mesh geom by its vertices in the world frame: count, centroid, second moments, extent."""
    i = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[i]:m.mesh_vertadr[i] + m.mesh_vertnum[i]].astype(float)
    w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
    c = w.mean(axis=0)
    return {"vertices": int(len(w)), "centroid": _l(c), "moments": _l((w - c).T @ (w - c) / len(w)),
            "min": _l(w.min(axis=0)), "max": _l(w.max(axis=0))}


def controller_data(path):
    """The `.yaml` files in the folders of the parts the assembly at `path` uses, read."""
    folder = Path(path).parent
    files = {(folder / model.get("file")).resolve()
             for model in ET.parse(path).getroot().iter("model")}
    if (folder / "assembly.yaml").exists():
        for unit in yaml.safe_load((folder / "assembly.yaml").read_text())["units"].values():
            files |= {(folder / part["file"]).resolve() for part in unit["parts"].values()}
    data = {}
    for part_folder in sorted({f.parent for f in files} - {folder.resolve()}):
        for f in sorted(part_folder.glob("*.yaml")):
            data[f.relative_to(ROOT).as_posix()] = yaml.safe_load(f.read_text())
    return data


def summary(path):
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("home").id)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    B, J, G = mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_GEOM

    out = {"options": {k: _l(getattr(m.opt, k)) for k in OPTIONS},
           "sizes": {"nconmax": int(m.nconmax), "njmax": int(m.njmax)},
           "home_qpos": _l(d.qpos)}

    out["bodies"] = {}
    for b in range(1, m.nbody):
        r = d.ximat[b].reshape(3, 3)
        out["bodies"][m.body(b).name] = {
            "parent": _name(m, B, m.body_parentid[b]), **_pose(d.xpos[b], d.xmat[b]),
            "mass": float(m.body_mass[b]), "com": _l(d.xipos[b]),
            "inertia": _l(r @ np.diag(m.body_inertia[b]) @ r.T), "gravcomp": float(m.body_gravcomp[b])}

    out["joints"] = {}
    for j in range(m.njnt):
        v = m.jnt_dofadr[j]
        out["joints"][m.joint(j).name] = {
            "type": int(m.jnt_type[j]), "body": _name(m, B, m.jnt_bodyid[j]), "anchor": _l(d.xanchor[j]),
            "axis": _l(d.xaxis[j]), "limited": int(m.jnt_limited[j]), "range": _l(m.jnt_range[j]),
            "actfrclimited": int(m.jnt_actfrclimited[j]), "actfrcrange": _l(m.jnt_actfrcrange[j]),
            "solref": _l(m.jnt_solref[j]), "solimp": _l(m.jnt_solimp[j]), "margin": float(m.jnt_margin[j]),
            "stiffness": float(m.jnt_stiffness[j]), "springref": float(m.qpos_spring[m.jnt_qposadr[j]]),
            "armature": float(m.dof_armature[v]), "damping": float(m.dof_damping[v]),
            "frictionloss": float(m.dof_frictionloss[v])}

    out["actuators"] = {}
    for u in range(m.nu):
        out["actuators"][m.actuator(u).name] = {
            "joint": _name(m, J, m.actuator_trnid[u, 0]), "trntype": int(m.actuator_trntype[u]),
            "dyntype": int(m.actuator_dyntype[u]), "gaintype": int(m.actuator_gaintype[u]),
            "biastype": int(m.actuator_biastype[u]), "gainprm": _l(m.actuator_gainprm[u][:3]),
            "biasprm": _l(m.actuator_biasprm[u][:3]), "gear": _l(m.actuator_gear[u]),
            "ctrllimited": int(m.actuator_ctrllimited[u]), "ctrlrange": _l(m.actuator_ctrlrange[u]),
            "forcelimited": int(m.actuator_forcelimited[u]), "forcerange": _l(m.actuator_forcerange[u])}

    out["couplings"] = {}
    for e in range(m.neq):
        out["couplings"][m.equality(e).name] = {
            "type": int(m.eq_type[e]), "obj1": _name(m, J, m.eq_obj1id[e]), "obj2": _name(m, J, m.eq_obj2id[e]),
            "data": _l(m.eq_data[e][:5]), "solref": _l(m.eq_solref[e]), "solimp": _l(m.eq_solimp[e]),
            "active": int(m.eq_active0[e])}

    out["exclusions"] = sorted(sorted((_name(m, B, s >> 16), _name(m, B, s & 0xFFFF)))
                               for s in m.exclude_signature.tolist())

    out["collision_geoms"] = {}
    for b in range(1, m.nbody):
        geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and (m.geom_contype[g] or m.geom_conaffinity[g])]
        for k, g in enumerate(geoms):
            name = _name(m, G, g) or f"{m.body(b).name}#{k}"
            entry = {"body": m.body(b).name, "type": int(m.geom_type[g]),
                     "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]),
                     "condim": int(m.geom_condim[g]), "priority": int(m.geom_priority[g]),
                     "friction": _l(m.geom_friction[g]), "solmix": float(m.geom_solmix[g]),
                     "solref": _l(m.geom_solref[g]), "solimp": _l(m.geom_solimp[g]),
                     "margin": float(m.geom_margin[g]), "gap": float(m.geom_gap[g])}
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                entry |= _mesh(m, d, g)
            else:
                entry |= {"size": _l(m.geom_size[g]), **_pose(d.geom_xpos[g], d.geom_xmat[g])}
            out["collision_geoms"][name] = entry

    out["visual_geoms"] = {}
    for b in range(1, m.nbody):
        geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and not (m.geom_contype[g] or m.geom_conaffinity[g])]
        for k, g in enumerate(geoms):
            entry = {"body": m.body(b).name, "type": int(m.geom_type[g]), "group": int(m.geom_group[g])}
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                entry |= _mesh(m, d, g)
            else:
                entry |= {"size": _l(m.geom_size[g]), **_pose(d.geom_xpos[g], d.geom_xmat[g])}
            out["visual_geoms"][f"{m.body(b).name}#{k}"] = entry

    out["controller_data"] = controller_data(path)
    out["sites"] = {m.site(s).name: _pose(d.site_xpos[s], d.site_xmat[s]) for s in range(m.nsite)}
    out["cameras"] = {m.camera(c).name: {**_pose(d.cam_xpos[c], d.cam_xmat[c]), "fovy": float(m.cam_fovy[c]),
                                         "resolution": _l(m.cam_resolution[c]), "intrinsic": _l(m.cam_intrinsic[c]),
                                         "sensorsize": _l(m.cam_sensorsize[c])}
                      for c in range(m.ncam)}
    return out


def differences(a, b, path="", atol=1e-12, rtol=1e-9):
    """Where two summaries differ: numbers beyond the tolerance, anything else exactly."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = [f"{path}/{k}: only in one" for k in sorted(a.keys() ^ b.keys())]
        for k in sorted(a.keys() & b.keys()):
            out += differences(a[k], b[k], f"{path}/{k}", atol, rtol)
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [f"{path}: {len(a)} items != {len(b)}"]
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += differences(x, y, f"{path}[{i}]", atol, rtol)
        return out[:1] if out and not isinstance(a[0], (dict, list)) else out  # one line per vector
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and (isinstance(a, float) or isinstance(b, float)):
        return [] if np.isclose(a, b, atol=atol, rtol=rtol) else [f"{path}: {a} != {b}"]
    return [] if a == b else [f"{path}: {a} != {b}"]


def main():
    name = sys.argv[1]
    out = ROOT / "tests" / "verified" / f"{name}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(summary(ROOT / "assemblies" / name / f"{name}.xml"), indent=1) + "\n")
    print("wrote", out.relative_to(ROOT))


if __name__ == "__main__":
    main()
