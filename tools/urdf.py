"""A URDF from an MJCF: the same robot, for simulators and tools that read URDF.

Used by `assemble.py`. From the compiled MJCF, per body: a link with its inertial (on its
principal axes), its visual meshes and its collision shapes; per joint: origin, axis, limits,
effort = the MJCF's force limit (0 for a joint without one), damping and friction; joint
couplings as `mimic`; a body without a joint as a fixed joint. The URDF keeps MuJoCo's world body
as a link `world` when several bodies hang from it (an assembly); otherwise the root body is the
root link (a component, which must sit at its own origin).

URDF wants a velocity limit, which MJCF doesn't carry. It comes from the controller data next to
the component (`controller.yaml` `max_velocity`, `motor.yaml` `speed_limit`); a coupled joint
gets |multiplier| x its driver's.
"""

import os
import re
from pathlib import Path

import mujoco
import numpy as np
import yaml


def fmt(v):
    return " ".join(f"{x:.10g}" if abs(x) > 1e-15 else "0" for x in np.atleast_1d(v))


def pose(pos, quat):
    t = np.eye(4)
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, quat)
    t[:3, :3] = m.reshape(3, 3)
    t[:3, 3] = pos
    return t


def rpy(rot):
    """URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = np.arcsin(np.clip(-rot[2, 0], -1, 1))
    if abs(rot[2, 0]) < 1 - 1e-12:
        roll, yaw = np.arctan2(rot[2, 1], rot[2, 2]), np.arctan2(rot[1, 0], rot[0, 0])
    else:  # gimbal lock: put everything in yaw
        roll, yaw = 0.0, np.arctan2(-rot[0, 1], rot[1, 1])
    return np.array([roll, pitch, yaw])


def origin(t):
    return f'<origin xyz="{fmt(t[:3, 3])}" rpy="{fmt(rpy(t[:3, :3]))}"/>'


def velocity_limits(folder):
    """{joint name pattern: rad/s} from the controller data in a component's folder. `{S}` in a
    name stands for the side letter (L/R or l/r)."""
    out = {}
    controller, motor = Path(folder) / "controller.yaml", Path(folder) / "motor.yaml"
    if controller.exists():
        spec = yaml.safe_load(controller.read_text())
        out.update(zip(spec["joints"], spec["max_velocity"]))
    if motor.exists():
        spec = yaml.safe_load(motor.read_text())
        out.update({shaft: spec["speed_limit"] for shaft in spec["shafts"]})
    return out


def velocity_of(joint, limits):
    """The limit whose pattern names this joint (the name ends with it, after any prefix)."""
    found = [v for pattern, v in limits.items()
             if re.search(r"(^|[/_])" + re.escape(pattern).replace(r"\{S\}", "[LRlr]") + "$", joint)]
    assert len(found) == 1, f"joint {joint}: {len(found)} velocity limits in the controller data"
    return found[0]


def write(xml, robot, limits):
    """The URDF text for the MJCF file `xml`, written to sit next to it."""
    xml = Path(xml)
    m = mujoco.MjModel.from_xml_path(str(xml))
    world = sum(m.body_parentid[b] == 0 for b in range(1, m.nbody)) > 1

    def mesh_file(mid):
        f = m.paths[m.mesh_pathadr[mid]:m.paths.index(b"\0", m.mesh_pathadr[mid])].decode()
        return Path(os.path.relpath(xml.parent / f, xml.parent)).as_posix()

    mimic = {}
    for e in range(m.neq):
        if m.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT:
            c = m.eq_data[e][:5]
            assert np.allclose(c[2:], 0), "only linear couplings map to mimic"
            mimic[m.eq_obj1id[e]] = (m.eq_obj2id[e], c[1], c[0])

    out = ['<?xml version="1.0"?>',
           f"<!-- Written by tools/assemble.py from {xml.name}: the same robot. Edit the MJCF, not this file. -->",
           f'<robot name="{robot}">']
    for b in range(0 if world else 1, m.nbody):
        name = m.body(b).name or "world"
        out.append(f'  <link name="{name}">')
        if b > 0:
            out.append(f'    <inertial>{origin(pose(m.body_ipos[b], m.body_iquat[b]))}<mass value="{m.body_mass[b]:.10g}"/>'
                       f'<inertia ixx="{m.body_inertia[b][0]:.10g}" ixy="0" ixz="0" iyy="{m.body_inertia[b][1]:.10g}" '
                       f'iyz="0" izz="{m.body_inertia[b][2]:.10g}"/></inertial>')
        for g in np.where(m.geom_bodyid == b)[0]:
            t = pose(m.geom_pos[g], m.geom_quat[g])
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                t = t @ np.linalg.inv(pose(m.mesh_pos[mid], m.mesh_quat[mid]))
                shape = f'<mesh filename="{mesh_file(mid)}"/>'
            elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                shape = f'<box size="{fmt(2 * m.geom_size[g])}"/>'
            else:
                raise ValueError(f"geom {m.geom(g).name}: type {m.geom_type[g]} has no URDF shape here")
            kind = "visual" if m.geom_group[g] == 1 else "collision"
            assert kind == "visual" or (m.geom_contype[g] or m.geom_conaffinity[g]), m.geom(g).name
            out.append(f"    <{kind}>{origin(t)}<geometry>{shape}</geometry></{kind}>")
        out.append("  </link>")
        if b == 0:
            continue

        if not world and m.body_parentid[b] == 0:
            assert np.allclose(m.body_pos[b], 0) and np.allclose(m.body_quat[b], [1, 0, 0, 0]), "root must sit at the origin"
            continue
        parent = m.body(m.body_parentid[b]).name or "world"
        t = pose(m.body_pos[b], m.body_quat[b])
        joints = [j for j in range(m.njnt) if m.jnt_bodyid[j] == b]
        if not joints:
            out.append(f'  <joint name="{name}_fixed" type="fixed"><parent link="{parent}"/><child link="{name}"/>{origin(t)}</joint>')
            continue
        (j,) = joints
        assert m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and np.allclose(m.jnt_pos[j], 0)
        jn, d = m.joint(j).name, m.jnt_dofadr[j]
        effort = m.jnt_actfrcrange[j][1] if m.jnt_actfrclimited[j] else 0.0
        if j in mimic:
            velocity = abs(mimic[j][1]) * velocity_of(m.joint(mimic[j][0]).name, limits)
        else:
            velocity = velocity_of(jn, limits)
        lo, hi = m.jnt_range[j]
        line = (f'  <joint name="{jn}" type="revolute"><parent link="{parent}"/><child link="{name}"/>{origin(t)}'
                f'<axis xyz="{fmt(m.jnt_axis[j])}"/>'
                f'<limit lower="{lo:.10g}" upper="{hi:.10g}" effort="{effort:.10g}" velocity="{velocity:.10g}"/>'
                f'<dynamics damping="{m.dof_damping[d]:.10g}" friction="{m.dof_frictionloss[d]:.10g}"/>')
        if j in mimic:
            src, mult, off = mimic[j]
            line += f'<mimic joint="{m.joint(src).name}" multiplier="{mult:.10g}" offset="{off:.10g}"/>'
        out.append(line + "</joint>")
    out.append("</robot>\n")
    return "\n".join(out)
