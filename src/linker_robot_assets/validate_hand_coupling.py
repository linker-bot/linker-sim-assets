"""Audit every shipped hand's passive following using only ctrl and mj_step.

    python -m linker_robot_assets.validate_hand_coupling --output results.json
    python -m linker_robot_assets.validate_hand_coupling --hand o6 --cycle

Default: a one-second ramp, then hold, for five simulated seconds. --cycle
closes over one second, holds, opens over one second, and holds (four seconds).
O6 is checked over eight seconds against final-second stability criteria.
This audits the default component files; it is not hardware calibration.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from . import asset_root
from .composer.schemas import ComponentMeta


def coupling_targets(model: mujoco.MjModel) -> np.ndarray:
    """Move equality parents within both their own and their children's limits.

    The current hands have scalar joints, zero reference angles, and positive
    linear ratios. Keep unrelated axes at zero (including L30's active thumb).
    Never force a parent to an angle that contradicts a distal joint limit.
    """
    if (
        model.nq != model.njnt
        or model.neq == 0
        or np.any(model.qpos0)
        or np.any(model.eq_type != mujoco.mjtEq.mjEQ_JOINT)
        or np.any(model.eq_obj2id < 0)
        or np.any(model.eq_data[:, 0])
        or np.any(model.eq_data[:, 2:5])
        or np.any(model.eq_data[:, 1] <= 0)
    ):
        raise ValueError(
            "Audit requires scalar, zero-reference, positive linear hand couplings"
        )
    goals = np.zeros(model.nq)
    for parent in np.unique(model.eq_obj2id):
        upper = model.jnt_range[parent, 1]
        for e in np.flatnonzero(model.eq_obj2id == parent):
            child = model.eq_obj1id[e]
            upper = min(upper, model.jnt_range[child, 1] / model.eq_data[e, 1])
        goals[model.jnt_qposadr[parent]] = min(0.6, 0.5 * upper)
    return goals[model.jnt_qposadr[model.actuator_trnid[:, 0]]]


def probe(
    model: mujoco.MjModel,
    *,
    cycle: bool = False,
    motor_gains=None,
    duration=None,
    steady=False,
) -> dict:
    """Check every integration step; stop on warnings or automatic reset.

    motor_gains=(kp, kv) is only for composed torque motors, to reproduce the
    original component servos. It applies effort through ctrl, never qfrc or
    an independent passive-joint actuator.
    """
    data = mujoco.MjData(model)
    targets = coupling_targets(model)
    if np.any(np.isin(model.eq_obj1id, model.actuator_trnid[:, 0])):
        raise ValueError("Passive joints must not have independent actuators")
    if motor_gains is None and not np.all(model.actuator_biasprm[:, 1] < 0):
        raise ValueError("Position actuators required, or supply motor_gains")
    parent = model.jnt_qposadr[model.eq_obj2id]
    child = model.jnt_qposadr[model.eq_obj1id]
    commanded = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    dofs = model.jnt_dofadr[model.actuator_trnid[:, 0]]
    peak_error = 0.0
    peak_limit = 0.0
    contacts = 0
    peak_parent = np.zeros(model.neq)
    duration = duration if duration is not None else (4.0 if cycle else 5.0)
    steps = round(duration / model.opt.timestep)
    error = np.zeros(model.neq)
    completed = True
    tail_error = tail_speed = tail_limit = 0.0
    for _ in range(steps):
        scale = min(data.time, 1.0)
        if cycle and data.time >= 2:
            scale = max(3.0 - data.time, 0.0)
        target = targets * scale
        if motor_gains is None:
            data.ctrl[:] = target
        else:
            kp, kv = motor_gains
            data.ctrl[:] = kp * (target - data.qpos[commanded]) - kv * data.qvel[dofs]
        mujoco.mj_step(model, data)
        if data.warning.number.any() or not np.isfinite(data.qpos).all():
            completed = False
            break
        error = data.qpos[child] - model.eq_data[:, 1] * data.qpos[parent]
        peak_error = max(peak_error, float(abs(error).max()))
        peak_parent = np.maximum(peak_parent, data.qpos[parent])
        peak_limit = max(
            peak_limit,
            float(
                np.maximum(
                    model.jnt_range[:, 0] - data.qpos, data.qpos - model.jnt_range[:, 1]
                ).max()
            ),
        )
        contacts = max(contacts, data.ncon)
        if data.time >= duration - 1:
            tail_error = max(tail_error, float(abs(error).max()))
            tail_speed = max(tail_speed, float(abs(data.qvel).max()))
            tail_limit = max(
                tail_limit,
                float(
                    np.maximum(
                        model.jnt_range[:, 0] - data.qpos,
                        data.qpos - model.jnt_range[:, 1],
                    ).max()
                ),
            )
    # Do not pass a stationary model just because both sides remain at zero.
    moved = bool(np.all(peak_parent > 0.1))
    active = bool(np.all(data.eq_active))
    passed = (
        completed and moved and active and peak_error < 0.005 and peak_limit < 0.005
    )
    if steady:
        passed = (
            completed
            and moved
            and active
            and tail_error < 0.001
            and tail_speed < 0.01
            and tail_limit < 0.005
        )
    return {
        "passed": passed,
        "completed": completed,
        "time": float(data.time),
        "target_rad": targets.tolist(),
        "timestep": float(model.opt.timestep),
        "active_joint_motion": moved,
        "eq_active": data.eq_active.tolist(),
        "max_error_rad": peak_error,
        "final_error_rad": float(abs(error).max()),
        "max_joint_limit_violation_rad": peak_limit,
        "max_contacts": contacts,
        "warnings": data.warning.number.tolist(),
        "criterion": "final-second" if steady else "full-trajectory",
        "tail_error_rad": tail_error,
        "tail_velocity_rad_s": tail_speed,
        "pairs": [
            {
                "passive": model.joint(int(model.eq_obj1id[e])).name,
                "active": model.joint(int(model.eq_obj2id[e])).name,
                "ratio": float(model.eq_data[e, 1]),
                "active_rad": float(data.qpos[parent[e]]),
                "passive_rad": float(data.qpos[child[e]]),
                "error_rad": float(error[e]),
            }
            for e in range(model.neq)
        ],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets-root", type=Path, default=asset_root())
    parser.add_argument(
        "--hand", choices=("l6", "o6", "l20lite", "l25", "l30"), action="append"
    )
    parser.add_argument("--cycle", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    rows = []
    for hand in args.hand or ("l6", "o6", "l20lite", "l25", "l30"):
        component = args.assets_root / "components/hands" / f"linkerhand_{hand}"
        meta = ComponentMeta.load(component / "meta.yaml")
        for side, variant in meta.variants.items():
            path = component / variant.mjcf
            o6 = hand == "o6"
            result = probe(
                mujoco.MjModel.from_xml_path(str(path)),
                cycle=args.cycle,
                duration=8 if o6 else None,
                steady=o6,
            )
            row = dict(
                hand=hand,
                side=side,
                model=str(path.relative_to(args.assets_root)),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                **result,
            )
            rows.append(row)
            print(
                f"{hand}/{side}: max error={result['max_error_rad']} rad, "
                f"final={result['final_error_rad']} rad; "
                f"{'PASS' if result['passed'] else 'FAIL'}",
                flush=True,
            )
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(
                {"mujoco": mujoco.__version__, "cycle": args.cycle, "results": rows},
                indent=2,
            )
            + "\n"
        )
    return 0 if all(row["passed"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
