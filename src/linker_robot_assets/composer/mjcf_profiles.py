"""Opt-in MuJoCo numerical profiles, applied before XML serialization.

These are solver settings, not identified hardware parameters. In particular,
O6 coupling does not introduce distal motors or project joint coordinates.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Sequence

from .determinism import fmt_float
from .schemas import MujocoOptions, SchemaError

O6_TIMESTEP = 0.0000625
O6_SOLREF = "0.000125 1"
O6_SOLIMP = "0.9999 0.9999 0.001 0.5 2"


def apply_mujoco_options(
    root: ET.Element,
    settings: MujocoOptions,
    o6_mimic_groups: Sequence[Sequence[str]] = (),
) -> None:
    """Apply a scene timestep and, optionally, all declared O6 couplings.

    Each group is the five prefixed passive joint names of one O6 component.
    Other components' equalities and all physical/actuator parameters survive.
    A larger timestep is rejected instead of silently weakening the selected
    profile through MuJoCo's refsafe time-constant clamp.
    """
    timestep = settings.timestep
    equalities: list[ET.Element] = []
    if settings.coupling_profile is not None:
        if settings.coupling_profile != "o6_coupled":
            raise SchemaError(f"Unknown coupling profile: {settings.coupling_profile}")
        if not o6_mimic_groups:
            raise SchemaError(
                "o6_coupled requires at least one linkerhand_o6 component"
            )
        timestep = O6_TIMESTEP if timestep is None else timestep
        if timestep > O6_TIMESTEP:
            raise SchemaError(
                f"o6_coupled requires timestep <= {O6_TIMESTEP}; got {timestep}"
            )
        for group in o6_mimic_groups:
            joints = set(group)
            matches = [
                e for e in root.findall("equality/joint") if e.get("joint1") in joints
            ]
            if (
                len(joints) != 5
                or len(matches) != 5
                or {e.get("joint1") for e in matches} != joints
                or any(not e.get("joint2") for e in matches)
            ):
                raise SchemaError(
                    "o6_coupled requires exactly five two-joint O6 equalities per hand"
                )
            equalities.extend(matches)

    if timestep is not None:
        option = root.find("option")
        if option is None:
            option = ET.SubElement(root, "option")
        option.set("timestep", fmt_float(timestep))
    for equality in equalities:
        equality.set("solref", O6_SOLREF)
        equality.set("solimp", O6_SOLIMP)
