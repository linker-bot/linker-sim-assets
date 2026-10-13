"""Opt-in MuJoCo numerical profiles, applied before XML serialization.

These are solver settings, not identified hardware parameters. In particular,
Hand coupling does not introduce distal motors or project joint coordinates.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Sequence

from .determinism import fmt_float
from .o6_profile import (
    O6_SOLREF,
    O6_TIMESTEP,
    configure_o6_hand,
    configure_o6_solver,
)
from .schemas import MujocoOptions, SchemaError

COUPLED_HANDS = ("l6", "o6", "l20lite", "l25", "l30")
COUPLED_COMPONENTS = frozenset(f"linkerhand_{name}" for name in COUPLED_HANDS)
HAND_TIMESTEP = 0.0000625
HAND_SOLREF = "0.000125 1"
HAND_SOLIMP = "0.9999 0.9999 0.001 0.5 2"


def apply_mujoco_options(
    root: ET.Element,
    settings: MujocoOptions,
    mimic_groups: Sequence[Sequence[str]] = (),
) -> None:
    """Apply a scene timestep and the caller's selected hand couplings.

    Each group contains one selected component's declared passive joint names.
    The composer selects only O6 for o6_coupled, or all five tested hand types
    for hand_coupled. Counts are validated against metadata, not assumed to be
    five: L20 Lite has ten passive joints, and L30 has four (its thumb is active).
    o6_coupled uses native position servos at 2 ms; hand_coupled retains the
    earlier 62.5 us equality-only profile. Other components stay scoped out.
    """
    timestep = settings.timestep
    equalities: list[ET.Element] = []
    profile = settings.coupling_profile
    if profile is not None:
        if profile not in ("o6_coupled", "hand_coupled"):
            raise SchemaError(f"Unknown coupling profile: {profile}")
        if not mimic_groups:
            raise SchemaError(
                "o6_coupled requires at least one linkerhand_o6 component"
                if profile == "o6_coupled"
                else "hand_coupled requires at least one supported hand component"
            )
        maximum = O6_TIMESTEP if profile == "o6_coupled" else HAND_TIMESTEP
        timestep = maximum if timestep is None else timestep
        if timestep > maximum:
            raise SchemaError(
                f"{profile} requires timestep <= {maximum}; got {timestep}"
            )
        for group in mimic_groups:
            joints = set(group)
            matches = [
                e for e in root.findall("equality/joint") if e.get("joint1") in joints
            ]
            if (
                not joints
                or len(joints) != len(group)
                or (profile == "o6_coupled" and len(joints) != 5)
                or len(matches) != len(joints)
                or {e.get("joint1") for e in matches} != joints
                or any(not e.get("joint2") for e in matches)
            ):
                raise SchemaError(
                    f"{profile} requires one two-joint equality per declared passive joint"
                )
            equalities.extend(matches)
            if profile == "o6_coupled":
                configure_o6_hand(root, group)

    if timestep is not None:
        option = root.find("option")
        if option is None:
            option = ET.SubElement(root, "option")
        option.set("timestep", fmt_float(timestep))
        if profile == "o6_coupled":
            configure_o6_solver(option)
    for equality in equalities:
        equality.set("solref", O6_SOLREF if profile == "o6_coupled" else HAND_SOLREF)
        equality.set("solimp", HAND_SOLIMP)
