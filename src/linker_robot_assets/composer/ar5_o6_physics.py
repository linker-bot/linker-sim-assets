"""Assembly-only native physics; the compliant controller stays in linker-sim."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from .schemas import SchemaError
from .urdf_ops import CompiledComponent


def apply_ar5_o6_physics(root: ET.Element, compiled: list[CompiledComponent]) -> None:
    """Apply the accepted AR5/O6 material and passive-dynamics configuration.

    Keep composed actuators as torque motors: adding position servos here would
    compete with the runtime's native output drives. Their caps are simulation
    output limits, not hardware ratings or virtual-shaft advancing torques.
    """
    arms = [c for c in compiled if c.meta.kind == "arm"]
    hands = [c for c in compiled if c.meta.kind == "hand"]
    if (
        not arms
        or not hands
        or any(c.meta.name not in {"ar5", "ar5_08"} for c in arms)
        or any(c.meta.name != "linkerhand_o6" for c in hands)
    ):
        raise SchemaError("mjcf_physics=ar5_o6 requires only AR5 arms and O6 hands")

    option = root.find("option")
    assert option is not None
    option.attrib.update(
        timestep="0.0020833333333333333",
        integrator="implicitfast",
        solver="Newton",
        iterations="100",
        ls_iterations="50",
        cone="elliptic",
        impratio="100",
    )

    # Cancel gravity only for robot subtrees, including wrist attachments and
    # camera/optical bodies. World gravity remains available to task objects.
    for arm in arms:
        body = root.find(f".//body[@name='{arm.root_link}']")
        assert body is not None
        for child in body.iter("body"):
            child.set("gravcomp", "1")

    caps: dict[str, str] = {}
    for component in arms + hands:
        is_arm = component.meta.kind == "arm"
        active = set(component.actuated_joints)
        followers = set(component.mimic_joints)
        for joint in root.findall(".//worldbody//joint"):
            name = joint.get("name", "")
            if name not in active | followers:
                continue
            lateral = name.endswith("_thumb_cmc_yaw")
            joint.attrib.update(
                armature="0.01" if is_arm else "0.001",
                frictionloss="0",
                damping="0"
                if is_arm or lateral
                else "0.4"
                if name in followers
                else "4",
            )
            if name in active:
                caps[name] = "200" if is_arm else "10" if lateral else "60"
        if not is_arm:
            for equality in root.findall("equality/joint"):
                if equality.get("joint1") in followers:
                    equality.attrib.update(
                        solref="0.002 1", solimp="0.999 0.999 0.001 0.5 2"
                    )

    for motor in root.findall("actuator/motor"):
        cap = caps.get(motor.get("joint", ""))
        if cap is not None:
            motor.attrib.update(
                ctrllimited="true",
                ctrlrange=f"-{cap} {cap}",
                forcelimited="true",
                forcerange=f"-{cap} {cap}",
            )

    hand_classes = tuple(f"{c.role}_" for c in hands)
    for geom in root.findall(".//worldbody//geom"):
        cls = geom.get("class", "")
        if not cls.endswith("_collision"):
            continue
        hand = cls.startswith(hand_classes)
        geom.attrib.update(
            friction="4 0 0" if hand else "1 0 0",
            condim="4",
            solref="0.004 1",
            solimp="0.95 0.95 0.001 0.5 2" if hand else "0.9 0.95 0.001 0.5 2",
            margin="0",
            priority="0",
            solmix="1",
        )
        # Leave gap untouched, including values inherited from default classes.
        # Current robot geoms compile to zero; the runtime fallback is separate.
