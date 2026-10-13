"""The tested 2 ms O6 position-servo/contact profile; no runtime controller."""

import xml.etree.ElementTree as ET
from collections.abc import Sequence

from .schemas import SchemaError

O6_TIMESTEP = 0.002
O6_SOLREF = "-10000000 -63000"
O6_SOLIMP = "0.9999 0.9999 0.001 0.5 2"
O6_CONTACT_SOLREF = "0.004 1"
O6_CONTACT_SOLIMP = "0.95 0.95 0.001 0.5 2"


def configure_o6_hand(root: ET.Element, passive: Sequence[str]) -> None:
    """Scope the profile to the selected O6 subtree, including prefixed scenes."""
    thumbs = [name for name in passive if name.endswith("thumb_ip")]
    if len(thumbs) != 1:
        raise SchemaError("o6_coupled requires the O6 thumb_ip linkage")
    prefix = thumbs[0].removesuffix("thumb_ip")
    hand = root.find(f".//body[@name='{prefix}hand_base_link']")
    if hand is None or not prefix.endswith(("lh_", "rh_")):
        raise SchemaError("o6_coupled could not identify its O6 hand subtree")
    # A scene may mount another component below this hand. Do not tune its
    # joints, contacts or actuators just because it is a descendant body.
    bodies = [b for b in hand.iter("body") if b.get("name", "").startswith(prefix)]
    selected_joints = [j for b in bodies for j in b.findall("joint")]
    joints = {j.get("name") for j in selected_joints}
    if not set(passive) <= joints:
        raise SchemaError("o6_coupled passive joints must belong to the same O6")
    for joint in selected_joints:
        joint.set("solreflimit", "0.000125 1")
        joint.set("solimplimit", O6_SOLIMP)
    for geom in (g for b in bodies for g in b.findall("geom")):
        geom.set("solref", O6_CONTACT_SOLREF)
        geom.set("solimp", O6_CONTACT_SOLIMP)
        geom.set("margin", "0")
        geom.set("gap", "0")
    actuators = [a for a in root.findall("actuator/*") if a.get("joint") in joints]
    if len(actuators) != 6 or any(a.tag != "position" for a in actuators):
        raise SchemaError("o6_coupled requires six native position actuators")
    for act in actuators:
        name = act.get("joint").removeprefix(prefix)
        cap = (
            "1"
            if name == "thumb_cmc_yaw"
            else ("4" if prefix.endswith("lh_") else "3.8148")
            if name == "thumb_cmc_pitch"
            else "2.55"
        )
        act.set("kp", "600")
        act.set("kv", "4.5")
        act.set("forcelimited", "true")
        act.set("forcerange", f"-{cap} {cap}")
    # The palm convex collider fills the real thumb-base clearance. This is
    # the same single-pair exclusion used in the validated DemoGrasp profile.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    pair = (prefix + "hand_base_link", prefix + "thumb_metacarpals")
    if not any(
        {e.get("body1"), e.get("body2")} == set(pair)
        for e in contact.findall("exclude")
    ):
        ET.SubElement(contact, "exclude", body1=pair[0], body2=pair[1])


def configure_o6_solver(option: ET.Element) -> None:
    for key, value in {
        "integrator": "implicitfast",
        "solver": "Newton",
        "iterations": "100",
        "tolerance": "1e-8",
        "cone": "elliptic",
        "impratio": "10",
    }.items():
        option.set(key, value)
    flag = option.find("flag")
    if flag is None:
        flag = ET.SubElement(option, "flag")
    flag.set("nativeccd", "disable")
