"""Hand telemetry decoder — linear-fit placeholder.

WARNING: SDK-pending. The Linker SDK has not yet defined an angle
convention; this module ships a linear interpolation from an SDK-shaped
0–100 value per channel to the URDF [lower, upper] limit of the
corresponding actuated joint. When the SDK lands an angle convention,
bump `CONVENTION` to `sdk-vN` and re-run any bagged data stamped with
`linear-fit-v0`.

Convention (verified empirically against Linker Hand O6 telemetry):

    sdk_value = 100 -> joint at URDF lower limit (rest / open)
    sdk_value = 0   -> joint at URDF upper limit (full travel)
    joint = lower + (100 - sdk)/100 * (upper - lower)

Matches `linker_sim.io.replay.hands` (raw=full-scale byte → lower limit)
but operates on the SDK 0–100 percent scale rather than 0–255 bytes.

Tracked at:

- docs/known_limitations.md (linear-fit + UMI-Dex path hack)
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from linker_robot_assets import asset_root


CONVENTION = "linear-fit-v0"

_VALID_SIDES = ("left", "right")
_SIDE_TO_PREFIX = {"left": "l", "right": "r"}
_VALID_SLOT_MODES = ("active", "raw")

# Placeholder in `decoder.yaml::slots` for an SDK wire slot that carries no
# actuated joint (预留 / reserved). Present so the file describes the SDK's
# real vector width, letting callers hand over a recorded packet verbatim.
_RESERVED = "reserved"


def _resolve_component_dir(name: str, component_root: Path | None) -> Path:
    root = component_root or (asset_root() / "components" / "hands")
    cdir = root / name
    if not cdir.is_dir():
        raise FileNotFoundError(
            f"hand component {name!r} not found at {cdir} "
            f"(component_root={root})"
        )
    return cdir


def _read_decoder_yaml(cdir: Path) -> dict:
    path = cdir / "decoder.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"{path} missing — author it per Phase 4.2")
    with path.open() as f:
        spec = yaml.safe_load(f) or {}
    declared = spec.get("convention")
    if declared != CONVENTION:
        raise ValueError(
            f"{path}: convention {declared!r} != module CONVENTION {CONVENTION!r}. "
            "Bump the file or the module to match."
        )
    if not (spec.get("slots") or spec.get("channels")):
        raise ValueError(f"{path}: needs a 'slots' or 'channels' list")
    return spec


def _channel_lists(
    spec: dict, cdir: Path, *, legacy: bool
) -> tuple[list[str], list[str]]:
    """Resolve (slot_names, active_names) for one hand's SDK vector.

    `slots` describes the SDK's full wire vector and may contain `reserved`
    placeholders; `active_names` drops them. A hand whose vector has no
    reserved slots declares `channels` instead, and the two lists coincide.
    """
    if legacy:
        chans = spec.get("legacy_channels")
        if not chans:
            raise ValueError(
                f"{cdir / 'decoder.yaml'}: no 'legacy_channels' list "
                "(required for legacy=True)"
            )
        return list(chans), list(chans)

    slots = spec.get("slots")
    if slots:
        active = [s for s in slots if s != _RESERVED]
        if not active:
            raise ValueError(
                f"{cdir / 'decoder.yaml'}: 'slots' has no non-reserved entries"
            )
        return list(slots), active

    chans = list(spec["channels"])
    return chans, chans


def sdk_channel_width(
    name: str,
    *,
    slots: str = "active",
    legacy: bool = False,
    component_root: Path | None = None,
) -> int:
    """Column count `decode_hand` expects for this hand in the given mode.

    Lets a caller size a recording's per-hand column block without
    hard-coding it: `"raw"` is the SDK's full wire vector (reserved slots
    included), `"active"` is the actuated-joint count.
    """
    if slots not in _VALID_SLOT_MODES:
        raise ValueError(f"slots {slots!r} not in {_VALID_SLOT_MODES}")
    cdir = _resolve_component_dir(name, component_root)
    spec = _read_decoder_yaml(cdir)
    slot_names, active_names = _channel_lists(spec, cdir, legacy=legacy)
    return len(slot_names) if slots == "raw" else len(active_names)


def _expand_template(name: str, side: str) -> str:
    return name.replace("{S}", _SIDE_TO_PREFIX[side])


def _read_urdf_limits(
    urdf_path: Path, joint_names: list[str]
) -> tuple[np.ndarray, np.ndarray]:
    """Extract per-joint (lower, upper) from a URDF via xml.etree.

    Avoids pulling yourdfpy as a dep — joint limits are a single xpath
    query and the rest of yourdfpy's machinery is unused here.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    by_name = {j.get("name"): j for j in root.findall("joint")}
    lo = np.zeros(len(joint_names), dtype=np.float64)
    hi = np.zeros(len(joint_names), dtype=np.float64)
    for i, name in enumerate(joint_names):
        joint = by_name.get(name)
        if joint is None:
            raise KeyError(
                f"{urdf_path}: joint {name!r} not found "
                f"(available: {sorted(by_name)})"
            )
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(
                f"{urdf_path}: joint {name!r} has no <limit> element"
            )
        lo[i] = float(limit.get("lower", "0"))
        hi[i] = float(limit.get("upper", "0"))
    return lo, hi


def _urdf_actuated_order(urdf_path: Path) -> list[str]:
    """Actuated (non-fixed, non-mimic) joint names, in URDF document order.

    Mirrors ``composer.urdf_ops.collect_joint_names`` so the decoder's output
    column order lines up with ``handle.joints[role]`` — the order the replay
    pipeline feeds hand columns against positionally.
    """
    root = ET.parse(urdf_path).getroot()
    names: list[str] = []
    for j in root.findall("joint"):
        if j.get("type") == "fixed" or j.find("mimic") is not None:
            continue
        name = j.get("name")
        if name:
            names.append(name)
    return names


def _resolve_hand_urdf(cdir: Path, name: str, side: str) -> Path:
    """Locate a hand's per-side URDF, tolerating both component layouts.

    Most hands (l6/l25/l20lite/l30) use ``variants/<side>/hand.urdf``; o6 ships
    a flat ``<name>_<side>.urdf`` at the component root. Try the variants path
    first, then fall back to the flat one.
    """
    candidates = (
        cdir / "variants" / side / "hand.urdf",
        cdir / f"{name}_{side}.urdf",
    )
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError(
        f"{name}/{side}: no hand URDF found "
        f"(tried {[str(c) for c in candidates]})"
    )


def decode_hand(
    name: str,
    side: str,
    sdk_0_100: np.ndarray,
    *,
    legacy: bool = False,
    slots: str = "active",
    component_root: Path | None = None,
) -> np.ndarray:
    """Linear interp from SDK [0, 100] to URDF [lower, upper] per joint.

    Args:
        name: hand component directory name (e.g. ``"linkerhand_l6"``).
        side: ``"left"`` or ``"right"``.
        sdk_0_100: per-channel SDK values, shape ``(n_channels,)`` or
            ``(T, n_channels)``. Float; values outside [0, 100] are clipped.
            Columns are in the selected channel list's (SDK) order.
        legacy: use ``decoder.yaml::legacy_channels`` (an alternate SDK
            channel order shipped by early/buggy client devices) instead of
            the default ``channels``. Both list the same joints; only the
            input column order differs.
        slots: ``"active"`` (default) means the input holds one column per
            actuated joint, with any reserved SDK slots already stripped.
            ``"raw"`` means the input is the SDK's full wire vector, reserved
            slots included, as stored by recorders that save the packet
            verbatim — this function drops them. Use
            :func:`sdk_channel_width` to size the block either way.
        component_root: override the asset-tree component root (test-only).

    Returns:
        Radians, dtype float32, one column per actuated joint, reordered from
        SDK channel order into the URDF actuated-joint document order (==
        ``handle.joints[role]``, which replay feeds positionally). Leading
        dimensions are preserved; the last is the actuated-joint count, which
        is narrower than the input when ``slots="raw"`` and the hand has
        reserved slots.

    Raises:
        FileNotFoundError: component dir or ``decoder.yaml`` missing.
        ValueError: ``decoder.yaml`` convention mismatch, channel count
            mismatch, bad ``slots`` mode, missing ``legacy_channels`` when
            ``legacy=True``, or a joint without a ``<limit>`` element.
        KeyError: a templated joint name not present in the variant URDF.
    """
    if side not in _VALID_SIDES:
        raise ValueError(f"side {side!r} not in {_VALID_SIDES}")
    if slots not in _VALID_SLOT_MODES:
        raise ValueError(f"slots {slots!r} not in {_VALID_SLOT_MODES}")

    cdir = _resolve_component_dir(name, component_root)
    spec = _read_decoder_yaml(cdir)
    slot_names, active_names = _channel_lists(spec, cdir, legacy=legacy)
    joint_names = [_expand_template(c, side) for c in active_names]

    sdk = np.asarray(sdk_0_100, dtype=np.float32)
    expected = len(slot_names) if slots == "raw" else len(joint_names)
    if sdk.shape[-1] != expected:
        hint = ""
        other = len(joint_names) if slots == "raw" else len(slot_names)
        if other != expected and sdk.shape[-1] == other:
            other_mode = "active" if slots == "raw" else "raw"
            hint = (
                f" — that is this hand's {other_mode!r} width, so the columns "
                f"are {'already stripped' if other_mode == 'active' else 'the raw SDK packet'}"
                f"; pass slots={other_mode!r}"
            )
        raise ValueError(
            f"{name}/{side}: slots={slots!r} expects {expected} columns "
            f"but input has shape {sdk.shape}{hint}"
        )

    if slots == "raw" and len(slot_names) != len(joint_names):
        keep = [i for i, s in enumerate(slot_names) if s != _RESERVED]
        sdk = sdk[..., keep]

    urdf_path = _resolve_hand_urdf(cdir, name, side)
    lo, hi = _read_urdf_limits(urdf_path, joint_names)

    # Optional per-joint clip overrides (joint name -> [lo, hi]).
    overrides = spec.get("clip_overrides") or {}
    for jname, bounds in overrides.items():
        try:
            i = joint_names.index(_expand_template(jname, side))
        except ValueError:
            # Override may be templated/untemplated; try the other form.
            i = joint_names.index(jname) if jname in joint_names else -1
        if i >= 0:
            lo[i] = float(bounds[0])
            hi[i] = float(bounds[1])

    sdk_clipped = np.clip(sdk, 0.0, 100.0)
    out = lo.astype(np.float32) + ((100.0 - sdk_clipped) / 100.0) * (hi - lo).astype(np.float32)

    # The channel list is in SDK/hardware order; the sim feeds hand columns
    # positionally against the URDF's actuated-joint document order
    # (== handle.joints[role]). Reorder so returned columns line up with it.
    manifest = _urdf_actuated_order(urdf_path)
    if sorted(manifest) != sorted(joint_names):
        raise ValueError(
            f"{name}/{side}: decoder.yaml channels {joint_names} do not match "
            f"the URDF's actuated joints {manifest} ({urdf_path})"
        )
    channel_idx = {jn: i for i, jn in enumerate(joint_names)}
    perm = [channel_idx[m] for m in manifest]
    return out[..., perm].astype(np.float32)
