"""Write each assembly's units and URDFs from its components.

    python tools/assemble.py           # write the files
    python tools/assemble.py --check   # write nothing; exit 1 if a written file is out of date

For every `assemblies/<name>/assembly.yaml` this writes, next to it:
- one MJCF per unit (`left.xml`, ...): the unit's parts copied into one file, each part's root
  body inside the mount frame it names (`mount`), every name with the part's prefix
- `<name>.urdf` from `<name>.xml` (the hand-edited scene that places the units)
and a URDF next to every component an assembly uses (`arms/ar5_08/ar5_08_left.urdf`, ...).

The tool copies XML text: every number reaches the unit exactly as written in the component.
It holds no physics. Besides placing, prefixing and merging, it only
- drops the parts' solver options (`<option>`, `<size>`): the assembly's scene sets them
- rewrites mesh and texture paths so they still point at the components' files
- with `parts_collide: false`, excludes contacts between every two bodies of different parts
Edit the components or assembly.yaml, never the written files; CI runs `--check`.
"""

import argparse
import itertools
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import urdf  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
HEADER = "<!-- Written by tools/assemble.py from {source} and the components it names. Edit those, not this file. -->"
# attributes that name an element or refer to one: they get the part's prefix
NAMES = {"name", "class", "childclass", "mesh", "material", "texture", "hfield", "skin", "body", "body1", "body2",
         "joint", "joint1", "joint2", "geom", "geom1", "geom2", "site", "tendon", "actuator", "camera", "target",
         "objname", "refname"}
FILES = {"mesh", "texture", "hfield", "skin"}  # elements whose `file` is a path
SECTIONS = ("default", "asset", "worldbody", "actuator", "tendon", "equality", "contact", "sensor")
DROPPED = ("option", "size")  # solver settings: the assembly's scene owns them


def prefixed(element, prefix):
    for e in element.iter():
        for key in NAMES & set(e.attrib):
            if not (key == "class" and e.get(key) == "main"):
                e.set(key, prefix + e.get(key))


def rebased(element, source_dir, unit_dir, compiler):
    for e in element.iter():
        if e.tag in FILES and e.get("file"):
            folder = compiler.get("meshdir" if e.tag == "mesh" else "texturedir", "")
            path = (source_dir / folder / e.get("file")).resolve()
            e.set("file", Path(os.path.relpath(path, unit_dir)).as_posix())


def collision_bodies(path):
    """The bodies of a part that carry a colliding geom (compiled, so defaults are resolved)."""
    model = mujoco.MjModel.from_xml_path(str(path))
    return [model.body(b).name for b in range(1, model.nbody)
            if any(model.geom_contype[g] or model.geom_conaffinity[g] for g in range(model.ngeom) if model.geom_bodyid[g] == b)]


def unit_xml(name, spec, folder):
    """One unit's MJCF tree, and the components it used."""
    unit = ET.Element("mujoco", {"model": name})
    sections = {}
    compiler, colliders, used = None, {}, []
    for part, p in spec["parts"].items():
        path = (folder / p["file"]).resolve()
        used.append(path)
        tree = ET.parse(path).getroot()
        unknown = [c.tag for c in tree if c.tag not in (*SECTIONS, *DROPPED, "compiler")]
        assert not unknown, f"{path}: {unknown} can't be assembled (a keyframe belongs in the assembly's scene)"
        part_compiler = tree.find("compiler")
        part_compiler = dict(part_compiler.attrib) if part_compiler is not None else {}
        if compiler is None:
            compiler = part_compiler
            ET.SubElement(unit, "compiler", compiler)
            for tag in SECTIONS:
                sections[tag] = ET.SubElement(unit, tag)
        assert part_compiler == compiler, f"{path}: compiler {part_compiler} differs from the unit's {compiler}"
        prefix = p["prefix"]
        for section in SECTIONS:
            block = tree.find(section)
            if block is None:
                continue
            prefixed(block, prefix)
            rebased(block, path.parent, folder, part_compiler)
            if section != "worldbody":
                sections[section].extend(block)
                continue
            if "mount" not in p:
                assert not len(sections["worldbody"]), f"{name}: only the first part is the root; give {part} a `mount`"
                sections["worldbody"].extend(block)
                continue
            frames = [f for f in sections["worldbody"].iter("frame") if f.get("name") == p["mount"]]
            assert len(frames) == 1, f"{name}/{part}: mount frame {p['mount']!r} found {len(frames)} times"
            assert not len(frames[0]), f"{name}/{part}: mount frame {p['mount']!r} is already used"
            frames[0].extend(block)
        colliders[part] = [prefix + b for b in collision_bodies(path)]

    contact = sections["contact"]
    if not spec["parts_collide"]:
        for a, b in itertools.combinations(colliders, 2):
            for b1, b2 in itertools.product(colliders[a], colliders[b]):
                contact.append(ET.Element("exclude", {"body1": b1, "body2": b2}))
    # exclusions in body order (tree order), each pair once
    order = {b.get("name"): i for i, b in enumerate(sections["worldbody"].iter("body"))}
    pairs = {}
    for e in contact.findall("exclude"):
        b1, b2 = sorted((e.get("body1"), e.get("body2")), key=order.__getitem__)
        pairs.setdefault((b1, b2), ET.Element("exclude", {"body1": b1, "body2": b2}))
        contact.remove(e)
    rest = list(contact)
    contact.clear()
    contact.extend([pairs[k] for k in sorted(pairs, key=lambda k: (order[k[0]], order[k[1]]))] + rest)

    for tag, block in sections.items():
        if not len(block) and tag != "worldbody":
            unit.remove(block)
    return unit, used


def text(tree, source):
    ET.indent(tree, "  ")
    return "\n".join(["<?xml version='1.0' encoding='utf-8'?>", HEADER.format(source=source),
                      ET.tostring(tree, encoding="unicode"), ""])


def assemble(check=False):
    """Write every file the tool owns (or, with check, write nothing); the paths that changed."""
    # path -> its text, or (MJCF, robot name, velocity limits) for a URDF, written after the units
    files, components = {}, {}
    for yaml_path in sorted(ROOT.glob("assemblies/*/assembly.yaml")):
        folder = yaml_path.parent
        spec = yaml.safe_load(yaml_path.read_text())
        limits = {}
        for name, unit in spec["units"].items():
            tree, used = unit_xml(name, unit, folder)
            files[folder / f"{name}.xml"] = text(tree, yaml_path.relative_to(ROOT).as_posix())
            for path in used:
                components[path] = urdf.velocity_limits(path.parent)
                limits.update(components[path])
        scene = folder / f"{folder.name}.xml"
        for model in ET.parse(scene).getroot().iter("model"):  # components the scene loads itself
            path = (folder / model.get("file")).resolve()
            if path.parent != folder:
                components[path] = urdf.velocity_limits(path.parent)
        files[folder / f"{folder.name}.urdf"] = (scene, folder.name, limits)
    for path, limits in components.items():
        files[path.with_suffix(".urdf")] = (path, path.stem, limits)

    changed = []
    for path, content in sorted(files.items(), key=lambda kv: isinstance(kv[1], tuple)):
        if isinstance(content, tuple):  # with check: the URDF of the MJCF files as they are
            content = urdf.write(*content)
        if not path.exists() or path.read_text() != content:
            changed.append(path.relative_to(ROOT).as_posix())
            if not check:
                path.write_text(content)
    return changed, len(files)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="write nothing; fail if a written file is out of date")
    args = ap.parse_args()
    changed, total = assemble(args.check)
    if args.check:
        if changed:
            print("out of date (run python tools/assemble.py):", *changed, sep="\n  ")
            sys.exit(1)
        print(f"{total} written files are up to date")
    else:
        print(f"wrote {len(changed)} of {total} files", *changed, sep="\n  ")


if __name__ == "__main__":
    main()
