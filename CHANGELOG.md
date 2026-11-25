# Changelog

The version is the git tag; the `linker-lw-v0` package carries the same version.

## 1.0.0

The files of 0.7.0 with a promise: within 1.x, LW-v0's paths, names, physics and geometry stay
as they are (README "Versions"; README.zh-CN "版本"). The L6, L20 Lite, L25 and L30 hands are not
covered until they are verified.

- **The table's work surface is green** (`#2E7D5B`, an anti-static mat's green), the rest of the
  support grey as before. Only the colour of the table's visual mesh changes (its texture
  coordinates and `lw_v0_palette.png`); its geometry and all physics are the same as 0.7.0's.

- Package `linker-lw-v0` 1.0.0; products install it at the tag `v1.0.0`.
- Speeds measured with 0.6.0's and 0.7.0's files in linker-lab and linker-sim: the same.

## 0.7.0

**Parts and assemblies; no physics change.** LW-v0 is now built from parts, each a robot of its
own with its MJCF, URDF and meshes: `arms/ar5_08/` (left, right), `flanges/ar5_08_o6/` (one for
both sides), `brackets/gemini335l_wrist/` (left, right), `brackets/zed2i_column/`,
`cameras/gemini335l/`, `cameras/zed2i/`, `supports/lw_v0/` and `hands/o6/`.

- **Moved:** `workstations/lw_v0/` → `assemblies/lw_v0/`; the meshes into their parts' folders.
  Products change one path (`importlib.resources.files("linker_lw_v0") / "assemblies/lw_v0"`).
  Body, joint, site, camera, drive and coupling names are unchanged.
- **Written, not edited:** `assemblies/lw_v0/{left,right,top_camera}.xml`, `lw_v0.urdf` and each
  part's URDF are written by `tools/assemble.py` from the parts and `assembly.yaml`; CI checks
  they are current.
- **The same physics:** LW-v0 compiles to the same model as 0.6.0 on MuJoCo 3.17.0 and 4.0.1
  (bodies, joints, drives, couplings, contacts, exclusions, collision shapes, options, keyframe).
  It is frozen in `tests/verified/lw_v0.json`.
- **ZED 2i body frame on the camera:** `top_camera/sensor_camera`'s frame was 0.43 m from the
  camera; it is now the maker's camera frame (x forward, y left, z up). The camera's numbers in
  the files change (its mount, inertia frame, optical frames, collision mesh); in the world
  nothing moves (within 1e-8 m: the collision mesh's single-precision vertices).
- **Palettes:** one per part, `<folder>_palette.png`; colours unchanged.
- [MOUNTS.md](MOUNTS.md): every mount frame, its axes and the parts that fit it.

## 0.6.0

**Physics change** (the O6 standalone and LW-v0's hands):

- **Thumb yaw passive damping on the joint:** `lh/rh_thumb_cmc_yaw` joint `damping` 0 → 4, its
  drive's `kv` 24 → 20. The real passive damping is 4; until now it sat inside the drive
  (24 = the motor's output damping 20 + 4), so it vanished whenever the drive's 10 N·m limit
  saturated (fast moves, pressing an object). On the joint it always acts. The motor model's
  derived passive damping is 4 either way (README "Robot controllers").
- **Hand mount at exactly π:** LW-v0's hand bodies sit on the flange with `quat="0 0 0 1"`
  instead of the rounded `euler="0 0 ±3.14159265"` (+ left, − right); each hand moves by
  3.6 nrad and both sides now use the same rotation. The URDF writes π to 10 digits.
- URDFs regenerated from the MJCF (`o6_left/right.urdf`, `lw_v0.urdf`).

**O6 motor model `geared-shaft-v1`** (`hands/o6/motor.yaml`; README "Robot controllers"):
each physics step the drive's target is kept so the drive's torque stays 1 N·m within its force
limit, as the products already do for the arm. On MuJoCo 4 a drive held at its limit loses its
implicit damping (MuJoCo 4.0.0) and oscillates there; the motor model read that as a load and
stopped the finger for good. The constants are unchanged; the model has a new name, so
products that read 0.5.0's model name refuse this one.

- `hands/o6/motor.yaml` and `arms/ar5_08/controller.yaml`: the O6 motor's constants and the
  AR5_08 controller's limits as data (until now written in linker-sim's and linker-lab's
  code); README "Robot controllers" writes out both models. In `linker-lw-v0`.

## 0.5.0

The repository now holds robot files, not tools: LW-v0 and five LinkerHand models, each as URDF
and MJCF with its meshes, edited directly.

- **LW-v0** (`workstations/lw_v0/`): the camera workstation linker-sim and linker-lab simulate.
  Its physics is the version both products were verified with: the same bodies, joints, drives,
  contact settings and collision shapes.
  - Visual meshes coloured with the hardware team's materials; the geometry is unchanged.
  - Camera bodies are the makers' meshes (Orbbec, Stereolabs), placed in the same camera frames.
  - Camera intrinsics on `<camera>` and the `home` keyframe are in the MJCF.
  - `lw_v0.urdf`: the whole station as one tree.
- **Hands** (`hands/{o6,l6,l20lite,l25,l30}/`): URDF + MJCF per side and `sdk_channels.yaml`.
  - O6: LW-v0's hand (drives, contact, collision hulls, couplings) as a standalone robot, coloured.
    This changes the standalone O6's physics from v0.4.0's.
  - L6, L20 Lite, L25, L30: unchanged in content, renamed; neutral grey until their real colours
    are known (v0.4.0's URDF colours were not matched to the products). L30's actuators
    now follow its joint order (ctrl order changed).
- **Package** `linker-lw-v0`: LW-v0's files for Python products (`importlib.resources`),
  installed from this repository at a tag; not on PyPI.
- **Removed** (still at tag `v0.4.0`): the composer, recipes, units, manifests, validators, the
  `linker_robot_assets` Python package, the other arms (AR5, A7, A7 Lite, P7), torsos, the bench
  table, the standalone camera and attachment components, and every other workstation.
- Licence: Apache-2.0. Meshes are plain git files; nothing new is in Git LFS.
