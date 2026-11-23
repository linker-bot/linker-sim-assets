# Changelog

The version is the git tag; the `linker-lw-v0` package carries the same version.

## 0.5.0 (unreleased)

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
