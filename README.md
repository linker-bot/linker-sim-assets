# linker-sim-assets

Robot files from Linkerbot for your own simulator: URDF and MJCF with their meshes.

(中文: [README.zh-CN.md](README.zh-CN.md))

| Folder | Robot | Simulated by us |
| --- | --- | --- |
| [`workstations/lw_v0/`](workstations/lw_v0) | **LW-v0**: two AR5_08 arms with LinkerHand O6 hands, wrist cameras (Orbbec Gemini 335L), a top camera (Stereolabs ZED 2i) and the table | Yes: [linker-sim](https://github.com/linker-bot/linker-sim) (MuJoCo 4.0.1) and linker-lab (Isaac Lab, MuJoCo 3.17) |
| [`hands/o6/`](hands/o6) | LinkerHand O6, left and right | Yes, as part of LW-v0 (the same hand) |
| [`hands/l6/`](hands/l6), [`l20lite/`](hands/l20lite), [`l25/`](hands/l25), [`l30/`](hands/l30) | LinkerHand L6, L20 Lite, L25, L30 | No (see [Not yet verified](#not-yet-verified)) |

Other robots (the AR5, A7, A7 Lite and P7 arms, torsos, other workstations) are at tag
[`v0.4.0`](https://github.com/linker-bot/linker-sim-assets/tree/v0.4.0). They come back here one
at a time, after each has been verified in our simulators.

## Which file to load

| You use | Load |
| --- | --- |
| MuJoCo | `workstations/lw_v0/lw_v0.xml` (the whole station), or one part: `left.xml`, `right.xml`, `top_camera.xml`, `support.xml`. A hand: `hands/<hand>/<hand>_<side>.xml` |
| ROS, Isaac Sim, PyBullet, Drake, viser and other URDF tools | `workstations/lw_v0/lw_v0.urdf`, or `hands/<hand>/<hand>_<side>.urdf` |
| Python (linker-sim, linker-lab) | The package `linker-lw-v0` from this repository at a tag, e.g. in `pyproject.toml`: `"linker-lw-v0 @ git+https://github.com/linker-bot/linker-sim-assets@v0.5.0"`; then `importlib.resources.files("linker_lw_v0") / "workstations/lw_v0/lw_v0.xml"` |

The package `linker-lw-v0` holds LW-v0's files at the same paths as this repository, the O6's
meshes it uses, and `hands/o6/sdk_channels.yaml`. It contains no code. Everything else: clone the
repository or download its ZIP.

`lw_v0.xml` places the four parts with MuJoCo's `<attach>`. Each part is one articulation: an arm
part is arm, flange, hand, camera bracket and wrist camera. A simulator without `<attach>` can load
the parts and place them at the poses in `lw_v0.xml` itself.

## Frames and units

- Metres, kilograms, seconds, radians. Every mesh is in its link's frame, in metres.
- LW-v0's world frame: origin between the two arm bases, on top of the column; x forward (towards
  the table and the wrist cameras' view), y to the left arm, z up. The table top is at z = -0.5 m,
  the floor at z = -1.267 m (the table's feet).
- Names: in `lw_v0.xml` every name has its part as prefix (`left/`, `right/`, `top_camera/`,
  `support/`), e.g. `left/arm_AR5_5_08L_W4C4A6_joint_1`, `left/hand_lh_index_mcp_pitch`.
  `lw_v0.urdf` uses the same names. A hand's own files use the hand's names
  (`lh_index_mcp_pitch`).

## LW-v0

| | |
| --- | --- |
| Joints | Per side: 7 arm joints, 6 driven hand joints, 5 hand joints coupled to them; 36 in total |
| Drives | Position drives on the 7 arm joints and 6 hand joints per side (`<actuator>`) |
| Home pose | Keyframe `home` in `lw_v0.xml` |
| Physics | 480 Hz (`timestep` 1/480 s), implicit-fast, Newton solver, elliptic friction cone; in `<option>` of `lw_v0.xml` |

Cameras (`<camera>` in the part files; the camera looks along its optical body's +z, image x right,
y down, as in OpenCV):

| Camera | Model | Resolution | fx, fy (px) | Rate on the real robot |
| --- | --- | --- | --- | --- |
| `left/camera_rgb_optical`, `right/…` | Gemini 335L colour | 1280 × 800 | 596.81, 593.02 | 60 Hz |
| `left/camera_depth_optical`, `right/…` | Gemini 335L depth | 1280 × 800 | 640.00, 627.87 | 30 Hz |
| `top_camera/sensor_left_optical` | ZED 2i left, 2.1 mm lens | 1280 × 720 | 448.13, 514.13 | 60 Hz |

The principal point is the image centre. The rates are not in the files.

### What the MJCF carries that URDF can't

URDF has no place for these, so they are only in the MJCF:

- drives and their gains (`<position kp kv>`), armature
- gravity compensation: every robot body is gravity-compensated (`gravcomp="1"`)
- contact settings: friction, `condim`, `solref`, `solimp`, `margin`, `gap`; which body pairs
  don't collide (`<contact><exclude>`): within an arm part, only different fingers collide
- the stiffness of the hand's couplings (`<equality>`); URDF's `mimic` has only the ratio
- solver options, the home keyframe and the camera intrinsics

`lw_v0.urdf` carries the rest: links with masses and inertias, visual and collision meshes,
joints with limits, couplings as `mimic`, damping and friction. In it:
- `effort` is the MJCF's force limit (arm J1–J4 120 N·m, J5–J7 100 N·m; hand thumb yaw 10, the
  other drives 60); a coupled hand joint has no drive, so its effort is 0
- `velocity` is not simulated by the MJCF. The arm's values are the robot controller's motion
  limits (J1–J7: 0.39, 0.32, 0.39, 0.39, 0.49, 0.49, 0.49 rad/s); a hand drive's is 1 rad/s, a
  coupled joint's that times its ratio

### Measured and modelled

- Masses, inertias and kinematics: from the manufacturers' URDF and CAD.
- Drives, torque limits and contact settings: modelled. They are the values LW-v0's grasping
  policies were trained with in linker-lab and run with in linker-sim.
- Collision shapes: convex hulls, the shapes the grasping policies were trained with; not the
  visual meshes.
- Colours: matched to product photos by the hardware team, not to manufacturers' colour codes.
- Camera bodies: the makers' own meshes (see NOTICE).

## Hands

Each hand folder has `<hand>_left.urdf`, `<hand>_left.xml`, the same for the right hand,
`sdk_channels.yaml` and `meshes/`.

- **O6**: the hand of LW-v0, standalone: the same drives, contact settings, collision hulls,
  couplings and solver options as in LW-v0.
- **L6, L20 Lite, L25, L30**: carried over from v0.4.0 unchanged in content. They collide with
  convex hulls of their visual meshes. They are uncoloured (one neutral grey) until we have their
  real colours, as we do for the O6.

### SDK channels

The hand SDK sends and receives one value per driven joint, 0 to 100, in its own order.
`sdk_channels.yaml` lists which joint each channel drives (`{S}` is `l` or `r`):

- `channels`: the SDK's order; `reserved` marks a slot that drives nothing
- `legacy_channels` (L6): the order of early devices
- `slots` (L25): the full 20-slot SDK layout, with 4 reserved slots
- `convention: linear-fit-v0`: SDK 100 is the joint's lower limit and 0 its upper limit:

  ```
  angle = lower + (100 - value) / 100 * (upper - lower)
  value = 100 - 100 * (angle - lower) / (upper - lower)
  ```

  with `lower` and `upper` the joint's limits in the URDF, or `clip_overrides` where listed.

### Not yet verified

L6, L20 Lite, L25 and L30 compile in MuJoCo and their URDF and MJCF agree (the test below), but
we have not simulated them. Known differences in their data, to be fixed with the hardware team:

- Left and right disagree: L25's URDF effort/velocity (100/1 left, 1/100 right); L20 Lite's link
  masses (up to 2.3×), two effort limits and the ring/pinky coupling (1.30 left, 1.34 right);
  L30's wrist link mass (0.707 kg left, 0.076 kg right), index MCP limit (1.57 / 1.518 rad) and
  thumb effort/velocity.
- The URDF's joint damping and friction differ from the MJCF's (MJCF: damping 4–5, friction loss
  0.5).
- Their MJCF drives have no force limits.

## Known issues of LW-v0

LW-v0 keeps the geometry both products were verified with. These are deferred to a later
workstation (LW-v1), because fixing them changes the physics or the camera views:

- The wrist cameras sit 16.4 mm higher on their brackets than in the brackets' CAD.
- The top camera's optical frames lie 22.7 mm in front of where the ZED 2i's own description
  puts them, along the optical axis.

## For contributors

Edit the URDF and the MJCF directly; there is no generator. `tests/test_assets.py` checks that
every MJCF compiles and that each URDF matches its MJCF (joints, couplings, masses, inertias, link
poses at the home pose, mesh files), that every referenced file exists and every mesh is used, the
mesh format, and the SDK channel files:

```
uv run --group test pytest
```

CI runs it on MuJoCo 3.17.0 and 4.0.1, with the package `linker-lw-v0` built and installed.

Meshes: one `.obj` per link, `meshes/visual/<link>_vis.obj` with an `.mtl` and one `palette.png`
per robot (colours are texture coordinates into the palette, so URDF tools and MuJoCo show the same
colours); collision meshes in `meshes/collision/<link>_col.obj`. No STL, glTF or other formats.

## Licence

Apache-2.0 ([LICENSE](LICENSE)). The camera meshes are the makers', also Apache-2.0
([NOTICE](NOTICE)).
