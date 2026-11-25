# linker-sim-assets

Robot files from Linkerbot for your own simulator: URDF and MJCF with their meshes.

(中文: [README.zh-CN.md](README.zh-CN.md))

| Folder | Robot | Simulated by us |
| --- | --- | --- |
| [`assemblies/lw_v0/`](assemblies/lw_v0) | **LW-v0**: two AR5_08 arms with LinkerHand O6 hands, wrist cameras (Orbbec Gemini 335L), a top camera (Stereolabs ZED 2i) and the table | Yes: [linker-sim](https://github.com/linker-bot/linker-sim) (MuJoCo 4.0.1) and linker-lab (Isaac Lab, MuJoCo 3.17) |
| [`hands/o6/`](hands/o6) | LinkerHand O6, left and right | Yes, as part of LW-v0 (the same hand) |
| [`arms/ar5_08/`](arms/ar5_08), [`flanges/`](flanges), [`brackets/`](brackets), [`cameras/`](cameras), [`supports/lw_v0/`](supports/lw_v0) | The other parts of LW-v0, each a robot of its own: the AR5_08 arm (left, right), the arm-to-O6 flange, the camera brackets, the Gemini 335L and ZED 2i cameras, the table with the arm column | Yes, as parts of LW-v0 |
| [`hands/l6/`](hands/l6), [`l20lite/`](hands/l20lite), [`l25/`](hands/l25), [`l30/`](hands/l30) | LinkerHand L6, L20 Lite, L25, L30 | No (see [Not yet verified](#not-yet-verified)) |

Other robots (the AR5, A7, A7 Lite and P7 arms, torsos, other workstations) are at tag
[`v0.4.0`](https://github.com/linker-bot/linker-sim-assets/tree/v0.4.0). They come back here one
at a time, after each has been verified in our simulators.

## Which file to load

| You use | Load |
| --- | --- |
| MuJoCo | `assemblies/lw_v0/lw_v0.xml` (the whole station), or one unit: `left.xml`, `right.xml`, `top_camera.xml`. One part alone: its folder's `.xml`, e.g. `arms/ar5_08/ar5_08_left.xml`, `hands/<hand>/<hand>_<side>.xml` |
| ROS, Isaac Sim, PyBullet, Drake, viser and other URDF tools | `assemblies/lw_v0/lw_v0.urdf`, or a part's `.urdf`, e.g. `hands/<hand>/<hand>_<side>.urdf` |
| Python (linker-sim, linker-lab) | The package `linker-lw-v0` from this repository at a tag, e.g. in `pyproject.toml`: `"linker-lw-v0 @ git+https://github.com/linker-bot/linker-sim-assets@v1.0.0"`; then `importlib.resources.files("linker_lw_v0") / "assemblies/lw_v0/lw_v0.xml"` |

The package `linker-lw-v0` holds LW-v0's files at the same paths as this repository, the O6's
meshes it uses, `hands/o6/sdk_channels.yaml` and `motor.yaml`, and `arms/ar5_08/controller.yaml`. It contains no code. Everything else: clone the
repository or download its ZIP.

`lw_v0.xml` places four units with MuJoCo's `<attach>`: `left.xml`, `right.xml` (each one
articulation: arm, flange, hand, camera bracket and wrist camera), `top_camera.xml` (the ZED 2i
on its column bracket) and the support. A simulator without `<attach>` can load the units and
place them at the poses in `lw_v0.xml` itself.

## Parts and assemblies

Each part is a robot of its own, with its MJCF, URDF and meshes, and loads alone: an arm, a
hand, a flange, a bracket, a camera, the support. A part's root body sits at its own origin;
where another part attaches it has an empty frame `mount_<child>`, so the parent decides where
the child sits. [MOUNTS.md](MOUNTS.md) lists every mount, its axes and the parts that fit it.
Every fact is in one file: the O6 in LW-v0 is `hands/o6/o6_<side>.xml`, not a copy.

An assembly (`assemblies/lw_v0/`) is built from parts by two files edited by hand:
`assembly.yaml` lists each unit's parts, their name prefixes and the mount each goes on; `lw_v0.xml`
places the units and holds the solver options and the home keyframe. MJCF can't place a part
inside another attached part, so `tools/assemble.py` writes the units (`left.xml`, `right.xml`,
`top_camera.xml`) and `lw_v0.urdf`: it copies each part's XML into its mount frame, prefixes the
names, and with `parts_collide: false` keeps different parts of a unit from colliding. It holds no
physics; the units compile to exactly what MuJoCo's own composition (`MjSpec`) gives. The written
files are committed, so you can load them without running anything.

## Frames and units

- Metres, kilograms, seconds, radians. Every mesh is in its link's frame, in metres.
- LW-v0's world frame: origin between the two arm bases, on top of the column; x forward (towards
  the table and the wrist cameras' view), y to the left arm, z up. The table top is at z = -0.5 m,
  the floor at z = -1.267 m (the table's feet).
- Names: in `lw_v0.xml` every name has its unit and part as prefix (`left/` + `arm_`, `flange_`,
  `hand_`, `bracket_`, `camera_`; `top_camera/` + `sensor_`; `support/`), e.g.
  `left/arm_AR5_5_08L_W4C4A6_joint_1`, `left/hand_lh_index_mcp_pitch`. `lw_v0.urdf` uses the same
  names. A part's own files use its own names (`lh_index_mcp_pitch`).

## LW-v0

| | |
| --- | --- |
| Joints | Per side: 7 arm joints, 6 driven hand joints, 5 hand joints coupled to them; 36 in total |
| Drives | Position drives on the 7 arm joints and 6 hand joints per side (`<actuator>`) |
| Home pose | Keyframe `home` in `lw_v0.xml` |
| Physics | 480 Hz (`timestep` 1/480 s), implicit-fast, Newton solver, elliptic friction cone; in `<option>` of `lw_v0.xml` |

Cameras (`<camera>` in the camera parts; the camera looks along its optical body's +z, image x right,
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
  don't collide (`<contact><exclude>`): within an arm unit, only different fingers collide
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

## Robot controllers

The real arm and hand have controllers between a command and the joints. Their behaviour is
not in the URDF or MJCF, which only describe the mechanism and its position drives. The
constants are data files; linker-sim and linker-lab implement both models. **A simulator that
loads only the MJCF has the drives without these controllers:** the fingers reach their
targets as fast as their 60 N·m drives allow, much faster and harder than the real hand.

### O6 hand motor (`hands/o6/motor.yaml`, model `geared-shaft-v1`)

Each driven joint is moved by a motor shaft through a self-locking gear. The shaft is
the joint drive's target: the MJCF `<position>` drive (stiffness `Kp`, damping `kv`) pulls
the joint toward it, and the coupled joints follow through `<equality>`. Once per command
(period `dt`; 1/60 s in our products), for each shaft `s` with target `t` (clamped to the
joint range), measured joint position `q` and its previous value `q0`:

```
v      = (q - q0) / dt                                         joint speed
tau    = clamp(motor_stiffness * (t - s), ±advancing_cap)       motor torque
load   = (tau - Kp * (s - (q + dt*v)) + output_damping * v)
         / (Kp + (output_damping + viscosity) / dt)               step the torque can make against the finger
free   = min(dt * speed_limit, dt * |tau| / (viscosity + passive))  step of the unloaded motor
step   = sign(tau) * min(max(sign(tau) * load, 0), |t - s|, free)  never backwards, never past t
s      = s + step
```

`passive` is the passive damping the shaft sees: the joint's own damping, plus the part of the
drive's `kv` that is not the motor's `output_damping`, plus each coupled joint's damping times
its coupling ratio squared (e.g. thumb flexion: 4 + 0.4 × 1.86² = 5.38384). Between commands
the drive's target moves linearly from the old shaft position to the new one, plus the
velocity feed-forward `output_damping / Kp × step / dt`, so the drive does not lag the shaft.

Each physics step the target is then kept within the drive's force limit (the joint's
`actuatorfrcrange`, 10 N·m thumb yaw, 60 N·m the others) less 1 N·m: if
`Kp * (target - q) - kv * qdot` would exceed it, the target becomes the one that gives exactly
that torque (`q` and `qdot` measured at that step). A drive is then never asked for more than
it can give. Without this step, on MuJoCo 4 a drive held at its limit loses its implicit
damping and oscillates at the limit, and the motor model above, which reads that oscillation
as a load, stops the shaft for good: the finger no longer opens. (New in 0.6.0.)

### AR5_08 arm controller (`arms/ar5_08/controller.yaml`)

The xCore controller moves the joints along time-optimal, jerk-limited trajectories within
each joint's `max_velocity`, `max_acceleration` and `max_jerk`; all joints of a move arrive
together. It keeps (J6, J7) inside the convex `wrist_region` octagon, whose bounding box is
the J6/J7 ranges in the URDF/MJCF. The drives' torque caps (120 N·m J1–J4, 100 N·m J5–J7)
are in the MJCF.

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

## Versions

Each release is a git tag `v<major>.<minor>.<patch>`; the package `linker-lw-v0` has the same
version. Pin a tag (or its commit) and you get exactly those files.

**What 1.x keeps** (from 1.0.0 on, until 2.0.0), for LW-v0 and the parts it is built from:

- **Paths:** `assemblies/lw_v0/lw_v0.xml`, `left.xml`, `right.xml`, `top_camera.xml`,
  `lw_v0.urdf`, and each part's MJCF and URDF where they are today. The package holds them at the
  same paths under `linker_lw_v0/`, with `hands/o6/sdk_channels.yaml`, `hands/o6/motor.yaml` and
  `arms/ar5_08/controller.yaml`.
- **Names:** every body, joint, site, camera, drive (`<actuator>`) and coupling (`<equality>`)
  name, in LW-v0 (`left/hand_lh_index_mcp_pitch`) and in each part's own files
  (`lh_index_mcp_pitch`); the keyframe `home` and its pose.
- **Physics:** LW-v0's compiled model, frozen in `tests/verified/lw_v0.json`, which CI checks on
  every change: bodies, masses and inertias, joints and their limits, drives and their gains,
  couplings, contact settings and exclusions, collision shapes, solver options. The O6's SDK
  channel map, its motor model (`geared-shaft-v1`) and constants, and the arm controller's limits
  stay as they are (also in that file).
- **Geometry:** LW-v0's visual meshes keep their shape and place (also in that file).

**What may change in 1.x:**

- A **minor** version (1.1.0) adds: new robots, parts and assemblies, new files, new names. It
  may correct a colour (a material of a visual mesh).
- A **patch** version (1.0.1) fixes what the promise doesn't cover: documentation, tests, tools,
  file formatting and comments, and the names of meshes, materials, textures and default classes
  inside a file.
- **Not covered:** the L6, L20 Lite, L25 and L30 hands, which we haven't verified in simulation
  ([Not yet verified](#not-yet-verified)). Any 1.x version may change them; once verified, each
  joins the promise. Also `tools/` and `tests/`, which are for contributors.

Anything else, such as a change to LW-v0's physics or geometry or a renamed joint, is either a new
assembly next to LW-v0 (LW-v1, for example with the issues above fixed), or 2.0.0. The changelog
lists every change.

## For contributors

Edit the parts' MJCF, `assembly.yaml` and the assembly's scene (`lw_v0.xml`); then write the rest:

```
uv run --group test python tools/assemble.py
```

It rewrites each assembly's units and URDF, and the URDF of every part an assembly uses (the four
unverified hands keep their own URDFs). Don't edit written files: each starts with a comment
saying so, and CI fails if they differ from what the tool writes. `tests/test_assets.py` checks
that every MJCF compiles and that each URDF matches its MJCF (joints, couplings, masses, inertias,
link poses at the home pose, mesh files), that every referenced file exists and every mesh is
used, the mesh format, the SDK channel and controller files, that the written files are current
and equal MuJoCo's own composition, and that LW-v0's physics is still what was verified:

```
uv run --group test pytest
```

CI runs it on MuJoCo 3.17.0 and 4.0.1, with the package `linker-lw-v0` built and installed.

**Verified assemblies are frozen.** `tests/verified/lw_v0.json` is LW-v0's compiled physics at
the home pose, in the world frame (bodies, joints, drives, couplings, contacts, collision shapes,
options, visual geometry) and its parts' controller data. A part change that alters it fails the test. If the change is meant, write the file
again (`python tools/verified.py lw_v0`) in a change that is a new major version, or make a new
assembly ([Versions](#versions)).

Meshes: one `.obj` per link, `meshes/visual/<link>_vis.obj`, with an `.mtl` and one
`<folder>_palette.png` per part or robot. Colours are texture coordinates into the palette, so URDF tools and
MuJoCo show the same colours. The palette is named after its folder so that tools which gather
textures into one folder (USD converters) keep the palettes apart. Collision meshes are
`meshes/collision/<link>_col.obj`. No STL, glTF or other formats.

## Licence

Apache-2.0 ([LICENSE](LICENSE)). The camera meshes are the makers', also Apache-2.0
([NOTICE](NOTICE)).
