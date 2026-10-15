# Flanges and camera assemblies

Language: [English](camera-assemblies.md) | [中文](camera-assemblies.zh-CN.md)

`ar5_08_l6_l/r` and `ar5_08_o6_l/r` include a shared 44.5 g flange by default.
`ar5_08_l6_gemini335l_l/r` and `ar5_08_o6_gemini335l_l/r` add the 29 g bracket
and 133 g Gemini 335L camera: 206.5 g total per wrist, or 162 g above the flange-only
unit. Select each arm's unit independently for no/left/right/both cameras.
There are no bracket-only units. Installing a physical camera is independent of
enabling image rendering. Adding attachments preserves the selected hand component's
joint names, limits, mimic and actuator order.

## Mounts and inertia

The components are `attachments/ar5_08_l6_o6_flange`,
`attachments/ar5_08_gemini335l_bracket` (left/right variants), and
`sensors/orbbec_gemini335l`. Each has separate visual and collision geometry.
The source CAD is in millimetres; committed meshes are in metres. Provenance files
record source hashes, baked transforms, known masses and inertia approximations.

The large flange face seats at arm `tool0`; its 2 mm pilot enters the arm recess.
The hand seat is +22.5 mm along tool0 Z. Both O6 1014_v4 seating planes are
+2 mm from their roots, so both root translations are 20.5 mm. The old left
22.5 mm root translation no longer applies. L6 gets the same seating distance, with a +2° X correction
for the right hand's tilted source seating plane. Mounting preserves the selected hand component's internal joint frames;
the O6 component revision itself is described below. These are physical assembly corrections, not new task TCP offsets.

At standard mechanical zero, the specified flange holes are above and the camera
is below, looking toward the hand underside. The flange clocking is left −90°,
right +90°; the hand's net around-axis orientation remains left +90°, right −90°.
The manifest exposes actual fixed links for `flange:hand_mount`,
`bracket:camera_mount`, and `camera:depth_optical`, `camera:rgb_optical`,
`camera:right_ir_optical`. No mount relies on the composer's unsupported generic
`meta.yaml` frame-offset semantics.

Flange and bracket inertia use the known mass with uniform CAD density. Camera
inertia uses its known mass and a shell bounding-box estimate. Neither is measured
hardware inertia. Tiny fixed-frame masses are subtracted from the physical body,
so they do not silently add payload mass. Gravity and control gains are consumer
configuration; adding mass does not enable gravity.

## O6 thumb motion restriction

The hardware owner confirmed the previous assembly. The 1014 hand retains these
attachments, but its complete trajectory still needs renewed hardware confirmation.
With this Gemini 335L bracket, some sideways thumb motions while straight hit the
camera. Bending must be coordinated with sideways rotation. This is a restriction
of the current physical assembly, not a collision-filter defect.

The 1014 joint limits are retained and thumb-camera contact remains enabled.
Do not infer safety from independent joint limits or assume that bending unlocks
the entire sideways range. Hand geometry is mirrored; attachment approximations
still require separate checks on both sides. Check the complete coupled
trajectory, including mimic joints, fingers, camera and bracket. The regression
tests contain colliding and collision-free examples; their angles are not a task
controller, a general safe-region table, or a hardware safety guarantee. Reassess
this note when the hardware/asset revision changes.

The camera collider follows the CAD hull, retaining slanted corners: a bounding
box incorrectly collides at the normal open pose. Flange and bracket colliders use
24 angular CAD sectors, preserving the central recess but conservatively filling
some fastener holes. Only named installation seams are excluded. Consumers must
also enable articulation self-collision.

## Optical reference and top module

Gemini `camera_link` follows the nominal vendor depth/left-IR origin (X forward,
Y left, Z up); optical frames use X right, Y down, Z forward. Right IR is 95 mm
from left IR, and RGB is nominally 23.75 mm to its right with the vendor's small
rotation. These are nominal CAD/vendor references, not device calibration.
Replace intrinsics/extrinsics with device/profile calibration when available.
The vendor reference revision and geometry registration are in `provenance.json`;
vendor meshes are not redistributed.

`sensors/workstation_zed2i` is extracted from the existing bench asset. The static
`workstation_zed2i` unit allows a scene to load it alongside independent arm units.
Five existing bench workstations retain the same visual geometry and top-camera
installation through an explicit recipe component. They remain single composed
workstations; the static sensor unit does not merge separately loaded arms.

The top stand's old COM/inertia was in a frame preceding a +90° X rotation.
Rotating both into its actual mesh frame places the COM within 11 μm of the
uniform-density CAD calculation. The camera's apparently large local COM is
consistent with its offset mesh and is retained. Original total CAD masses are
retained; these are not newly weighed ZED hardware masses. The two nominal optical
frames follow measured lens-cover centers (120 mm baseline), offset 3 mm outward
to avoid an opaque cover. They are a rendering approximation, not lens calibration.

## Regeneration and checks

Run the wrist generator with the versions recorded in provenance, then recompose
affected units/workstations and run `just test`:

```bash
uv run --no-project --with pyyaml --with trimesh==4.11.1 --with scipy==1.17.0 --with numpy==2.3.1 \
  python scripts/build_wrist_attachments.py
PYTHON=/path/to/authoring/python just test
```

`--source-dir` optionally re-ingests the six hash-checked original CAD files.
Normal regeneration uses committed normalized meshes. The tests compile every
asset and check drift, frame parity, mass/actuation, optical baselines, and real
MuJoCo contact on both prohibited and feasible thumb routes. Isaac backend,
camera-following, and planning coverage are validated in the simulator repository.

## O6 1014_v4 migration

The 1014_v4 hand has mirrored kinematics and geometry. Both palm seating planes are
2 mm above the hand root; AR5_08 root translation is now 20.5 mm on both sides,
replacing the old left/right 22.5/20.5 mm pair. This follows the delivered meshes
and existing flange datum, not a new hardware measurement. Other left-hand recipes
compensate 2 mm along local hand Z while retaining their own seating plane/orientation.

The hardware mass is adopted as requested: palm 237.4 g, thumb 64.02 g, other fingers
68.56 g, totaling 369.98 g/hand. Delivered COM/right inertia tensors are used; left
ixy/iyz are reflected correctly. Numerical validity and coordinate checks are not
independent inertia measurements. PR #18 component kp=600/kv=4.5, force caps, passive
dynamics and coupling settings remain unchanged. Composed models retain torque motors
and application controller gains are not retuned. Thumb yaw limit is 1.36 rad on both
sides, with thumb mimic 1.86. The new pitch frame is shared by URDF and MJCF.

Left Y reflection is baked into meshes with corrected winding. Baseline right simplified
colliders are transformed and mirrored; the changed thumb metacarpal uses a new convex
hull. Each hand has 8,390 collision triangles versus 242,886 in the delivery. Hulls fill
concavities and do not represent exact CAD contact. The old right pitch=0.57/yaw=1.2
route now hits the camera. Regression examples bend to pitch=0.57 before yaw=0.45 on
both sides and still check straight-thumb yaw=0.3 contact; these are not safe-angle tables.
The new structure has not undergone renewed physical trajectory confirmation.

All six units/five workstations are regenerated; older tasks must pin their previous
asset revision. The component `provenance_1014.json` records source/output hashes and
coordinate transforms. Reimport with `scripts/build_o6_1014.py --source-dir
<1014-delivery> --baseline-dir <a8c0d475-o6-component>`, recompose affected recipes,
and run `just test`. Both source directories must be unmodified.
