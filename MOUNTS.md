# Mounts

Every component is a standalone robot whose root body sits at its own origin. Where another part
attaches, the component has an empty frame `mount_<child>`: the child's root frame goes exactly
there. The parent owns the offset (the flange decides where the hand sits), so a part fits any
parent with the right mount. `tools/assemble.py` puts each part's root body inside the mount
frame named in `assemblies/<name>/assembly.yaml` (with the parent's prefix, e.g.
`arm_mount_flange`).

Lengths in metres, angles in radians. "Pose" is as written in the parent's MJCF, relative to the
body that holds the frame.

| Component | Mount | In body | Pose | Fits |
| --- | --- | --- | --- | --- |
| `arms/ar5_08` (left) | `mount_flange` | `AR5_5_08L_W4C4A6_tcp` | at the tool point; rotated −π/2 about z (written 1.57079633) | `flanges/ar5_08_o6` |
| `arms/ar5_08` (right) | `mount_flange` | `AR5_5_08R_W4C4A6_tcp` | at the tool point; rotated +π/2 about z | `flanges/ar5_08_o6` |
| `flanges/ar5_08_o6` | `mount_hand` | `hand_mount` (0.0225 along z from the flange origin) | 0.002 back along z; rotated π about z (exact) | `hands/o6` left or right |
| `flanges/ar5_08_o6` | `mount_bracket` | `flange_link` | at the flange origin | `brackets/gemini335l_wrist` left or right |
| `brackets/gemini335l_wrist` (left) | `mount_camera` | `camera_mount` | at the camera's seat | `cameras/gemini335l` |
| `brackets/gemini335l_wrist` (right) | `mount_camera` | `camera_mount` | at the camera's seat | `cameras/gemini335l` |
| `brackets/zed2i_column` | `mount_camera` | `camera_base_link` | at the camera's seat on the column | `cameras/zed2i` |

The component frames these mounts join:

- **AR5_08 arm** (`arms/ar5_08`): the base frame is the arm's mounting face, z along J1. The tool
  point `..._tcp` is 0.09 beyond the J6/J7 wrist centre, z pointing out of the wrist (0.8745 above
  the base with all joints at zero). Left and right are mirror images; the flange mount turns the
  other way on each.
- **Flange** (`flanges/ar5_08_o6`): one part for both arms. Its origin is the arm's tool point,
  z pointing away from the arm.
- **O6 hand** (`hands/o6`): the root `{l,r}h_hand_base_link` is the wrist's mounting face, z
  pointing towards the fingertips. On LW-v0 it sits 0.0205 beyond the flange origin.
- **Gemini 335L wrist bracket** (`brackets/gemini335l_wrist`): its origin is the flange origin; its
  camera seat looks along the hand, 20° off its axis. Left and right are mirror images.
- **Cameras** (`cameras/gemini335l`, `cameras/zed2i`): the maker's camera frame, x forward along the
  optical axis, y to the left, z up. The optical frames (`*_optical`, z forward, x right, y down)
  carry the MJCF `<camera>`s with their intrinsics.
- **Support** (`supports/lw_v0`): table and arm column, origin at the column top between the two
  arm bases; it has no mounts. `assemblies/lw_v0/lw_v0.xml` places the units on it.
