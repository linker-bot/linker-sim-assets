# O6 mechanical coupling at 2 ms

The optional `o6_coupled` profile fixes passive following with native MuJoCo
joint equalities and position servos, without reducing the original 2 ms step.
It addresses [#17](https://github.com/linker-bot/linker-sim-assets/issues/17).
These are numerical tuning parameters, not hardware calibration.

Load the generated [left](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/left.xml)
or [right](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/right.xml) model:

```python
import mujoco
import numpy as np
from linker_robot_assets import asset_root

path = asset_root() / "components/hands/linkerhand_o6/coupled/right.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
target = np.array([0, .25, .6, .6, .6, .6])  # radians
for _ in range(round(8 / model.opt.timestep)):
    data.ctrl[:] = target * min(data.time, 1.)
    mujoco.mj_step(model, data)
```

No passive-joint motor, qpos projection, gravity compensation or coupling callback
is needed. The step remains 0.002 s: 500 integration steps per simulated second.

| Setting | O6 profile |
|---|---|
| Position actuator kp / kv | 600 / 4.5 |
| Equality solref | -10000000 -63000 |
| Equality solimp | 0.9999 0.9999 0.001 0.5 2 |
| Contact solref | 0.004 1 |
| Contact solimp | 0.95 0.95 0.001 0.5 2 |
| Joint-limit solref / solimp | 0.000125 1 / 0.9999 0.9999 0.001 0.5 2 |
| Solver | implicitfast, Newton, 100 iterations, tolerance=1e-8 |
| Contact options | elliptic, impratio=10, nativeccd disabled, margin=gap=0 |
| Actuator force caps | yaw 1 Nm; thumb left 4 / right 3.8148 Nm; other fingers 2.55 Nm |

Joint damping=4, frictionloss=0.5, armature=3.4707e-08, masses, inertias, limits,
geometry and linkage ratios retain their source values. One additional collision
exclusion matches the tested DemoGrasp profile: palm versus thumb metacarpals,
where the convex palm collider fills the thumb-base clearance. Other collision
exclusions and collision masks are retained.

For hand-authored composed scenes, select:

```yaml
mujoco:
  coupling_profile: o6_coupled
```

Run `python -m linker_robot_assets.composer.compose <recipe-directory>`.
**This profile retains O6 native position actuators in composed scenes:** O6
`data.ctrl` entries are angles, including in a mixed arm/hand model. Other
components retain their existing torque motors. This supersedes the earlier
version of this PR, which converted O6 to torque motors and required 62.5 us.
External PD damping on torque motors is not equivalent to implicit native servo
damping at 2 ms. Recipes without this profile retain their previous output.
The scene-wide timestep defaults to 2 ms; explicit smaller steps are accepted,
and larger steps are rejected. Solver options affect the whole composed scene.

Set compatible contact parameters on objects as well. MuJoCo combines geom
parameters using priority/solmix, and explicit contact pairs override them.
A higher-priority object or pair can override the hand's new contact impedance.
The regression fixture sets both the fingertip and plate to 0.95.

**Validation and limits.** Tests use only actuator commands and mj_step after
initialization. Empty-hand motion, hold/open/close, standalone and composed
hands, source physics preservation and profile scoping are checked. The shipped
contact validator builds four fixed-base thumb/index plate fixtures from the
current assets using recorded DemoGrasp poses; thumb yaw is fixed and contacts
are frictionless. Both contact and release/recontact run for 30 s and are judged
on the final 5 s, not transient error or penetration peaks.

MuJoCo 3.14.0 nominal fixtures retain approximately 25–31 N of force. Contact
force standard deviation is below 1e-5 N, versus 4.15–6.05 N with contact
solimp=0.999 and otherwise identical tuning. Detailed reproducible results are
in [o6-coupling-validation.json](o6-coupling-validation.json).
Earlier exploratory tests also covered 60 s holds, reduced loads, step commands
and planes tilted by ±10 degrees on MuJoCo 3.8.0 and 3.14.0. Those tests motivated
0.95 rather than 0.97, which could oscillate again after a contact perturbation.

The tradeoff is finite static contact penetration. The 3.14 fixtures show
about 0.33–0.96 mm; the same fixed fixtures in 3.8 reach about 2.5 mm.
Do not confuse this with transient penetration. Full multifinger frictional
grasping and arbitrary moving objects remain outside the validated scope.
No hardware force accuracy or self-locking is claimed.

```bash
python -m linker_robot_assets.o6_coupling
python -m linker_robot_assets.o6_coupling --check-drift
python -m linker_robot_assets.validate_hand_coupling --hand o6 --output motion.json
python -m linker_robot_assets.validate_o6_contact --output contact.json
python -m linker_robot_assets.validate_o6_contact --recontact --output recontact.json
PYTHONPATH=src python -m unittest discover -s tests -p test_o6_coupling.py -v
```

[Other hands](hand-mujoco-coupling.md) currently retain the separate 62.5 us
`hand_coupled` profile pending their own 2 ms tests. Selecting `hand_coupled`
uses that legacy equality-only recipe for all hands, including O6; use
`o6_coupled` for the new O6 position-servo/contact recipe.
