# Passive-joint coupling across the hand assets

[O6 2 ms transfer tests on other hands / 其他手型复测](other-hands-2ms-validation.md)

> **O6 update:** its standalone coupled files and `o6_coupled` now use the
> [2 ms position-servo/contact profile](o6-mujoco-coupling.md). The 62.5 us
> shared-profile audit below is historical for O6 and remains the current
> configuration for the other hands. `hand_coupled` still uses that legacy
> equality-only configuration for every selected hand, including O6.

All five shipped hand families reproduce substantial passive-joint lag with
their original MuJoCo solver settings. The opt-in `hand_coupled` profile fixes
the audited motion for **both sides of L6, O6, L20 Lite, L25 and L30**. It changes
only the timestep and the selected mechanical equality solver parameters.

## Models and usage

| Hand | Active joints | Passive joints | Standalone MJCFs |
|---|---:|---:|---|
| L6 | 6 | 5 | [left](../src/linker_robot_assets/assets/components/hands/linkerhand_l6/coupled/left.xml), [right](../src/linker_robot_assets/assets/components/hands/linkerhand_l6/coupled/right.xml) |
| O6 | 6 | 5 | [left](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/left.xml), [right](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/right.xml) |
| L20 Lite | 10 | 10 | [left](../src/linker_robot_assets/assets/components/hands/linkerhand_l20lite/coupled/left.xml), [right](../src/linker_robot_assets/assets/components/hands/linkerhand_l20lite/coupled/right.xml) |
| L25 | 16 | 5 | [left](../src/linker_robot_assets/assets/components/hands/linkerhand_l25/coupled/left.xml), [right](../src/linker_robot_assets/assets/components/hands/linkerhand_l25/coupled/right.xml) |
| L30 | 17 | 4 | [left](../src/linker_robot_assets/assets/components/hands/linkerhand_l30/coupled/left.xml), [right](../src/linker_robot_assets/assets/components/hands/linkerhand_l30/coupled/right.xml) |

Load these with `mujoco.MjModel.from_xml_path`, for example:

```python
from linker_robot_assets import asset_root
import mujoco

path = asset_root() / "components/hands/linkerhand_l25/coupled/right.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
# Set data.ctrl to this hand's position targets, then call mj_step as usual.
```

The component files keep their **position actuators**. Joint names, actuator
order, limits and gains come from that hand's source model; do not reuse O6's
six-element control vector for the other hand families. No passive qpos writes,
extra motors, or coupling callback is needed. L20 Lite's two followers per
finger and side-specific ratios are preserved. L30's thumb DIP is **active**;
the profile only couples its four other DIP joints to their PIP joints.

For a composed unit/workstation, add this to a hand-authored recipe:

```yaml
mujoco:
  coupling_profile: hand_coupled
```

Regenerate with `python -m linker_robot_assets.composer.compose <directory>`.
The profile selects the five supported hand component types, including mixed
hand scenes, and writes the required scene-wide timestep into the final MJCF.
Other components' equalities are preserved. Composed files keep their **torque
motors**; the application must continue converting desired positions to torque
commands with its controller. This setting does not change actuator semantics.
L30 has no auto-derived solo unit, but can be composed from a custom recipe in
the same way as the other hand components.

Existing mount and component-compiler compatibility rules still apply. The
mixed-scene regression combines O6, L25 and L20 Lite; the coupling profile does
not resolve unrelated conflicts between component URDF compiler declarations.

The O6-only `mujoco.coupling_profile: o6_coupled` still selects only O6 in mixed
scenes. Existing recipes without either profile retain their current output.
`mujoco.timestep` can explicitly select a smaller timestep; with either coupling
profile, larger values than 0.0000625 s are rejected.

## Matched audit results

The comparison starts each model at its source default pose, ramps the equality
parent targets for one second, and holds to five simulated seconds. For each
parent, the target is half its compatible upper limit (including all follower
limits), capped at 0.6 rad. Unrelated axes remain at zero. Source and corrected
models use identical commands, gravity, contacts and physical parameters. This
range-aware protocol differs from the earlier [O6-only reproduction](o6-mujoco-coupling.md).

MuJoCo 3.8.0 results, with coupling residuals measured at **every physics step**:

| Hand | Source final maximum error, left / right (rad) | Corrected full-trajectory maximum, left / right (rad) |
|---|---|---|
| L6 | 0.578584 / 0.578592 | 0.000002755 / 0.000002755 |
| O6 | 0.521750 / 0.521754 | 0.000170615 / 0.000170663 |
| L20 Lite | 0.763226 / 0.787545 | 0.000108111 / 0.000120527 |
| L25 | 0.601786 / 0.599422 | 0.000326305 / 0.000261586 |
| L30 | 0.590923 / 0.590856 | 0.000303702 / 0.000293091 |

All ten corrected ramp/hold runs finish without warnings. All constraints
remain enabled. L6 has one existing self-contact in these motions; its source
still fails when contacts are disabled in a separate causal control. The other
eight models have no contacts in this audit. Collision settings are not changed.

The common settings are:

```xml
<option timestep="0.0000625"/>
<!-- On each selected hand's declared joint equality: -->
<!-- solref="0.000125 1" solimp="0.9999 0.9999 0.001 0.5 2" -->
```

The original hand dynamics are retained, including L6 damping=5 and its
kp/kv=20/2 servos, versus damping=4 and kp/kv=200/0 in the other component
MJCFs. The profile preserves frictionloss, armature, masses, inertias, ranges,
ratios, actuators and collision exclusions. No assumption of five passive
joints is applied to L20 Lite or L30.

MuJoCo equalities are soft. Default solver settings permit large errors for
these dynamics; enabling a constraint alone does not enforce an exact angle
ratio. The time constant and timestep must be selected together because default
`refsafe` clamps the former to at least twice the latter. These profiles require
**32 times more integration steps per simulated second**. Keep control intervals
in seconds and do not overwrite the model timestep with the original 2 ms.

These results are numerical motion validation, **not hardware calibration**.
The tests cover moderate ramp/hold and opening/closing, and O6's earlier bounded
step response. They do not certify full-range or arbitrary contact-loaded
grasps, abrupt input on every hand, contact-force convergence or self-locking.
Those need task-specific validation of linkage, stops, forces and backdrive.

## Reproduce and maintain

```bash
# Regenerate all ten directly loadable files, or add --hand l25 for one family.
python -m linker_robot_assets.hand_coupling
python -m linker_robot_assets.hand_coupling --check-drift

# Five-second audit; JSON contains targets, every-pair final angles, maxima,
# warnings, model hashes and MuJoCo version. Original models fail as expected.
python -m linker_robot_assets.validate_hand_coupling --source --output source.json
python -m linker_robot_assets.validate_hand_coupling --output coupled.json
python -m linker_robot_assets.validate_hand_coupling --cycle --output cycles.json

PYTHONPATH=src python -m unittest discover -s tests -p 'test_*coupling.py' -v
```

`python -m linker_robot_assets.o6_coupling` remains an O6-only compatibility
entry point. Tests verify source failures, all ten generated files, dynamic
open/close cycles on all ten newly composed solo models, mixed-scene selection,
unchanged physics/actuators, and L30's independently actuated thumb. The audit
stops on warnings/resets and checks actual parent motion, so stationary joints
cannot pass solely by having zero equality residual.
