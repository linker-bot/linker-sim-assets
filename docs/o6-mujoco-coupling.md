# O6 mechanical coupling in MuJoCo

The optional `o6_coupled` numerical profile makes the five passive joints follow
the six actively controlled O6 coordinates through MuJoCo joint equalities.
It addresses [issue #17](https://github.com/linker-bot/linker-sim-assets/issues/17).
Existing component, unit and workstation defaults remain compatible.

## Load a standalone hand

The package includes complete, directly loadable position-controlled models:

- [Left hand](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/left.xml)
- [Right hand](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/right.xml)

```python
import mujoco
import numpy as np
from linker_robot_assets import asset_root

path = asset_root() / "components/hands/linkerhand_o6/coupled/right.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
# Actuator order: thumb yaw, thumb pitch, index, middle, ring, pinky.
target = np.array([0, 0.3, 0.8, 0.8, 0.8, 0.8])  # radians
for _ in range(round(5 / model.opt.timestep)):
    data.ctrl[:] = target * min(data.time / 1.0, 1.0)  # one-second ramp
    mujoco.mj_step(model, data)
```

No passive qpos assignment, extra distal motor or per-step coupling code is
needed. Actuator targets and mechanical joint coordinates are distinct: contact
may prevent a finger reaching its target while its actual proximal/distal
coordinates should still satisfy their coupling relationship.

## Select the profile in a composed scene

Add this to a hand-authored unit/workstation `recipe.yaml` containing O6:

```yaml
mujoco:
  coupling_profile: o6_coupled
```

Then regenerate with `python -m linker_robot_assets.composer.compose <directory>`.
The composer applies the profile to **all O6 hands** in that recipe, preserves
other components' equalities, and writes the required **scene-wide** timestep
into `workstation.xml`. Do not edit auto-derived solo recipes: for a custom solo
scene, create a new directory with this complete recipe instead:

```yaml
schema_version: 1
name: o6_r_coupled
components:
  hand:
    component: hands/linkerhand_o6
    variant: right
freeze_base: hand
mujoco:
  coupling_profile: o6_coupled
```

Pass `--assets-root <asset_root>` if the new directory is outside the usual
`assets/units/` or `assets/workstations/` layout. The recipe and generated MJCF
are fingerprinted by the existing manifest and `--check-drift` workflow.

Composed models keep their existing **torque motors**: `data.ctrl` contains
torque commands (gear=1), not joint-angle targets. Continue using the application's
position controller to produce those commands. Selecting this profile does not
change actuator semantics, gains or effort limits. The standalone files above
retain their source **position actuators**, for which `data.ctrl` is in radians.

Recipes can also set a timestep without a coupling profile:

```yaml
mujoco:
  timestep: 0.001
```

Component `<option>` settings are intentionally not merged: a timestep affects
the entire scene and must be selected at scene level. With `o6_coupled`, an
explicit timestep may be smaller than 0.0000625 s, but a larger value is rejected.
A profile selection without an O6 component, an unknown profile or an invalid
timestep is also rejected instead of silently ignoring the request.

## Why these settings

| Setting | Source O6 | `o6_coupled` |
|---|---|---|
| Integration timestep / s | 0.002 | 0.0000625 |
| Joint equality solref | 0.02 1 | 0.000125 1 |
| Joint equality solimp | 0.9 0.95 0.001 0.5 2 | 0.9999 0.9999 0.001 0.5 2 |

The profile retains damping=4, frictionloss=0.5, armature=3.4707e-08, link
inertias, joint ranges, contact settings and coupling ratios. Four fingers use
`q_DIP = 0.89*q_MCP`; thumb ratios retain their side-specific source values.

The original model reproduces the reported failure in MuJoCo 3.3.7 and 3.8.0:
after five seconds at `[0, .3, .8, .8, .8, .8]`, right index MCP is 0.799944 rad,
expected DIP is 0.711950 rad, and actual DIP is -0.021209 rad, despite all five
equalities being active, no contacts and no simulation warnings.

Joint equality constraints are soft. Their force response depends on solver
parameters as well as the body's dynamics. Setting a smaller `solref` alone
at the original timestep is insufficient: default `refsafe` limits its time
constant to at least twice the integration timestep. Merely increasing solver
iterations, removing gravity/contact, or removing passive friction did not fix
the original case. Removing passive damping produced unstable transients.

With the profile and the one-second ramp above, both tested MuJoCo versions
reach index MCP=0.800012 rad and DIP=0.712011 rad at five seconds; maximum
coupling error across all five joints is 0.000228 rad. An immediate step still
has a transient peak near 0.0384 rad: this remains a finite-compliance constraint.

The timestep requires **32 times more integration steps** per simulated second.
Keep command/observation intervals in seconds; e.g. a 2 ms control interval now
contains 32 physics steps. Applications that override `model.opt.timestep`
must preserve a compatible value. No exact wall-clock slowdown is promised.

These are numerical simulation settings, **not hardware-calibrated damping,
friction, inertia or transmission parameters**. The shipped regression covers
empty-hand motion and profile composition, not all grasp/contact loads. Loaded
use needs separate validation of coupling error, joint stops, forces and
transmission backdrive. This profile does not add self-locking or change contact
geometry, and it does not establish force convergence or hardware accuracy.

## Regeneration and verification

The two standalone files are generated from the original O6 XMLs and metadata;
do not edit the copies by hand. After changing those sources:

```bash
python -m linker_robot_assets.o6_coupling
python -m linker_robot_assets.o6_coupling --check-drift
PYTHONPATH=src python -m unittest discover -s tests -p test_o6_coupling.py -v
```

The regression runs actual `mj_step()` dynamics for both hands and newly
composed solo units. It checks opening and closing, all five coupling residuals
at every physics step, real active/passive motion, warning/reset behavior and
joint-limit excursions. A separate step-input test bounds the transient.
Static checks cover source-parameter preservation, full bimanual composition,
profile scoping, invalid settings and generated-file drift.
