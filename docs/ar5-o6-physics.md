# AR5/O6 assembly physics

The six AR5+O6 recipes explicitly select `mjcf_physics: ar5_o6`. The composer
applies [this native configuration](../src/linker_robot_assets/composer/ar5_o6_physics.py)
after assembling components and converting position actuators to torque motors.
Regeneration, rather than editing generated XML, preserves the configuration.
Standalone O6, AR5 without O6, A7/P7+O6 and L6 combinations retain their existing
dynamics and materials. AR5/AR5_08 J6/J7 limits are a separate model correction:
**±pi/3 (±60°)** in both component formats and every affected assembly, including
solo URDFs used by planners. Other joint limits are unchanged.

## Native MJCF values

| Scope | Passive damping (N·m·s/rad) | Armature (kg·m²) | Motor output range (N·m) |
|---|---:|---:|---:|
| AR5 J1–J7 | 0 | 0.01 | ±200 |
| O6 thumb lateral | 0 | 0.001 | ±10 |
| O6 thumb flexion / four MCP flexion joints | 4 | 0.001 | ±60 |
| O6 followers | 0.4 | 0.001 | No independent motor |

Joint frictionloss is zero. Equality ratios remain thumb 1.86 and other fingers
0.89; their solref is `(0.002, 1)` and solimp `(0.999, 0.999, 0.001, 0.5, 2)`.
Every body below the arm root, including flange, bracket, camera and optical
frames, has `gravcomp=1`. World gravity remains `(0, 0, -9.81)` for task objects.
Geometry, mass/inertia, installation transforms and collision exclusions remain
unchanged. These are selected simulation settings, not hardware ratings.

All collision geoms in these assemblies use condim=4, solref `(0.004, 1)`,
margin=0, priority=0 and solmix=1. Hand geoms use friction `(4, 0, 0)` and solimp
`(0.95, 0.95, 0.001, 0.5, 2)`; arm/accessory/base geoms use `(1, 0, 0)` and
`(0.9, 0.95, 0.001, 0.5, 2)`. Visual geoms remain non-collidable. Original robot
geom gap=0 is explicit, including previously inherited zero. The simulator uses
0.0001 m only as the fallback for otherwise unauthored scene assets.

MJCF scene options are timestep=1/480 s, implicitfast, Newton solver,
iterations=100, ls_iterations=50, elliptic cone and impratio=100. Cone/impratio
are scene-wide. These XML options do not select the **Newton collision library**:
MuJoCo's solver named `Newton` is a different concept. A plain MuJoCo consumer
uses its own collision implementation and will not necessarily reproduce the
Newton runtime's multi-point contacts or grasp outcomes.

## Runtime boundary

Generated `workstation.xml` retains torque motors with gear=1, capped torque
controls and force ranges. It does **not** accept angle commands through `ctrl`.
It contains no extra position servo competing with the importing simulator.
The generic manifest `default_gains`/`gain_profiles` remain component metadata;
they are **not** the selected Newton compliant-controller configuration.

The corresponding simulator source is
[PR #64](https://github.com/linker-bot/linker-sim-isaac/pull/64),
`configs/controllers/newton_compliant/{arm_controller,hand_controller}.yaml`
and its compliant-drive settings. Native position outputs there use:

| Group | Kp | Kd |
|---|---:|---:|
| Arm J1–J7 | 16000 | [614.2237548828125, 613.7109985351562, 603.791015625, 600.3331298828125, 604.13671875, 604.7486572265625, 603.66796875] |
| Thumb lateral | 600 | 24 |
| Thumb flexion | 2500 | 40 |
| Four MCP flexion joints | 1250 | 40 |

The hand's virtual advancing torque is separately 3 N·m lateral / 5 N·m flexion,
and the arm reference guard is 199 N·m. Neither is a native asset motor cap.
The simulator supplies virtual-shaft state, smooth references, velocity
feedforward, substep interpolation and reset handling. This asset does not
reimplement them or encode them as unsupported MJCF attributes.

[Simulator PR #66](https://github.com/linker-bot/linker-sim-isaac/pull/66) owns
command frequency **60 Hz**, integration **480 Hz**, collision refresh **240 Hz**,
Newton buffer/work-grid settings, solver/collision determinism, contact matching
and friction gain kf=1000. CPU and CUDA share those defaults. Standard MJCF cannot
encode these Newton-specific execution settings; its timestep represents only
integration. Runtime choices can override XML options.

Task objects are not part of this robot asset. In the corresponding simulator
scene their friction is `(1, 0.005, 0)`, solimp `(0.9, 0.95, 0.001, 0.5, 2)`,
condim=4 and normal solref `(0.004, 1)`. Equal-priority hand/object mixing gives
μ=4, torsion 0.005 m and solimp `(0.925, 0.95, 0.001, 0.5, 2)`. REFSAFE and upstream
friction floors still apply. A task adding objects directly to this MJCF must
supply its own object materials and controller; this file alone is not a grasp
benchmark or a fully calibrated physical model.

URDF remains the kinematic/hardware-oriented companion: the J6/J7 model limit
correction is shared, but it does not encode the assembly's MuJoCo dynamics,
gravity compensation, new native actuator caps or virtual controller. Do not
interpret its legacy effort/dynamics metadata as the Newton selected settings.

## Regeneration and verification

Use an authoring environment with MuJoCo, NumPy, SciPy, trimesh and PyYAML:

```bash
PYTHON=/path/to/python just compose
PYTHON=/path/to/python just test
PYTHON=/path/to/python just check-drift
```

Tests compile and read back the six configured assemblies, check limit coverage
across components and generated models, and verify unselected configurations.
The simulator asset-pin update and final integrated acceptance follow this
asset PR's merge; see [integration #53](https://github.com/linker-bot/linker-sim-isaac/issues/53).
