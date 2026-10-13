# Other hands: O6 2 ms parameter transfer / 其他手型复测

O6 PR 的 2 ms 配置已提交。随后对 L6、L20 Lite、L25、L30 的左右手进行了
140 条测试（MuJoCo 3.8.0 / 3.14.0，各 32 条整手运动和 38 条接触测试）。
结果表明该参数组不能直接作为所有手型的通用配置，因此保留其他手型现有的
62.5 µs coupled 文件；这些实验没有改写它们的正式模型。

Transferred settings: timestep=0.002 s, native position kp=600/kv=4.5,
implicitfast/Newton, equality solref=-1e7 -63000 and solimp=0.9999,
contact solref=.004 1 and solimp=.95 .95 .001 .5 2, and the O6 solver/limit
settings. Source damping, frictionloss, armature, geometry, inertias and
coupling ratios are preserved. The O6-specific palm/thumb exclusion is not
applied to other hands. Positive actuator effort caps come from each source
URDF, rather than copying O6 torque limits.

| 手型 | 两版本一致的主要结果 | 是否直接采用 O6 参数 |
|---|---|---|
| L6 | 整手 8 个场景均出现早期 QACC 警告；单拇指也异常，其他四指接触可稳定 | 否 |
| L20 Lite | 普通保持/开合/50 g 可稳定；近限位及拇指受阻持续振荡 | 否 |
| L25 | 整手运动可稳定；右手各指受阻仍持续振荡，左手的行为也依赖版本 | 否 |
| L30 | 普通运动和指尖接触总体稳定；附加 50 g 后持续振荡 | 否 |

L6 在关闭全部接触后仍于约 0.054 / 0.058 s 出现 QACC 警告，说明不能仅靠排除
某个碰撞对解决本轮参数迁移失败。

L20 Lite 的近限位末段速度约 3.2–3.7 rad/s；拇指接触力标准差约 44–46 N。
L25 右手接触力标准差约 27–32 N。L30 的 50 g 模式末段速度约 0.21–0.25 rad/s，
联动残差约 0.010–0.012 rad。以上均为保持到第 25–30 秒仍存在的问题，不是启动瞬态。

For reproducibility, the JSON also records an auxiliary strict settling gate
(force standard deviation < 0.1 N). Counts using that gate follow below.
Small jitter just above that line is not called large instability: for example,
L30 contact on 3.8 has about 0.1 N force standard deviation on a 20 N mean.
Its recommendation is instead limited by the persistent 50 g load oscillation.

| Hand | Motion gate, 3.14 / 3.8 | Contact gate, 3.14 / 3.8 |
|---|---|---|
| l6 | 0/8 / 0/8 | 8/10 / 8/10 |
| l20lite | 6/8 / 6/8 | 7/10 / 3/10 |
| l25 | 8/8 / 8/8 | 4/10 / 5/10 |
| l30 | 6/8 / 6/8 | 8/8 / 2/8 |

Each whole-hand case is a 30 s hold, close/reopen, 98% compatible range, or
rigid 50 g payload at the index distal link (local [.006,0,.035] m).
Every equality-parent chain is separately tested against a frictionless plane.
Unrelated joints are fixed at zero; the plane is tangent to the distal collision
mesh at a compatible intermediate flexion pose, facing the closing velocity.
L20 Lite therefore retains both passive joints per chain. L30's independently
actuated thumb is not counted as a passive chain. These fixtures are geometric
numerical stress tests, not a physical force measurement or full grasp.

Some assets contain unusual effort metadata: L20 Lite includes 100 Nm entries;
L30 left thumb has three zero-effort entries. Nonpositive values follow the
existing composer's uncapped-actuator behavior and are recorded explicitly.
Some fixture forces are hundreds of Newtons; this does not establish real-hand
force capability. Caps and absolute force accuracy require separate validation.

The [JSON results](other-hands-2ms-validation.json) include each scenario,
completed/aborted state, warnings, actual caps, steady measurements and hashes.
No new timestep or physical parameters have been adopted for these four families.

```bash
# Run from the repository with its authoring dependencies installed.
# These diagnostics intentionally exit nonzero for the failed candidates.
PYTHONPATH=src python scripts/probe_hand_2ms.py --kind motion --output-dir /tmp/hand-2ms
PYTHONPATH=src python scripts/probe_hand_2ms.py --kind contact --output-dir /tmp/hand-2ms
# Optionally limit a diagnostic to one family, e.g. --hand l30.
```

[O6 2 ms profile](o6-mujoco-coupling.md) / [O6 中文](o6-mujoco-coupling.zh-CN.md)
