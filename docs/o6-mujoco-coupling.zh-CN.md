# O6 在 MuJoCo 中的机械联动

可选的 `o6_coupled` 配置通过 MuJoCo 的 joint equality 约束，使五个被动
关节跟随主动关节，解决 [#17](https://github.com/linker-bot/linker-sim-assets/issues/17)
描述的空载联动异常。它保留现有默认模型，同时提供可直接加载的
[左手 MJCF](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/left.xml)
和[右手 MJCF](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/right.xml)。

## 客户如何使用

```python
import mujoco
import numpy as np
from linker_robot_assets import asset_root

path = asset_root() / "components/hands/linkerhand_o6/coupled/right.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
# 顺序：拇指侧摆、拇指屈曲、食指、中指、无名指、小指；单位 rad。
target = np.array([0, 0.3, 0.8, 0.8, 0.8, 0.8])
for _ in range(round(5 / model.opt.timestep)):
    data.ctrl[:] = target * min(data.time, 1.0)  # 1 秒内增加到目标值
    mujoco.mj_step(model, data)
```

无需在每步手动赋值被动 `qpos`、增加被动关节电机或编写联动回调。
上述单手文件保留位置执行器，`data.ctrl` 的含义仍为目标角度。

对组合模型，在包含 O6 的手写 `recipe.yaml` 中加入：

```yaml
mujoco:
  coupling_profile: o6_coupled
```

然后运行 `python -m linker_robot_assets.composer.compose <场景目录>`。
生成器会设置场景内所有 O6 的联动约束，并把配套步长写入最终 `workstation.xml`，
其他组件的 equality 保持不变。自动派生的 solo recipe 不应手改；可在新目录创建
自定义 recipe，完整示例见[英文说明](o6-mujoco-coupling.md)。

组合模型仍使用 **motor 力矩执行器**（gear=1），`data.ctrl` 应由应用的位置控制器
计算为力矩；选择联动配置不会把它改成角度输入。单手组件与组合模型的控制含义
必须区分。

## 参数、结果与限制

| 参数 | 原始 O6 | 可选联动配置 |
|---|---|---|
| timestep / s | 0.002 | 0.0000625 |
| equality solref | 0.02 1 | 0.000125 1 |
| equality solimp | 0.9 0.95 0.001 0.5 2 | 0.9999 0.9999 0.001 0.5 2 |

原有 `damping=4`、`frictionloss=0.5`、`armature=3.4707e-08`、连杆惯量、关节限位、
执行器参数、接触配置与联动比例均保留。MuJoCo 的 equality 是软约束；启用只表示
参与求解，不保证任意条件下严格满足比例关系。默认 `refsafe` 会将有效时间常数
限制到至少 `2×timestep`，因此约束参数与步长需要配套。

在 MuJoCo 3.3.7 和 3.8.0 中，原始右手用阶跃位置命令运行 5 秒后，食指
MCP≈0.799944 rad，DIP 应为≈0.711950 rad，实际只有≈-0.021209 rad。
采用新配置及上面的 1 秒缓慢输入后，MCP≈0.800012 rad、DIP≈0.712011 rad，
五条约束全程最大误差约 0.000228 rad。直接阶跃输入仍有约 0.0384 rad 的瞬态峰值。

时间步长影响整个场景，需要原来 **32 倍积分步数**；控制周期应继续按秒计算。
例如 2 ms 控制周期内应执行 32 个物理步。应用若覆盖 `model.opt.timestep`，必须
保持兼容设置。recipe 允许显式设置更小步长，选中本配置时会拒绝更大的步长。
不启用联动配置时，也可单独声明 `mujoco: {timestep: 0.001}`。

这是经过数值测试的仿真配置，**不是实物标定参数**。当前回归测试覆盖空载开合、
阶跃输入、左右手及组合模型生成，不代表任意抓取、接触载荷均已验证。承载应用
仍需检查联动误差、关节止挡、接触力及反驱行为；本配置没有添加自锁传动或修改
碰撞几何，也不证明接触力已经收敛。

## 维护和测试

两个单手文件由原始 XML 和 metadata 生成；修改源文件后运行：

```bash
python -m linker_robot_assets.o6_coupling
python -m linker_robot_assets.o6_coupling --check-drift
PYTHONPATH=src python -m unittest discover -s tests -p test_o6_coupling.py -v
```

测试仅通过执行器命令与 `mj_step()` 产生运动，并逐步检查五条联动关系、关节越界、
警告及重置。它也验证原始物理参数未改变、组合场景保留配套设置及无效配置会报错。
