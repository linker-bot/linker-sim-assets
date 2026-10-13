# O6：保持 2 ms 的 MuJoCo 机械联动

`o6_coupled` 通过原生 joint equality 和位置伺服实现主从关节联动，解决
[#17](https://github.com/linker-bot/linker-sim-assets/issues/17)。无需被动关节电机、
运行时 qpos 投影、重力补偿或联动回调，每秒仍只积分 500 步。
这是数值调参方案，未经实物标定。

直接加载生成的[左手](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/left.xml)
或[右手](../src/linker_robot_assets/assets/components/hands/linkerhand_o6/coupled/right.xml)：

```python
import mujoco
import numpy as np
from linker_robot_assets import asset_root

path = asset_root() / "components/hands/linkerhand_o6/coupled/right.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
target = np.array([0, .25, .6, .6, .6, .6])
for _ in range(round(8 / model.opt.timestep)):
    data.ctrl[:] = target * min(data.time, 1.)
    mujoco.mj_step(model, data)
```

| 参数 | O6 配置 |
|---|---|
| timestep | 0.002 s |
| 主动位置执行器 | kp=600，kv=4.5 |
| equality solref | -10000000 -63000 |
| equality solimp | 0.9999 0.9999 0.001 0.5 2 |
| contact solref | 0.004 1 |
| contact solimp | 0.95 0.95 0.001 0.5 2 |
| limit solref / solimp | 0.000125 1 / 0.9999 0.9999 0.001 0.5 2 |

其他求解参数：implicitfast、Newton、iterations=100、tolerance=1e-8、elliptic、
impratio=10、nativeccd=disable、接触 margin=gap=0。
力矩上限：侧摆 1 Nm，拇指屈曲左 4 / 右 3.8148 Nm，其余手指 2.55 Nm。

关节 damping=4、frictionloss=0.5、armature=3.4707e-08、质量、惯量、几何、限位和
联动比例保持源模型值。新增一对碰撞豁免：掌部与拇指
metacarpals，避免掌部凸碰撞体填满真实拇指根部间隙。其他豁免和碰撞掩码保留。

组合场景的手写 recipe.yaml 可声明：

```yaml
mujoco:
  coupling_profile: o6_coupled
```

再运行 `python -m linker_robot_assets.composer.compose <场景目录>`。
**此配置在组合场景中也保留 O6 原生位置执行器，O6 的 data.ctrl 输入单位为角度。**
其他组件仍使用原有力矩执行器；混合机械臂/手模型应分别处理对应的输入。
2 ms 下外部 PD 的
阻尼不等价于原生位置伺服参与隐式积分的阻尼。未选择配置的现有 recipe 输出不变。
场景默认步长为 2 ms，可显式缩小，超过 2 ms 会报错；求解器选项影响整个场景。

接触物体也应设置兼容的参数。MuJoCo 会按 priority/solmix 混合两个 geom 的参数，
显式 pair 可以覆盖它们；更高优先级物体可能覆盖手上的 solimp。需要使用这组接触
参数时，应确保双方或显式 pair 的配置让 0.95 实际生效。

接触配置允许有限的静态穿透，需要结合应用中的物体和载荷检查其适用性。
这些数值参数不代表实物力精度，也不添加自锁特性。

```bash
python -m linker_robot_assets.o6_coupling
python -m linker_robot_assets.o6_coupling --check-drift
python -m linker_robot_assets.validate_hand_coupling --hand o6 --output motion.json
python -m linker_robot_assets.validate_o6_contact --output contact.json
python -m linker_robot_assets.validate_o6_contact --recontact --output recontact.json
PYTHONPATH=src python -m unittest discover -s tests -p test_o6_coupling.py -v
```

[其他手型](hand-mujoco-coupling.zh-CN.md) 使用 62.5 µs 配置。通用 hand_coupled
对所有选中手型（包括 O6）使用该约束配置；
需要 O6 新位置伺服/接触方案时选择 o6_coupled。
