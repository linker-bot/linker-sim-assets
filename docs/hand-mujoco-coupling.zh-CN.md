# 全部灵巧手的 MuJoCo 联动修复

[O6 2 ms transfer tests on other hands / 其他手型复测](other-hands-2ms-validation.md)

> **O6 更新：** 单手 coupled 文件和 o6_coupled 已采用
> [2 ms 位置伺服/接触配置](o6-mujoco-coupling.zh-CN.md)。下文 62.5 µs 数据对 O6
> 属于历史测试，对其他手型仍是当前配置。通用 hand_coupled 仍使用旧的约束配置，
> 包括其中选中的 O6；其他手型的 2 ms 迁移结果见上方复测报告。

排查确认：除 O6 外，**L6、L20 Lite、L25、L30 的左右手也存在被动关节跟随异常**。
现在五种手型均提供可直接加载的 `coupled/left.xml` 和 `coupled/right.xml`，位置在
`components/hands/linkerhand_<型号>/coupled/`。全部十个文件的链接见
[英文说明](hand-mujoco-coupling.md)。

## 使用方法

单手加载示例：

```python
from linker_robot_assets import asset_root
import mujoco

path = asset_root() / "components/hands/linkerhand_l25/coupled/right.xml"
model = mujoco.MjModel.from_xml_path(str(path))
data = mujoco.MjData(model)
# 按该型号原有的执行器顺序设置 data.ctrl，再调用 mj_step。
```

单手文件保留位置执行器，不需要额外联动代码、被动电机或每步赋值 `qpos`。
各型号主动关节数量不同，不能统一传入 O6 的六维命令：

| 型号 | 主动关节数 | 被动关节数 |
|---|---:|---:|
| L6 / O6 | 6 | 5 |
| L20 Lite | 10 | 10 |
| L25 | 16 | 5 |
| L30 | 17 | 4 |

L20 Lite 每根手指有两个被动关节；L30 的拇指 DIP 是主动关节，保持独立驱动，
仅另外四指 DIP 与 PIP 联动。所有原有联动比例和左右手差异均保留。

在组合场景的手写 `recipe.yaml` 中使用：

```yaml
mujoco:
  coupling_profile: hand_coupled
```

重新运行 composer 后，场景内上述五种手型的联动约束及全局配套步长会写入最终
MJCF。组合模型仍为 **motor 力矩控制**，需要应用原有的位置控制器计算力矩。
原来的 `o6_coupled` 仍只作用于 O6；不选配置的现有场景保持原样。L30 虽未自动
派生 solo unit，但支持通过自定义 recipe 组合生成。

## 排查与验证结果

采用统一协议：目标在 1 秒内增加、保持到第 5 秒；只驱动 equality 的主动端，
目标不超过主动和全部被动关节兼容上限的一半，且最多为 0.6 rad。其他轴保持零。
以下为 MuJoCo 3.8.0 数据；修复前后使用相同输入与物理参数：

| 型号 | 原始模型第 5 秒最大误差，左 / 右（rad） | 修复后全程最大误差，左 / 右（rad） |
|---|---|---|
| L6 | 0.578584 / 0.578592 | 0.000002755 / 0.000002755 |
| O6 | 0.521750 / 0.521754 | 0.000170615 / 0.000170663 |
| L20 Lite | 0.763226 / 0.787545 | 0.000108111 / 0.000120527 |
| L25 | 0.601786 / 0.599422 | 0.000326305 / 0.000261586 |
| L30 | 0.590923 / 0.590856 | 0.000303702 / 0.000293091 |

十个修复模型均无警告，所有 equality 启用；误差在每个物理步测量。L6 在该运动中
有一处原有自接触，但关闭接触后原始联动仍失败；其余八个模型在此协议中无接触。
修复没有修改碰撞策略。

统一求解配置为 `timestep=0.0000625`、joint equality `solref="0.000125 1"`、
`solimp="0.9999 0.9999 0.001 0.5 2"`。L6 原有的 damping=5、kp/kv=20/2，
以及其他手型的 damping=4、kp/kv=200/0 均保留；摩擦、armature、惯量、限位、
执行器及联动比例也保持原值。

积分步数是原来的 **32 倍**。控制周期要按秒处理，并避免应用重新覆盖为 2 ms
步长。该配置是数值仿真修复，**不是经过实物标定的物理参数**。测试覆盖中等幅度
的缓慢输入、保持与开合，以及 O6 先前的阶跃测试；完整范围、任意抓取载荷、
各型号突变输入、接触力收敛及自锁性能仍需单独验证。

## 维护与复测

```bash
python -m linker_robot_assets.hand_coupling
python -m linker_robot_assets.hand_coupling --check-drift
python -m linker_robot_assets.validate_hand_coupling --source --output source.json
python -m linker_robot_assets.validate_hand_coupling --output coupled.json
python -m linker_robot_assets.validate_hand_coupling --cycle --output cycles.json
PYTHONPATH=src python -m unittest discover -s tests -p 'test_*coupling.py' -v
```

`--source` 检查原始模型，当前应返回失败；新配置应通过。JSON 保存输入、每对
关节末态、全程峰值、警告、模型哈希和 MuJoCo 版本。测试还覆盖全部十个新生成
的单手组合模型，以及混合手型场景和 O6 兼容入口。
