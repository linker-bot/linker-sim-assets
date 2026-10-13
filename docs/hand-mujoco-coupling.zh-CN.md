# 全部灵巧手的 MuJoCo 联动修复

五种手型均提供可直接加载的 `coupled/left.xml` 和 `coupled/right.xml`，位置在
`components/hands/linkerhand_<型号>/coupled/`。全部十个文件的链接见
[英文说明](hand-mujoco-coupling.md)。

O6 单手 coupled 文件和 `o6_coupled` 使用
[2 ms 位置伺服/接触配置](o6-mujoco-coupling.zh-CN.md)。通用 `hand_coupled` 对全部
选中手型（包括 O6）使用 62.5 µs 的约束配置。

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

## 通用配置参数

通用 `hand_coupled` 配置为 `timestep=0.0000625`、joint equality `solref="0.000125 1"`、
`solimp="0.9999 0.9999 0.001 0.5 2"`。L6 原有的 damping=5、kp/kv=20/2，
以及其他手型的 damping=4、kp/kv=200/0 均保留；摩擦、armature、惯量、限位、
执行器及联动比例也保持原值。

通用配置的积分步数是 2 ms 步长的 **32 倍**。控制周期要按秒处理。
`hand_coupled` 允许显式设置不超过 0.0000625 s 的步长；`o6_coupled` 则允许不超过
0.002 s 的步长。上述配置是数值仿真参数，**未经实物标定**。抓取应用需要单独验证
联动、关节止挡、接触力及反驱行为；配置本身不添加自锁。

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
