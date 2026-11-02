# AR5/O6 组合物理配置

6 个 AR5＋O6 recipe 显式设置 `mjcf_physics: ar5_o6`。生成器在组合组件并转换为力矩 motor 后应用[原生配置](../src/linker_robot_assets/composer/ar5_o6_physics.py)；修改来源后重新生成，不直接修改生成 XML。独立 O6、未装 O6 的 AR5、A7/P7＋O6、L6 组合维持原动力学和材质。

AR5/AR5_08 的 J6/J7 限角是另一项模型修正：左右组件 URDF/MJCF 和全部受影响组合统一 **±pi/3（±60°）**，包括规划使用的单臂 URDF。其他关节限角保留。

## 原生 MJCF 数值

| 范围 | 被动阻尼 N·m·s/rad | armature kg·m² | motor 原生输出范围 N·m |
|---|---:|---:|---:|
| AR5 J1–J7 | 0 | 0.01 | ±200 |
| O6 拇指侧摆 | 0 | 0.001 | ±10 |
| O6 拇指弯曲、四指 MCP 弯曲 | 4 | 0.001 | ±60 |
| O6 从动关节 | 0.4 | 0.001 | 无独立电机 |

关节 frictionloss=0。联动比例保持拇指1.86、四指0.89；equality 的 solref=(0.002,1)、solimp=(0.999,0.999,0.001,0.5,2)。机械臂根下所有本体、法兰、相机架、相机和光学帧设置 gravcomp=1；世界重力仍为(0,0,-9.81)，供任务物体使用。几何、质量惯量、安装变换和碰撞排除保持原样。这些是仿真选型值，不是真机额定参数。

这些组合的所有碰撞 geom 使用 condim=4、solref=(0.004,1)、margin=0、priority=0、solmix=1。手部 friction=(4,0,0)、solimp=(0.95,0.95,0.001,0.5,2)；机械臂、附件、基座 friction=(1,0,0)、solimp=(0.9,0.95,0.001,0.5,2)。视觉 geom 仍不参与碰撞。保留原机器人 gap=0，包括继承值；该生成步骤不覆盖资产作者化的 gap。仿真0.0001 m回退仅用于未显式作者化的场景资产。

MJCF 场景设置为 timestep=1/480秒、implicitfast、Newton求解器、iterations=100、ls_iterations=50、elliptic、impratio=100。cone/impratio为场景级。MuJoCo求解器名称Newton与 **Newton碰撞库** 是不同概念；普通MuJoCo直接加载仍使用自己的碰撞实现，不能据此保证复现Newton运行时的多点接触和抓取效果。

## 运行时边界

生成的 workstation.xml 维持 gear=1 的力矩 motor、力矩输入范围及输出限矩；ctrl 输入是力矩，**不是目标角度**。它不会额外添加与导入端冲突的位置伺服。manifest 中的通用 default_gains/gain_profiles 仍为组件元数据，**不是**已选 Newton 柔顺控制器配置。

对应仿真来源是 [PR #64](https://github.com/linker-bot/linker-sim-isaac/pull/64)、configs/controllers/newton_compliant/{arm_controller,hand_controller}.yaml 及柔顺驱动设置。运行时原生位置输出采用：

| 分组 | Kp | Kd |
|---|---:|---:|
| J1–J7 | 16000 | [614.2237548828125,613.7109985351562,603.791015625,600.3331298828125,604.13671875,604.7486572265625,603.66796875] |
| 拇指侧摆 | 600 | 24 |
| 拇指弯曲 | 2500 | 40 |
| 四指 MCP | 1250 | 40 |

虚拟推进限矩另为侧摆3 N·m、屈曲5 N·m；机械臂目标保护为199 N·m，都不是资产原生motor上限。虚拟轴状态、平滑参考、速度前馈、子步插值和reset由仿真提供，资产不另实现，也不伪造标准MJCF不支持的属性。

[仿真 PR #66](https://github.com/linker-bot/linker-sim-isaac/pull/66)维护**指令60 Hz、积分480 Hz、碰撞刷新240 Hz**，以及Newton缓冲区/工作分配、求解/碰撞确定性、接触匹配和kf=1000，CPU/CUDA共用。标准MJCF不能表示这些Newton专有运行设置；timestep只表示积分步长，运行时可以覆盖XML选项。

任务物体不在机器人资产内。对应仿真中的物体采用friction=(1,0.005,0)、solimp=(0.9,0.95,0.001,0.5,2)、condim=4、法向solref=(0.004,1)。等优先级手—物混合后μ=4、扭转0.005 m、solimp=(0.925,0.95,0.001,0.5,2)。REFSAFE及上游摩擦下限仍有效。直接在此MJCF添加物体的任务须自行配置物体材质和控制器，单独此文件不等于完整抓取基准或真机标定模型。

URDF保留运动学/硬件描述用途：共享J6/J7限角修正，但不编码这套组合MuJoCo动力学、重力补偿、新原生电机限矩和虚拟控制器。其旧effort/dynamics元数据不能当作Newton选型配置。

## 重新生成与验证

作者环境需安装MuJoCo、NumPy、SciPy、trimesh和PyYAML：

```bash
PYTHON=/path/to/python just compose
PYTHON=/path/to/python just test
PYTHON=/path/to/python just check-drift
```

测试编译并读回6个组合的实际参数，检查组件/生成模型限角覆盖以及非目标组合保留。资产PR合并后再更新仿真引用并完成集成验收，主线见[仿真 #53](https://github.com/linker-bot/linker-sim-isaac/issues/53)。
