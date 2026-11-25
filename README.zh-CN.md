# linker-sim-assets

灵心巧手（Linkerbot）的机器人模型文件，供你在自己的仿真器中使用：URDF 和 MJCF 及其网格。

(English: [README.md](README.md))

| 目录 | 机器人 | 我们是否已仿真验证 |
| --- | --- | --- |
| [`workstations/lw_v0/`](workstations/lw_v0) | **LW-v0**：两台 AR5_08 机械臂配 LinkerHand O6 灵巧手、腕部相机（奥比中光 Gemini 335L）、顶部相机（Stereolabs ZED 2i）和工作台 | 是：[linker-sim](https://github.com/linker-bot/linker-sim)（MuJoCo 4.0.1）和 linker-lab（Isaac Lab，MuJoCo 3.17） |
| [`hands/o6/`](hands/o6) | LinkerHand O6，左手和右手 | 是，作为 LW-v0 的一部分（同一只手） |
| [`hands/l6/`](hands/l6)、[`l20lite/`](hands/l20lite)、[`l25/`](hands/l25)、[`l30/`](hands/l30) | LinkerHand L6、L20 Lite、L25、L30 | 否（见[尚未验证](#尚未验证)） |

其他机器人（AR5、A7、A7 Lite、P7 机械臂，躯干，其他工作站）在标签
[`v0.4.0`](https://github.com/linker-bot/linker-sim-assets/tree/v0.4.0) 中。它们会在我们的仿真器中逐一
验证后再回到这里。

## 加载哪个文件

| 你使用 | 加载 |
| --- | --- |
| MuJoCo | `workstations/lw_v0/lw_v0.xml`（整个工作站），或单个部件：`left.xml`、`right.xml`、`top_camera.xml`、`support.xml`。灵巧手：`hands/<hand>/<hand>_<side>.xml` |
| ROS、Isaac Sim、PyBullet、Drake、viser 等 URDF 工具 | `workstations/lw_v0/lw_v0.urdf`，或 `hands/<hand>/<hand>_<side>.urdf` |
| Python（linker-sim、linker-lab） | 从本仓库的某个标签安装 `linker-lw-v0`，例如在 `pyproject.toml` 中写 `"linker-lw-v0 @ git+https://github.com/linker-bot/linker-sim-assets@v0.5.0"`；然后 `importlib.resources.files("linker_lw_v0") / "workstations/lw_v0/lw_v0.xml"` |

`linker-lw-v0` 包含 LW-v0 的文件（路径与本仓库相同）、它用到的 O6 网格，以及
`hands/o6/sdk_channels.yaml`、`motor.yaml` 以及 `arms/ar5_08/controller.yaml`。包中没有代码。其他文件请克隆本仓库或下载 ZIP。

`lw_v0.xml` 用 MuJoCo 的 `<attach>` 放置四个部件。每个部件是一个关节链：机械臂部件包括机械臂、法兰、
灵巧手、相机支架和腕部相机。不支持 `<attach>` 的仿真器可以分别加载各部件，并按 `lw_v0.xml` 中的位姿放置。

## 坐标系和单位

- 米、千克、秒、弧度。每个网格都在其连杆坐标系中，单位为米。
- LW-v0 的世界坐标系：原点在两个机械臂底座之间、立柱顶部；x 朝前（朝向工作台，即腕部相机的视野方向），
  y 指向左臂，z 朝上。台面在 z = -0.5 m，地面在 z = -1.267 m（工作台脚底）。
- 名称：在 `lw_v0.xml` 中，每个名称都带有部件前缀（`left/`、`right/`、`top_camera/`、`support/`），
  例如 `left/arm_AR5_5_08L_W4C4A6_joint_1`、`left/hand_lh_index_mcp_pitch`。`lw_v0.urdf` 使用相同的名称。
  灵巧手自己的文件使用手的名称（`lh_index_mcp_pitch`）。

## LW-v0

| | |
| --- | --- |
| 关节 | 每侧：7 个机械臂关节、6 个驱动的手部关节、5 个与之耦合的手部关节；共 36 个 |
| 驱动 | 每侧 7 个机械臂关节和 6 个手部关节上的位置驱动（`<actuator>`） |
| 初始姿态 | `lw_v0.xml` 中的关键帧 `home` |
| 物理 | 480 Hz（`timestep` 1/480 s）、implicitfast 积分器、Newton 求解器、椭圆摩擦锥；见 `lw_v0.xml` 的 `<option>` |

相机（部件文件中的 `<camera>`；相机沿其光学坐标系的 +z 方向观察，图像 x 向右、y 向下，与 OpenCV 相同）：

| 相机 | 型号 | 分辨率 | fx, fy（像素） | 真机帧率 |
| --- | --- | --- | --- | --- |
| `left/camera_rgb_optical`、`right/…` | Gemini 335L 彩色 | 1280 × 800 | 596.81, 593.02 | 60 Hz |
| `left/camera_depth_optical`、`right/…` | Gemini 335L 深度 | 1280 × 800 | 640.00, 627.87 | 30 Hz |
| `top_camera/sensor_left_optical` | ZED 2i 左目，2.1 mm 镜头 | 1280 × 720 | 448.13, 514.13 | 60 Hz |

主点位于图像中心。帧率不在文件中。

### MJCF 有而 URDF 无法表达的内容

URDF 无法表达以下内容，因此它们只在 MJCF 中：

- 驱动及其增益（`<position kp kv>`）、电枢惯量（armature）
- 重力补偿：所有机器人刚体都开启重力补偿（`gravcomp="1"`）
- 接触参数：摩擦、`condim`、`solref`、`solimp`、`margin`、`gap`；哪些刚体对之间不碰撞
  （`<contact><exclude>`）：一个机械臂部件内部只有不同手指之间会碰撞
- 灵巧手耦合的刚度（`<equality>`）；URDF 的 `mimic` 只有比例
- 求解器选项、初始姿态关键帧和相机内参

`lw_v0.urdf` 包含其余内容：带质量和惯量的连杆、视觉和碰撞网格、带限位的关节、以 `mimic` 表示的耦合、
阻尼和摩擦。其中：
- `effort` 是 MJCF 中的力矩限制（机械臂 J1–J4 120 N·m，J5–J7 100 N·m；手部拇指横摆 10，其他驱动 60）；
  耦合的手部关节没有驱动，因此 effort 为 0
- `velocity` 不由 MJCF 仿真。机械臂的数值是机器人控制器的运动限制（J1–J7：0.39、0.32、0.39、0.39、
  0.49、0.49、0.49 rad/s）；手部驱动为 1 rad/s，耦合关节为其乘以耦合比例

### 实测与建模

- 质量、惯量和运动学：来自制造商的 URDF 和 CAD。
- 驱动、力矩限制和接触参数：建模得到。它们是 LW-v0 的抓取策略在 linker-lab 中训练、在 linker-sim 中运行时
  所用的数值。
- 碰撞形状：凸包，即抓取策略训练时使用的形状；不是视觉网格。
- 颜色：由硬件团队对照产品照片匹配，不是制造商的色号。
- 相机外壳：制造商自己的网格（见 NOTICE）。

## 机器人控制器

真实的机械臂和灵巧手在指令与关节之间都有控制器。它们的行为不在 URDF 或 MJCF 中，这两种文件只描述机构和
位置驱动。常数在数据文件中；linker-sim 和 linker-lab 实现了这两个模型。**只加载 MJCF 的仿真器只有驱动、
没有这些控制器：** 手指会以 60 N·m 驱动所允许的速度到达目标，比真实的手快得多、力量也大得多。

### O6 灵巧手电机（`hands/o6/motor.yaml`，模型 `non-backdrivable-shaft-v1`）

每个驱动关节由一根电机轴通过不可反驱的减速器带动。电机轴位置就是关节驱动的目标：MJCF 的 `<position>`
驱动（刚度 `Kp`、阻尼 `kv`）把关节拉向它，耦合关节通过 `<equality>` 跟随。每条指令（周期 `dt`；我们的
产品中为 1/60 s）对每根轴 `s`，目标 `t`（限制在关节范围内）、测得的关节位置 `q` 及其上一次的值 `q0`：

```
v      = (q - q0) / dt                                         关节速度
tau    = clamp(motor_stiffness * (t - s), ±advancing_cap)       电机力矩
load   = (tau - Kp * (s - (q + dt*v)) + output_damping * v)
         / (Kp + (output_damping + viscosity) / dt)               该力矩对抗手指负载所能前进的步长
free   = min(dt * speed_limit, dt * |tau| / (viscosity + passive))  空载电机的步长
step   = sign(tau) * min(max(sign(tau) * load, 0), |t - s|, free)  永不后退，永不越过 t
s      = s + step
```

`passive` 是电机轴感受到的被动阻尼：关节自身的阻尼，加上驱动 `kv` 中不属于电机 `output_damping` 的部分，
再加上每个耦合关节的阻尼乘以其耦合比的平方（例如拇指屈曲：4 + 0.4 × 1.86² = 5.38384）。两条指令之间，
驱动目标从旧的轴位置线性移动到新的轴位置，并加上速度前馈 `output_damping / Kp × step / dt`，使驱动不落后于电机轴。

随后，每个物理步都把目标限制在驱动的力矩上限（关节的 `actuatorfrcrange`：拇指侧摆 10 N·m，其余 60 N·m）
减去 1 N·m 之内：如果 `Kp * (target - q) - kv * qdot` 会超过它，就把目标改为恰好产生该力矩的值（`q` 和
`qdot` 为该步测得的值）。这样驱动从不被要求超出其能力。没有这一步时，在 MuJoCo 4 中处于力矩上限的驱动会失去
隐式阻尼并在上限处振荡；上面的电机模型会把这种振荡当作负载，使电机轴永久停住：手指再也张不开。（`v1`；`v0`
没有这一步。）

### AR5_08 机械臂控制器（`arms/ar5_08/controller.yaml`）

xCore 控制器让关节沿时间最优、限制加加速度的轨迹运动，不超过各关节的 `max_velocity`、`max_acceleration`
和 `max_jerk`；一次运动中所有关节同时到达。它让 (J6, J7) 保持在凸八边形 `wrist_region` 内，该八边形的
包围盒就是 URDF/MJCF 中 J6/J7 的范围。驱动的力矩上限（J1–J4 120 N·m，J5–J7 100 N·m）在 MJCF 中。

## 灵巧手

每个灵巧手目录包含 `<hand>_left.urdf`、`<hand>_left.xml`，右手同理，以及 `sdk_channels.yaml` 和 `meshes/`。

- **O6**：LW-v0 中的灵巧手，单独使用：驱动、接触参数、碰撞凸包、耦合和求解器选项都与 LW-v0 中相同。
- **L6、L20 Lite、L25、L30**：内容与 v0.4.0 相同。它们以视觉网格的凸包进行碰撞。在获得它们的真实颜色之前（O6 已有），它们显示为统一的中性灰色。

### SDK 通道

灵巧手 SDK 为每个驱动关节收发一个 0 到 100 的数值，顺序由 SDK 自己定义。`sdk_channels.yaml` 列出每个
通道驱动哪个关节（`{S}` 为 `l` 或 `r`）：

- `channels`：SDK 的顺序；`reserved` 表示不驱动任何关节的预留位
- `legacy_channels`（L6）：早期设备的顺序
- `slots`（L25）：完整的 20 位 SDK 布局，其中 4 位为预留
- `convention: linear-fit-v0`：SDK 数值 100 对应关节下限，0 对应上限：

  ```
  angle = lower + (100 - value) / 100 * (upper - lower)
  value = 100 - 100 * (angle - lower) / (upper - lower)
  ```

  其中 `lower` 和 `upper` 是 URDF 中的关节限位；若 `clip_overrides` 中有列出，则以其为准。

### 尚未验证

L6、L20 Lite、L25 和 L30 能在 MuJoCo 中编译，其 URDF 与 MJCF 一致（见下面的测试），但我们尚未仿真
验证。它们的数据中已知的差异（待与硬件团队一起修正）：

- 左右手不一致：L25 的 URDF effort/velocity（左手 100/1，右手 1/100）；L20 Lite 的连杆质量（最多相差
  2.3 倍）、两个 effort 限制以及无名指/小指耦合（左 1.30，右 1.34）；L30 的腕部连杆质量（左 0.707 kg，
  右 0.076 kg）、食指 MCP 限位（1.57 / 1.518 rad）以及拇指的 effort/velocity。
- URDF 中的关节阻尼和摩擦与 MJCF 不同（MJCF：阻尼 4–5，摩擦损耗 0.5）。
- 它们的 MJCF 驱动没有力矩限制。

## LW-v0 的已知问题

LW-v0 保留了两个产品验证时的几何。以下问题推迟到后续工作站（LW-v1）解决，因为修正它们会改变物理或相机视角：

- 腕部相机在支架上的位置比支架 CAD 中高 16.4 mm。
- 顶部相机的光学坐标系沿光轴比 ZED 2i 官方描述中的位置靠前 22.7 mm。

## 贡献者须知

直接编辑 URDF 和 MJCF；没有生成器。`tests/test_assets.py` 检查：每个 MJCF 都能编译；每个 URDF 与其 MJCF
一致（关节、耦合、质量、惯量、初始姿态下的连杆位姿、网格文件）；每个被引用的文件都存在，且每个网格都被
使用；网格格式；以及 SDK 通道文件：

```
uv run --group test pytest
```

CI 在 MuJoCo 3.17.0 和 4.0.1 上运行测试，并构建、安装 `linker-lw-v0` 包。

网格：每个连杆一个 `.obj`，即 `meshes/visual/<link>_vis.obj`，配一个 `.mtl`；每个机器人一张
`<robot>_palette.png`。颜色是指向调色板的纹理坐标，因此 URDF 工具和 MuJoCo 显示相同的颜色。调色板以机器人
命名，这样把纹理收集到同一目录的工具（如 USD 转换器）不会让不同机器人的调色板互相覆盖。碰撞网格为
`meshes/collision/<link>_col.obj`。不使用 STL、glTF 或其他格式。

## 许可证

Apache-2.0（[LICENSE](LICENSE)）。相机网格来自制造商，同样是 Apache-2.0（[NOTICE](NOTICE)）。
