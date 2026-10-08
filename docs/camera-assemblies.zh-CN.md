# 法兰与相机装配

语言：[English](camera-assemblies.md) | [中文](camera-assemblies.zh-CN.md)

`ar5_08_l6_l/r`、`ar5_08_o6_l/r` 默认包含共用的 44.5 g 实体法兰。
`ar5_08_l6_gemini335l_l/r`、`ar5_08_o6_gemini335l_l/r` 额外包含 29 g 支架和 133 g Gemini 335L，
每腕附件合计 206.5 g，比只有法兰增加 162 g。左右分别选 unit，表达无/左/右/双侧相机。
没有仅带支架的 unit。实体安装与图像渲染开关独立，现有关节名称、限位、mimic 和驱动顺序不变。

## 安装与惯性

组件分别为 `attachments/ar5_08_l6_o6_flange`、左右变体的
`attachments/ar5_08_gemini335l_bracket`、`sensors/orbbec_gemini335l`。
每个组件具有独立 visual/collision。源 CAD 单位为 mm，提交的 mesh 为 m；provenance 文件记录
来源哈希、烘焙变换、已知质量与惯性近似。

法兰大端贴合 arm `tool0`，2 mm 定位凸台进入臂端凹口，小端配合面位于 tool0 Z=22.5 mm。
O6 左侧 root 在配合面，右侧配合面比 root 高 2 mm，所以两侧 root 净平移为 22.5 / 20.5 mm。
L6 使用同一配合距离，右侧源模型配合面倾斜，额外通过 +2° X 旋转校正。手内部 joint frame 不变。
这是机械安装纠错，不是重新定义任务 TCP。

标准机械零位时指定法兰孔位在上、相机在下、看向手下方。法兰绕轴左 −90°、右 +90°，
手的净绕轴朝向保持左 +90°、右 −90°。manifest 提供实际 fixed link：`flange:hand_mount`、
`bracket:camera_mount`、`camera:depth_optical`、`camera:rgb_optical`、`camera:right_ir_optical`。
不依赖 composer 尚未支持的通用 `meta.yaml` frame 偏移语义。

法兰、支架按已知质量与 CAD 均匀密度估计惯性；相机按已知质量与外壳包围盒估计。
它们都不是实测惯性。微小 fixed-frame 质量从实体质量中扣除，避免悄悄增加载荷。
重力与增益由消费端配置；增加质量不代表启用重力。

## O6 拇指运动限制

硬件提供者已确认当前装配与实物一致。使用这套 Gemini 335L 支架时，拇指伸直侧摆的部分路径
会碰相机，需要将侧摆和弯曲配合。这是当前机械装配的限制，不是碰撞过滤错误。

保留原有关节限位和拇指—相机碰撞。不能仅凭独立关节范围判断安全，也不能假设弯曲后全部侧摆
范围都可用；左右几何不同。需要检查包含 mimic、各手指、相机和支架的完整组合轨迹。
回归中的碰撞/无碰撞角度只是验证示例，不是任务控制器、通用安全区域表或真机安全保证。
后续机械方案或资产版本改变时重新核验该说明。

相机采用保留斜角的 CAD 凸包，简单包围盒会在正常张开零位产生误报。法兰和支架按 24 个角向
CAD 扇区建立凸体，保留中央凹槽，部分螺孔仍保守填充。只排除具名安装接缝，消费端还须启用
articulation 自碰撞。

## 光学参考与顶部模块

Gemini `camera_link` 遵循厂家标称深度/左 IR 原点，X 向前、Y 向左、Z 向上；optical frame
采用 X 向右、Y 向下、Z 向前。右 IR 相距 95 mm，RGB 标称在左 IR 右侧 23.75 mm，并保留厂家
微小旋转。这些是 CAD/厂家标称参考，不是设备标定；有正式设备/profile 标定后应替换。
参考版本与几何配准依据在 `provenance.json`，没有再分发厂家 mesh。

`sensors/workstation_zed2i` 从现有 bench 资产提取。静态 `workstation_zed2i` unit 可与两套
独立臂 unit 同场景加载。五个原 bench 工作站通过显式 recipe 组件保持原视觉几何和顶部安装变换；
它们仍是整机组合，静态相机 unit 不合并独立加载的机械臂。

旧顶部支架 COM/惯性处于 +90° X 旋转之前的坐标系，将两者同时旋转到实际 mesh frame 后，
COM 与 CAD 均匀密度计算相差不到 11 μm。相机看似偏大的局部 COM 与偏置 mesh 一致，予以保留。
保留原 CAD 总质量，不冒充重新称量的 ZED 质量。两目按镜头盖中心建立标称 optical frame，
基线 120 mm，向外偏 3 mm 避免视点落在不透明镜头盖内。这是渲染近似，不是镜头标定。

## 再生成与检查

按 provenance 指定的版本运行腕部生成器，然后 recompose 受影响的 unit/workstation 并执行测试：

```bash
uv run --no-project --with pyyaml --with trimesh==4.11.1 --with scipy==1.17.0 --with numpy==2.3.1 \
  python scripts/build_wrist_attachments.py
PYTHON=/path/to/authoring/python just test
```

`--source-dir` 可重新导入六个经过哈希核对的原始 CAD；正常再生成直接使用仓库内归一化 mesh。
测试编译全部资产并检查 drift、frame parity、质量/驱动、光学基线，以及禁止/可行拇指路径的真实
MuJoCo 接触。Isaac 后端、相机随动和规划覆盖在仿真仓库中验证。
