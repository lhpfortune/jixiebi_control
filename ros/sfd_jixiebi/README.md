# sfd_jixiebi —— 绿色 6 自由度机械臂 URDF/Xacro 模型

仿照 `marm_description` 包的结构编写的机械臂描述包，包含完整的 URDF/Xacro 模型和 RViz 可视化 launch 文件。

## 一、目录结构

```
sfd_jixiebi/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── view_sfd_arm.launch      # RViz 可视化 launch 文件
├── rviz/
│   └── sfd_arm.rviz             # RViz 配置文件
└── urdf/
    ├── sfd_arm.xacro            # 模型源文件（带变量与宏，改这个）
    └── sfd_arm.urdf             # 由 xacro 自动生成（不要手改）
```

## 二、运行方法

```bash
cd ~/robotlearning_ws
catkin_make
source devel/setup.bash
roslaunch sfd_jixiebi view_sfd_arm.launch
```

launch 文件会自动打开 `joint_state_publisher_gui`，拖动滑块即可驱动 6 个关节。
若不需要 GUI（例如只用 `robot_state_publisher`），可以：

```bash
roslaunch sfd_jixiebi view_sfd_arm.launch gui:=false
```

单独检查模型语法：

```bash
check_urdf src/sfd_jixiebi/urdf/sfd_arm.urdf
```

修改 `.xacro` 后重新生成 `.urdf`：

```bash
xacro src/sfd_jixiebi/urdf/sfd_arm.xacro -o src/sfd_jixiebi/urdf/sfd_arm.urdf
```

## 三、连杆与关节结构

整条运动链从 `base_link` 一直串联到末端夹爪 `tool0`：

```
base_link                正方形底板（绿色 140×140×20 mm）+ 舵机0 外壳（黑色）
   │ joint1   continuous  axis = Z (0 0 1)     舵机0 —— XY 平面 360° 回转
link1                     回转底座（绿色圆柱 Ø104×70 mm）
   │ joint2   revolute    axis = X (1 0 0)     舵机1 —— YZ 平面俯仰，±1.5708 rad（行程 180°）
link2                     大臂（肩部舵机1 黑色外壳 + 绿色连杆 160 mm）
   │ joint3   revolute    axis = X (1 0 0)     舵机2 —— YZ 平面俯仰，±3π/4 rad（行程 270°）
link3                     小臂（肘部舵机2 黑色外壳 + 绿色连杆 140 mm）
   │ joint4   revolute    axis = X (1 0 0)     舵机3 —— YZ 平面俯仰，±3π/4 rad（行程 270°）
link4                     腕部俯仰段（腕部舵机3 黑色外壳 + 绿色连杆 80 mm）
   │ joint5   continuous  axis = Z (0 0 1)     舵机4 —— 夹爪 XY 平面 360° 回转
link5                     腕部回转法兰（舵机4 外壳 + 绿色法兰盘）
   ├─ joint6   prismatic  axis = Y (0 1 0)     舵机5 —— 夹爪开合（主关节），行程 0 ~ 0.01 m
   │    └─ link6              左指滑块（黑色，随 joint6 沿 +Y 平移）
   │         └─ finger_joint_left (fixed)  ── gripper_left   左夹爪（绿色手指 + 黑色指垫）
   └─ gripper_base_joint (fixed)
        └─ gripper_base       夹爪底座 / 舵机5 壳体（黑色）
             ├─ finger_joint_right (prismatic + mimic joint6) ── gripper_right
             │                                          右夹爪（随 joint6 沿 −Y 反向平移）
             └─ tool0_joint (fixed)        ── tool0         工具中心点 TCP
```

**夹爪两指对称开合的实现**：`joint6` 是主关节，右指的 `finger_joint_right` 用 URDF 的
`<mimic>` 标签跟随它反向运动，两指严格镜像：

```xml
<joint name="finger_joint_right" type="prismatic">
    <parent link="gripper_base"/>
    <child  link="gripper_right"/>
    <origin xyz="0 ${-finger_offset_y} 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="50" velocity="0.05" lower="${-gripper_stroke}" upper="0"/>
    <mimic joint="joint6" multiplier="-1" offset="0"/>
</joint>
```

`q_right = -1 × q_joint6`，所以 joint6 从 0 走到 0.01 时，左指 y 从 +0.012 到 +0.022，
右指 y 从 −0.012 到 −0.022，两指同步张开、始终对中。`joint_state_publisher` 的
`use_mimic_tags` 默认为 `true`，会自动算出 `finger_joint_right` 的值并发布，因此
GUI 里只会出现 `joint6` 一个滑块，拖动时两指联动。

## 四、关节参数汇总

| 关节 | 舵机 | 类型 | 轴 (axis) | 限位 | effort | velocity |
|------|------|------|-----------|------|--------|----------|
| joint1 | 舵机0 | continuous | `0 0 1` (Z) | 无限制，XY 平面 360° | 100 | 3.14 |
| joint2 | 舵机1 | revolute   | `1 0 0` (X) | `[-1.5708, 1.5708]`（180°） | 100 | 1.57 |
| joint3 | 舵机2 | revolute   | `1 0 0` (X) | `[-3π/4, 3π/4]` ≈ ±2.3562（行程 270°） | 100 | 3.14 |
| joint4 | 舵机3 | revolute   | `1 0 0` (X) | `[-3π/4, 3π/4]` ≈ ±2.3562（行程 270°） | 100 | 3.14 |
| joint5 | 舵机4 | continuous | `0 0 1` (Z) | 无限制，XY 平面 360° | 100 | 3.14 |
| joint6 | 舵机5 | prismatic  | `0 1 0` (Y) | `[0, 0.01]` | 50 | 0.05 |
| finger_joint_right | —（跟随 joint6） | prismatic | `0 1 0` (Y) | `[-0.01, 0]`，`<mimic joint6 ×-1>` | 50 | 0.05 |
| finger_joint_left / gripper_base_joint / tool0_joint | — | fixed | — | — | — | — |

> 说明：URDF 规范中 `continuous` 类型不允许出现 `lower`/`upper`，所以舵机 0/4（joint1、joint5）
> 只给出足够大的 `effort` 与 `velocity`，以符合课程中对“物理干涉”的考量。
> 舵机1 用 `revolute` 并给出 ±1.5708 rad 的硬限位，舵机2/3（joint3、joint4）同样用 `revolute`，
> 限位 ±3π/4 rad（精确 135°，由 `joint34_limit` 变量统一定义，改一处两关节同步）。

## 五、颜色约定

| 部位 | 材质名 | rgba |
|------|--------|------|
| 机械臂主连杆 | `Green` | `0.2 0.8 0.2 1.0` |
| 舵机 / 关节外壳、夹爪底座、指垫 | `Black` | `0.1 0.1 0.1 1.0` |

每个连杆都同时包含 `<visual>` 与 `<collision>`，且两者的 `origin` 与 `geometry` 完全一致
（在 xacro 中用 `green_cylinder` / `black_box` 等宏统一生成，保证不会写歪）。

## 六、尺寸参数

所有尺寸都写成 xacro 变量，集中在 `sfd_arm.xacro` 开头，方便直接改：

| 变量 | 默认值 (m) | 含义 |
|------|-----------|------|
| `base_size` / `base_height` | 0.14 / 0.02 | 正方形底板边长 / 厚度 |
| `servo0_radius` / `servo0_height` | 0.032 / 0.03 | 舵机0 外壳 |
| `link1_radius` / `link1_length` | 0.052 / 0.07 | 回转底座 |
| `link2_radius` / `link2_length` | 0.026 / 0.16 | 大臂 |
| `link3_radius` / `link3_length` | 0.023 / 0.14 | 小臂 |
| `link4_radius` / `link4_length` | 0.021 / 0.08 | 腕部俯仰段 |
| `link5_length` | 0.052 | 舵机4 输出面高度 |
| `gripper_stroke` | 0.01 | 夹爪开合行程（joint6 单侧行程） |

零点位姿下 `tool0` 位于 `base_link` 正上方 **0.628 m** 处，两夹爪分别位于 y = ±0.012 m。

## 七、说明

- 底座舵机（舵机0）按题目描述置于正方形底板下方，模型中用黑色圆柱表示，`joint1` 的 Z 轴
  穿过底板中心。
- 夹爪两指**对称开合**：`joint6`（prismatic，主关节）驱动 `link6` 沿 +Y 平移，
  `finger_joint_right` 通过 `<mimic joint="joint6" multiplier="-1"/>` 反向等量平移，
  两指始终关于夹爪中心线对称。`joint6 = 0` 时两指内侧面间距 0.012 m（近似闭合），
  `joint6 = 0.01` 时张开到 0.032 m，`tool0` 始终位于两指正中间。
- RViz 配置 `rviz/sfd_arm.rviz` 使用 RViz 自带 `default.rviz` 的停靠布局
  （`Window Geometry → QMainWindow State`），左侧 Displays 面板约 342 px 宽、
  中间 3D 视图约 575 px，并把 Displays 面板顶部的帮助框高度从 78 调小到 40。
- `base_link` 是固定于世界的根连杆，按 KDL 的限制没有添加 `<inertial>`（与参考包
  `marm_description` 做法一致），这样启动时不会出现根连杆惯量警告。


## 八、四色物块抓取演示

新增的 RViz + OpenCV 抓取演示见 [README_GRASP_DEMO.md](README_GRASP_DEMO.md)。运行：oslaunch sfd_jixiebi grasp_demo.launch。
