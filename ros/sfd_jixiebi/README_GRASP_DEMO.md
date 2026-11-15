# RViz 四色物块抓取演示

本目录在原有 6 自由度机械臂 URDF/Xacro 模型上增加了独立的抓取演示，不修改
`urdf/sfd_arm.xacro` 和 `urdf/sfd_arm.urdf`。演示包含：

- 红、绿、黄、蓝四个 30 mm 立方体物块；
- 四个同色放置区域，作为抓取后的目标地点；
- 机械臂依次接近、抓取、抬起、搬运、放置并退出；
- OpenCV HSV 颜色识别，发布检测结果和可视化框；
- 抓取过程中物块跟随 `tool0` 位姿运动，松开后固定到目标区域；
- 演示循环运行，便于课堂展示。

## 1. 运行

```bash
cd ~/robotlearning_ws
catkin_make
source devel/setup.bash
roslaunch sfd_jixiebi grasp_demo.launch
```

常用参数：

```bash
# 只演示一次
roslaunch sfd_jixiebi grasp_demo.launch loop:=false

# 加快到 1.5 倍
roslaunch sfd_jixiebi grasp_demo.launch speed:=1.5

# 不运行 OpenCV 节点，直接使用内置的物块位置
roslaunch sfd_jixiebi grasp_demo.launch vision:=false

# 不打开 RViz，只运行节点（用于检查话题）
roslaunch sfd_jixiebi grasp_demo.launch rviz:=false
```

原有的关节滑块查看方式仍然可用：

```bash
roslaunch sfd_jixiebi view_sfd_arm.launch
```

不要同时启动 `joint_state_publisher_gui` 和抓取演示，因为二者都会向
`/joint_states` 发布关节角。抓取演示节点负责发布六个主动关节和从动右夹爪关节。

## 2. RViz 中的显示内容

`rviz/grasp_demo.rviz` 已加入：

- `Pick and Place Scene`：四色物块、同色放置区域和文字标签；
- `OpenCV Detections`：OpenCV 在俯视图中检测到的彩色轮廓；
- `OpenCV Image`：带 HSV 阈值、形态学滤波、轮廓和外接框的调试图像，默认关闭，可在
  RViz 左侧 Displays 面板中勾选启用。

固定坐标系为 `base_link`，物块和放置区域都以米为单位显示在该坐标系下。

## 3. 节点与话题

| 节点 | 作用 |
|------|------|
| `sfd_color_vision` | 生成/读取俯视图，进行 OpenCV HSV 颜色识别 |
| `sfd_pick_place_demo` | 发布关节状态、控制机械臂、发布场景 Marker 和抓取状态 |
| `robot_state_publisher` | 根据 `/joint_states` 计算并发布 TF |

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 6 个主动关节 + `finger_joint_right` |
| `/sfd_jixiebi/detected_blocks` | `geometry_msgs/PoseArray` | 按红/绿/黄/蓝顺序排列的检测位置 |
| `/sfd_jixiebi/vision_markers` | `visualization_msgs/MarkerArray` | OpenCV 检测框和标签 |
| `/sfd_jixiebi/vision/debug_image` | `sensor_msgs/Image` | OpenCV 调试图像 |
| `/sfd_jixiebi/scene_markers` | `visualization_msgs/MarkerArray` | 物块、放置区域、放置完成标记 |
| `/sfd_jixiebi/demo_status` | `std_msgs/String` | 当前抓取阶段文字状态 |

## 4. OpenCV 识别流程

默认 `vision_source:=synthetic`。节点用 OpenCV 生成一张俯视工作区图像，再通过
HSV 阈值分割识别红、绿、黄、蓝四种颜色：

1. `BGR -> HSV`；
2. 对每种颜色做 `inRange`，红色使用两个 Hue 区间；
3. `MORPH_OPEN` 去噪、`MORPH_CLOSE` 补洞；
4. `findContours` 找最大轮廓；
5. 用图像矩计算物块中心，并按工作区尺寸换算到 `base_link` 下的米制坐标。

抓取节点订阅检测结果后，用解析逆运动学计算每个物块和放置区域的关节角。若 5 秒内
没有收到四色完整检测，节点会回退到 `src/sfd_jixiebi/demo_scene.py` 中的内置位置。

### 使用真实俯视相机

如果已有标定好的俯视相机，可运行：

```bash
roslaunch sfd_jixiebi grasp_demo.launch vision_source:=camera camera_image_topic:=/camera/image_raw
```

该模式默认把整幅图像线性映射到 `[-0.28, 0.28] m` 的工作区，适合演示用固定俯视相机。
若相机分辨率或工作区不同，请在 `src/sfd_jixiebi/demo_scene.py` 中调整 `WORKSPACE_MIN`、
`WORKSPACE_MAX`，或在后续扩展中使用 `camera_info` 做针孔模型反投影。

## 5. 实现边界

本演示是 **RViz 运动学可视化**，不是 Gazebo 物理仿真。抓取通过“物块 Marker 跟随
`tool0` 位姿”实现，夹爪开合由 `joint6` 的主/从关节联动显示。因此不包含接触力、摩擦、
碰撞和真实抓取失败判定。当前 URDF 的腕部和工具链较长，物块位置被设置在机械臂可完成
全俯视抓取的紧凑工作区内。