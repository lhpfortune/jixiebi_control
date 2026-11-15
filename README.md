# 机械臂目标识别与手势搬运综合实践

课程代码：**(2026-2027-1)-S0610350-01**。第7组以DICE五自由度机械臂完成URDF建模、点到点抓放、四色视觉分拣及手势选择确认搬运。

## 目录与交付版本

```text
ros/sfd_jixiebi/       最终ROS工程：URDF/Xacro、launch、视觉/抓放节点、RViz
robot/                平台侧控制服务及实验Notebook，输出已清理
pc/                   21关键点规则识别、去抖与HTTP适配器
tests/                录像配对试验、协议回归、运动学与ROS复验脚本
results/              逐帧/逐窗CSV、原始rostopic输出、复验JSON
docs/                 分工、第三方来源、验证范围与本次变更说明
```

本仓库公开本组开发与集成源码，以上目录可直接浏览；完整文件树同时打包于`project_source.zip`。仓库地址：https://github.com/lhpfortune/jixiebi_control 。实验箱原厂SDK与硬件标定属于外部平台依赖，通过THIRD_PARTY.md说明获取和放置方法，不将其重新声明为本组原创。教学提交代码包另附本机留存的平台依赖快照，便于课程验收复现。原始现场视频和含个人画面的文档不上传到此公开仓库。

## 环境

- ROS建模：Ubuntu 20.04、ROS Noetic、Python 3.8、catkin、RViz、Xacro、cv_bridge、OpenCV。原始平台实验记录为Python 3.8.10、OpenCV 4.5.1；本次WSL复验OpenCV 4.2.0。
- PC：Windows 10/11、Python 3.12；`numpy==1.26.4`、`opencv-contrib-python==4.11.0.86`、`mediapipe==0.10.35`。
- 平台侧：DICE SDK、camera.py、arm_application.py、config.py、config/与vconfig/；串口和UVC依赖沿用实验箱环境。

## ROS编译与启动

将`ros/sfd_jixiebi`复制到`~/robotlearning_ws/src/`，安装ROS Noetic相应依赖后执行：

```bash
cd ~/robotlearning_ws
source /opt/ros/noetic/setup.bash
chmod +x src/sfd_jixiebi/scripts/*.py
catkin_make --pkg sfd_jixiebi
source devel/setup.bash
roslaunch sfd_jixiebi grasp_demo.launch loop:=false
```

仅检查模型和关节滑块时使用：

```bash
roslaunch sfd_jixiebi view_sfd_arm.launch
```

两个入口分别由抓放节点和滑块发布`/joint_states`，不要同时启动。默认抓放视觉源为合成俯视工作区；已存在已标定相机话题时可运行：

```bash
roslaunch sfd_jixiebi grasp_demo.launch vision_source:=camera camera_image_topic:=/camera/image_raw
```

该演示为RViz运动学可视化，物块以Marker附着工具，不包含Gazebo接触、摩擦或通用碰撞规划。原演示5 s未收到完整四色检测会回退到内置场景位置；这属于演示策略，不用于真实机械臂感知失败后的继续运动。

## 平台执行端

把`robot/gesture_robot_server.py`与相应Notebook放到已有DICE视觉实验目录，与平台的camera.py、SDK、config/和vconfig/相邻。开始前设置自己的共享令牌：

```bash
export ROBOT_CONTROL_TOKEN='replace-with-your-own-token'
```

随后在平台Jupyter中运行：

```python
from gesture_robot_server import start_gesture_robot_server
server = start_gesture_robot_server(host='0.0.0.0', port=8765, block=False)
```

自动分拣另运行`robot/实验6_目标识别与垃圾分拣.ipynb`；归位与堆叠对应08、09Notebook。Notebook默认模式以文件内配置为准，真实运动前核对标定、点位、工作空间与当前控制器。一次只允许一个程序占用机械臂。

## PC手势端

```powershell
cd pc
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
powershell -File scripts/download_model.ps1
$env:ROBOT_CONTROL_TOKEN='与平台侧相同的令牌'
.\.venv\Scripts\python.exe pc_http_client.py --robot-url http://192.168.55.1:8765
```

只检查手势时加`--no-robot`。手部关键点模型来自Google官方地址，由脚本下载。ONE选择P4抓取，TWO选择正常复位；移开手后用OK确认；OPEN_PALM取消待确认任务。新HTTP适配器对FIST设置独立2帧优先发送通路；服务端收到STOP后进入ESTOP并尝试卸力，恢复需要人工处理。HTTP 200代表接受命令，是否完成以状态回读及动作结果为准。

## 配对回放与回归验证

使用原始《实验5_目标识别.mp4》：

```powershell
python tests/ablation.py --video "完整路径/实验5_目标识别.mp4" --out results/reproduced_ablation
python tests/controller_regression.py --server robot/gesture_robot_server.py
python tests/stop_priority_test.py
```

`ablation.py`对同一真实录像的6个黄色窗口和20个非黄色窗口比较H26—34/H24—38与2/4/8帧确认，逐帧与逐窗CSV可复核；它是相关时间窗回放试验，不是新增26次独立实体实验。原始录像因包含现场画面不公开，由课程材料提供。

协议回归使用127.0.0.1与FakeArm，覆盖两次取消、执行互锁和100次STOP；没有实体驱动。结果中STOP中位数和P95是软件接口响应，不是机械臂完全停止时间。新增HTTP适配器是本次交付补充，已通过优先停止序列回归；既有现场视频对应此前已运行的手势版本。

## 成员独立负责模块

| 成员 | 负责模块 |
| --- | --- |
| 李涵培 组长 | URDF/Xacro、模型与TF检查、控制实验协助、报告组织 |
| 胡哲凯 | 上位机连接、复杂机械臂控制、动作与相关控制代码、实物联调 |
| 王宏瑜 | 视觉识别、图像处理、阈值与识别效果分析、视觉控制联调 |

## 来源与复用

DICE平台SDK、相机和标定模块来源为实验平台配套材料，网址https://www.dice.com.cn/；ROS消息及工具来源于ROS；颜色处理参考OpenCV官方文档；关键点模型使用MediaPipe，论文：https://arxiv.org/abs/2006.10214 。模型包组织参考marm_description相关公开教程：https://github.com/huchunxu/ros_exploring 。详见THIRD_PARTY.md及docs/CHANGELOG.md。

本项目使用AI辅助文字、图示、统计复核与本次集成补充；既有模型、控制与视觉实验由小组开发调试，补充回放/协议试验在独立环境执行，具体范围以报告原有AI使用说明和变更记录为准。第三方组件按各自原有授权使用，本仓库不替第三方重授许可。
