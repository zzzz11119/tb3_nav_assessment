# TurtleBot3 第一阶段自主导航考核

基于 **Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 和 TurtleBot3
Burger** 的自主导航考核项目。仓库包含自建仿真场景、SLAM/Nav2 基线以及
四项附加挑战，并提供可重复执行的静态测试、运行态检查和 Ubuntu 验收证据。

## 项目状态

| 模块 | 状态 | 入口 |
|---|---|---|
| 自建 Gazebo 世界与传感器基线 | 已通过 | `simulation.launch.py` |
| 附加挑战一：RGB-D 相机 | 已通过 | `rgbd_simulation.launch.py` |
| 附加挑战二：Nav2 A* 规划器 | 已通过 | `tb3_astar_planner` |
| 附加挑战三：YOLOv5n 目标检测 | 已通过 | `visual_perception.launch.py` |
| 附加挑战四：自主前沿探索 | 已通过 | `autonomous_exploration.launch.py` |

自主探索现场结果：机器人穿过中央通道并完成全局探索，发送/成功/失败目标数为
`25/22/3`，地图自动保存，最终运行态检查通过。

## 仓库结构

```text
config/                 ROS 2、SLAM、探索与感知参数
docs/                   按挑战、设计、验收和项目记录分类的文档
launch/                 基础仿真及各附加挑战启动入口
maps/                   最终地图、探索地图和 SLAM 对比地图
models/                 YOLOv5n ONNX 与离线演示模型
optional_challenges/    独立的 Nav2 A* C++ 插件 package
tb3_nav_assessment/     Python 节点和前沿算法
test/                   主 package 静态验收测试
urdf/                   TurtleBot3 Burger RGB-D Xacro
worlds/                 自建 Gazebo 世界
```

完整文档导航见 [`docs/README.md`](docs/README.md)，地图用途见
[`maps/README.md`](maps/README.md)。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- TurtleBot3 Burger
- Python 3.10

每个终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 构建

在仓库根目录执行：

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --base-paths .
source install/setup.bash
```

若 TurtleBot3 仿真 package 尚未安装，请先按 ROBOTIS 的 Humble 分支准备
`~/turtlebot3_ws`。

## 快速运行

基础自建世界：

```bash
ros2 launch tb3_nav_assessment simulation.launch.py
```

RGB-D 相机：

```bash
ros2 launch tb3_nav_assessment rgbd_simulation.launch.py
```

YOLOv5n 检测：

```bash
ros2 launch tb3_nav_assessment visual_perception.launch.py \
  mode:=detection show_result:=true
```

自主前沿探索：

```bash
ros2 launch tb3_nav_assessment autonomous_exploration.launch.py \
  map_save_path:=$HOME/assessment_nav_ws/maps/autonomous_exploration
```

VirtualBox 图形性能不足时，可在探索命令中加入 `gui:=false rviz:=false`。

## 自动验证

主 package 静态测试：

```bash
python3 -m pytest test -q
```

A* package 静态测试：

```bash
python3 -m pytest \
  optional_challenges/tb3_astar_planner/test -q
```

运行态检查：

```bash
ros2 run tb3_nav_assessment simulation_smoke_check
ros2 run tb3_nav_assessment rgbd_smoke_check
ros2 run tb3_nav_assessment perception_smoke_check
ros2 run tb3_nav_assessment exploration_smoke_check
```

Ubuntu 最终验收记录为主 package `30 passed, 1 skipped`，A* package
`6 passed`。现场截图按模块收录在
[`docs/evidence/`](docs/evidence/README.md)。

## 附加挑战文档

1. [自定义 RGB-D 相机](docs/challenges/rgbd_camera_challenge.md)
2. [Nav2 A* 全局规划器](optional_challenges/tb3_astar_planner/docs/astar_planner_challenge.md)
3. [YOLOv5n 视觉感知](docs/challenges/visual_perception_challenge.md)
4. [SLAM + Nav2 自主探索](docs/challenges/autonomous_exploration_challenge.md)

## 许可证

项目代码使用 Apache-2.0。第三方模型和演示素材说明见
[`models/THIRD_PARTY_NOTICES.md`](models/THIRD_PARTY_NOTICES.md)。
