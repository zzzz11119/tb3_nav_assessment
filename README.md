# TurtleBot3 第一阶段自主导航考核

本 package 固定使用 **Ubuntu 22.04 + ROS2 Humble + Gazebo Classic +
TurtleBot3 Burger**。Gazebo 自建 world、传感器验证、仿真 launch 和自动冒烟
检查已经完成。当前主线为 SLAM Toolbox 建图、Cartographer 对比、Nav2 与
Python 多目标导航。

附加挑战一已按“独立入口、不改基础仿真”的原则加入：自定义 Burger RGB-D
URDF/Xacro、独立 Gazebo launch、RGB/深度/点云接口和自动验收节点。现场步骤见
[rgbd_camera_challenge.md](docs/rgbd_camera_challenge.md)。

附加挑战二因为使用 C++ / `pluginlib` 实现，保持为独立 package，
源码一并归档在
[`optional_challenges/tb3_astar_planner`](optional_challenges/tb3_astar_planner/README.md)
中。它只在专用启动入口中
将 A* 注入 Nav2，不修改本 package 的稳定仿真入口。

附加挑战三复用 RGB-D 相机，以独立
`visual_perception.launch.py` 启动 COCO 预训练 YOLOv5n ONNX 检测。
新入口同时提供 `mode:=original` 和 `mode:=detection`，原图像 topic
不会被覆盖。完整说明见
[visual_perception_challenge.md](docs/visual_perception_challenge.md)。

附加挑战四使用独立 `autonomous_exploration.launch.py` 串联原始自建 world、
SLAM Toolbox、Nav2 与自研 frontier-based 探索节点。它会自动选择安全可达
前沿、处理失败目标、发布状态/可视化，并在无前沿稳定后保存最终地图。完整说明见
[autonomous_exploration_challenge.md](docs/autonomous_exploration_challenge.md)。

SLAM 对比实验的场景梯度、公平输入、评价指标和停止条件见
[slam_benchmark_plan.md](docs/slam_benchmark_plan.md)。研究配置必须与最终考核
配置分离，不能为了增加算法数量破坏稳定演示。

## 第 1 周当前交付

- 12 m × 8 m 封闭世界和完整碰撞边界
- 西、东两个连通区域，中间门宽 2.4 m
- 3 个不同形状/尺寸的静态障碍物
- TurtleBot3 Burger 出生点 `(-4.0, -2.5)`
- 不依赖本机绝对路径的 `simulation.launch.py`
- `/scan`、`/odom`、`/clock`、TF 与 LaserScan 频率检查节点
- world/package 静态验收测试
- 实验日志、验收记录和证据清单

## 1. 环境基线

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 pkg prefix gazebo_ros
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix turtlebot3_bringup
ros2 pkg prefix turtlebot3_teleop
```

若 TurtleBot3 仿真 package 不存在，按 ROBOTIS Humble 官方分支安装：

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

每个新终端都要 source ROS2 和 TurtleBot3 overlay。

## 2. 先验收官方 empty world

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 topic list | grep -E '^/(scan|odom|tf|tf_static|clock)$'
ros2 topic hz /scan
```

再开一个终端启动键盘控制，持续运行并移动机器人至少 10 分钟：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

只有机器人可移动、5 个关键 topic 正常且 Gazebo 无持续报错，才在验收记录中
将官方基线标记为通过。

## 3. 构建本项目

在本仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

rosdep install \
  --from-paths . \
  --ignore-src -r -y
colcon build \
  --symlink-install \
  --base-paths .
source install/setup.bash
```

冷启动复现时，先删除仓库根目录的 `build/`、`install/`、`log/`，再执行上述
构建命令。这三个目录已被仓库根目录的 `.gitignore` 排除。

## 4. 启动自建世界

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch tb3_nav_assessment simulation.launch.py
```

无图形界面启动：

```bash
ros2 launch tb3_nav_assessment simulation.launch.py gui:=false
```

查看机器人与 LaserScan：

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

键盘遍历两个区域：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## 5. 自动验证传感器和 TF

保持仿真运行，在已 source 项目 overlay 的新终端执行：

```bash
ros2 run tb3_nav_assessment simulation_smoke_check \
  --ros-args -p timeout_sec:=12.0 -p min_scan_hz:=3.0
```

检查通过必须同时满足：

- `/scan` 有有限距离数据且频率不低于配置阈值
- `/odom`、`/clock` 持续发布
- `/tf`、`/tf_static` 有变换
- `odom → base_footprint → base_scan` 可查询

命令退出码为 `0` 才算通过；失败时会逐项显示缺失内容。

## 6. 静态测试

不启动 ROS2 也可检查 package 和 world 的结构：

```bash
python3 -m pytest projects/tb3_nav_assessment/test -q
```

测试会验证封闭外墙、两个区域、门宽、3 个不同障碍物、碰撞/可视几何、
Python 语法以及安装资源规则。

## 7. World 布局

| 元素 | 中心/范围（m） | 尺寸（m） |
|---|---|---|
| 外边界 | `x=[-6, 6]`, `y=[-4, 4]` | `12 × 8` |
| 中间隔墙 | `x=0`，上下两段 | 中央门宽 `2.4` |
| 小立方体 | `(-3.2, 2.2)` | `0.6 × 0.6 × 0.6` |
| 长方体 | `(3.1, 2.2)` | `1.2 × 0.7 × 0.8` |
| 圆柱体 | `(3.2, -2.2)` | 半径 `0.45`，高 `0.7` |
| Burger 出生点 | `(-4.0, -2.5)` | launch 参数可覆盖 |

## 8. 第 1 周验收与证据

按 [week01_acceptance.md](docs/week01_acceptance.md) 逐项执行。证据文件统一放入
本 package 的现场截图位于 `docs/evidence/`，不要提交大型 rosbag
数据库。

常见问题：

- `TURTLEBOT3_MODEL` 不是 `burger`：重新 export 后启动。
- 找不到 package：检查三个 setup 文件是否都已 source。
- `/clock` 不动：确认 Gazebo 没暂停，并检查 `use_sim_time:=true`。
- TF 缺失：先检查 `robot_state_publisher` 是否启动，再检查 Gazebo 插件日志。
- 机器人出生后弹飞/穿模：确认没有修改默认出生点到墙体或障碍物内部。

## 9. 附加挑战一：自定义 RGB-D 相机

该入口不会生成原始 SDF Burger，而是用自定义 Xacro 构造
`robot_description`，因此不要与 `simulation.launch.py` 同时启动：

```bash
ros2 launch tb3_nav_assessment rgbd_simulation.launch.py
```

保持仿真运行，在另一个已 source 的终端自动检查 RGB、深度、相机内参、点云
和相机 TF：

```bash
ros2 run tb3_nav_assessment rgbd_smoke_check \
  --ros-args -p timeout_sec:=15.0 -p min_image_hz:=5.0
```

期望看到以下接口：

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`
- `/camera/depth/points`

终端打印 `RGB-D OPTIONAL CHALLENGE CHECK: PASS` 且退出码为 `0` 才算现场
验收完成。完整参数、RViz 检查方法和证据清单见附加挑战文档。

## 10. 附加挑战二：Nav2 自定义 A* 规划器

A* 以独立 ROS 2 package 归档。克隆本仓库后，先将它复制到 workspace
的同级 `src` 目录：

```bash
cp -r optional_challenges/tb3_astar_planner \
  ~/assessment_nav_ws/src/tb3_astar_planner
cd ~/assessment_nav_ws
colcon build --packages-select tb3_astar_planner
```

插件实现、Nav2 注入方式和现场验收数据见
[`astar_planner_challenge.md`](optional_challenges/tb3_astar_planner/docs/astar_planner_challenge.md)。

## 11. 附加挑战三：预训练目标检测

原 RGB-D 模式：

```bash
ros2 launch tb3_nav_assessment visual_perception.launch.py mode:=original
```

检测模式和结果窗口：

```bash
ros2 launch tb3_nav_assessment visual_perception.launch.py \
  mode:=detection show_result:=true
```

运行态自动验收：

```bash
ros2 run tb3_nav_assessment perception_smoke_check \
  --ros-args -p timeout_sec:=20.0 -p require_detection:=true
```

检测节点保留 `/camera/color/image_raw`，另外发布
`/perception/yolo/annotated_image` 和 `/perception/yolo/detections`。
默认的离线海报演示使验收不依赖现场网络；检测实际物体时加
`demo_target:=false`。

## 12. 附加挑战四：自主环境探索

一键启动自建 world、在线 SLAM、无 AMCL 的 Nav2 导航和前沿探索：

```bash
ros2 launch tb3_nav_assessment autonomous_exploration.launch.py \
  map_save_path:=$HOME/assessment_nav_ws/maps/autonomous_exploration
```

运行中健康检查：

```bash
ros2 run tb3_nav_assessment exploration_smoke_check --ros-args \
  -p timeout_sec:=60.0 -p require_complete:=false
```

完成态验收会等待无前沿状态，并检查成功目标数和自动保存的地图：

```bash
ros2 run tb3_nav_assessment exploration_smoke_check --ros-args \
  -p timeout_sec:=900.0 -p require_complete:=true \
  -p min_known_area_m2:=60.0 -p min_successful_goals:=2
```

算法只把已知自由栅格作为 Nav2 目标和路径区域，不会让规划器穿越未知区。
`/exploration/frontiers` 可在 RViz 显示前沿与候选目标，
`/exploration/status` 输出目标统计、地图面积、完成原因和保存路径。

Ubuntu 22.04 / ROS 2 Humble 现场验收已通过：机器人自主穿过中部
通道并浏览全局，最终目标统计为 25/22/3（发送/成功/失败），
完成地图 YAML/PGM 自动保存，最终 `exploration_smoke_check` PASS。
证据位于 `docs/evidence/autonomous_exploration/`。
