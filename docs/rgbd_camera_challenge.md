# 附加挑战一｜TurtleBot3 Burger 自定义 RGB-D 相机

## 1. 验收目标

本挑战独立于基础 `simulation.launch.py`，不覆盖已经稳定的原始 Burger 模型。
通过标准如下：

1. 自定义 URDF/Xacro 在 Burger 上增加具有质量、惯量、可视和碰撞几何的相机。
2. `rgbd_simulation.launch.py` 启动自建 world、发布机器人模型并在 Gazebo 生成机器人。
3. 原 `/cmd_vel`、`/odom`、`/scan`、`/imu` 和 TF 功能保留。
4. 发布 RGB、深度、两组 `camera_info` 和有组织点云。
5. `base_link → camera_rgb_optical_frame` TF 可查询。
6. 自动验收命令退出码为 `0`。

## 2. 设计

- 机器人：TurtleBot3 Burger，复用 Humble 官方基础 URDF。
- 安装位姿：相机中心相对 `base_link` 为 `(0.055, 0, 0.205) m`。
- 外壳：`0.040 × 0.090 × 0.030 m`，质量 `0.055 kg`。
- 图像：`640 × 480`、15 Hz、水平视场角 60°。
- 深度范围：`0.12–5.0 m`。
- 坐标系：包含 RGB、depth 及符合 REP-103 的 optical frames。

15 Hz 是对演示质量和仿真负载的折中。附加题使用单独 launch，基础导航演示需要
稳定性时仍可继续运行原 `simulation.launch.py`。

## 3. 启动

完成 README 中的构建和环境 source 后运行：

```bash
ros2 launch tb3_nav_assessment rgbd_simulation.launch.py
```

无界面运行：

```bash
ros2 launch tb3_nav_assessment rgbd_simulation.launch.py gui:=false
```

可用原键盘节点检查物理运动和相机随动：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## 4. 数据接口

| 数据 | Topic | 期望类型 |
|---|---|---|
| RGB 图像 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| RGB 内参 | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 深度图像 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| 深度内参 | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 点云 | `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` |

现场检查：

```bash
ros2 topic list | grep '^/camera/'
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 run tf2_ros tf2_echo base_link camera_rgb_optical_frame
```

在 RViz2 中将 Fixed Frame 设为 `odom`，分别添加 Image、DepthCloud 或
PointCloud2 显示，并选择上表中的 topic。

## 5. 一键验收

保持仿真运行，在新终端执行：

```bash
ros2 run tb3_nav_assessment rgbd_smoke_check \
  --ros-args -p timeout_sec:=15.0 -p min_image_hz:=5.0
```

验收节点同时检查：

- 原 `/scan` 与 `/odom` 接口仍正常发布；
- RGB/深度图像尺寸、编码、帧名和消息频率；
- 两组相机内参与点云有效性；
- `base_link → camera_rgb_optical_frame` TF；
- 所有检查均通过时打印 `RGB-D OPTIONAL CHALLENGE CHECK: PASS` 并返回 `0`。

## 6. 证据清单

- Gazebo 中 Burger 与相机外壳的近景和世界全景。
- RViz 中 RGB 图像、深度图或点云。
- `ros2 topic list` 与两个 `topic hz` 输出。
- `tf2_echo` 输出。
- `rgbd_smoke_check` 全部 PASS 的终端截图。
- teleop 前进、转向后相机仍固定在机器人上的短视频。

只有上述现场证据齐全后，进度看板中的 RGB-D 挑战才能由“实现完成、待现场验收”
改为“已验收”。
