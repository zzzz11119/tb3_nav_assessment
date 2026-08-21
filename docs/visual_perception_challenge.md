# 附加挑战三｜RGB-D + 预训练 YOLO 目标检测

## 1. 验收目标

对应老师原题“视觉感知：利用机器人搭载的摄像头，部署简单的目标
检测算法，展示结果”。本实现的通过标准是：

1. 复用附加挑战一的 Burger RGB-D 相机，订阅原始 RGB 图像。
2. 使用 COCO 预训练 YOLOv5n 模型做神经网络推理，不用颜色分割
   代替预训练模型。
3. 发布带类别、置信度、边界框和 RGB-D 距离的结果图像。
4. 发布机器可读的 JSON 检测摘要，方便后续语义导航扩展。
5. 提供可离线演示的 COCO 海报目标与一键自动验收。
6. 原始模式和原始图像 topic 必须保留，检测节点不能覆盖它们。

## 2. 隔离设计与模式

| 入口 | 作用 | 是否启动检测 |
|---|---|---|
| `simulation.launch.py` | 最初稳定的原版 Burger 基线 | 否 |
| `rgbd_simulation.launch.py` | 附加挑战一 RGB-D 基线 | 否 |
| `visual_perception.launch.py mode:=original` | 通过新入口运行原 RGB-D 模式 | 否 |
| `visual_perception.launch.py mode:=detection` | RGB-D + YOLO + 结果发布 | 是 |

`/camera/color/image_raw` 始终是原始图像。检测节点只读取它，标注图像发往
独立的 `/perception/yolo/annotated_image`，因此 Nav2、RGB-D 验收和原有
演示都不会受到 topic 重映射或图像改写的影响。

## 3. 模型与推理链

- 模型：YOLOv5n，COCO 80 类预训练权重。
- 部署格式：ONNX opset 12，固定 `640 × 640`，原始输出 `1 × 25200 × 85`。
- 运行时：OpenCV DNN CPU，不需要 PyTorch、Ultralytics Python 包或现场下载。
- 前处理：保持宽高比的 letterbox、BGR 转 RGB、`1/255` 归一化。
- 后处理：目标置信度与类别分数相乘，阈值 `0.25`，NMS IoU 阈值 `0.45`。
- 距离：在检测框对应的深度 ROI 内取有效像素中位数。

模型、官方演示图、来源、转换参数和 SHA-256 均已放入
`models/THIRD_PARTY_NOTICES.md`。

## 4. 数据接口

| 数据 | Topic | 类型 |
|---|---|---|
| 原始 RGB | `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| 深度 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| 带框结果 | `/perception/yolo/annotated_image` | `sensor_msgs/msg/Image` |
| 结构化检测 | `/perception/yolo/detections` | `std_msgs/msg/String` (JSON) |

JSON 中包含模型名、时间戳、推理耗时、检测数量，以及每个目标的
`class_id`、`label`、`confidence`、`bbox_xyxy` 和 `distance_m`。

## 5. 构建

安装 package.xml 声明的 ROS/OpenCV 依赖后重新构建：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

rosdep install \
  --from-paths projects/tb3_nav_assessment \
  --ignore-src -r -y
colcon build \
  --symlink-install \
  --base-paths projects/tb3_nav_assessment
source install/setup.bash
```

构建后确认模型已经安装：

```bash
ros2 pkg prefix tb3_nav_assessment
find install/tb3_nav_assessment/share/tb3_nav_assessment/models \
  -name yolov5n.onnx -size +7M
```

## 6. 启动与展示

原 RGB-D 模式（不加载 YOLO，不生成演示海报）：

```bash
ros2 launch tb3_nav_assessment visual_perception.launch.py mode:=original
```

检测模式，并自动打开结果窗口：

```bash
ros2 launch tb3_nav_assessment visual_perception.launch.py \
  mode:=detection show_result:=true
```

默认会在机器人正前方生成一个官方 COCO 演示海报，用于无网络验收。
要检测仿真世界或摄像头前真实放置的物品时，关闭演示海报：

```bash
ros2 launch tb3_nav_assessment visual_perception.launch.py \
  mode:=detection demo_target:=false show_result:=true
```

也可不启动查看窗口，只用命令检查：

```bash
ros2 topic hz /perception/yolo/annotated_image
ros2 topic echo /perception/yolo/detections --once
```

## 7. 一键验收

保持 `mode:=detection` 仿真运行，在新终端执行：

```bash
ros2 run tb3_nav_assessment perception_smoke_check \
  --ros-args \
  -p timeout_sec:=20.0 \
  -p min_annotated_hz:=1.0 \
  -p require_detection:=true
```

检查同时验证：

- 原始 RGB 话题仍持续发布；
- 带框图像和 JSON 结果持续发布；
- 结果的模型身份和字段有效；
- 结果图频率达到阈值；
- 预训练模型至少检出一个目标。

全部通过时打印：

```text
VISUAL PERCEPTION OPTIONAL CHALLENGE CHECK: PASS
```

并以返回码 `0` 退出。

## 8. 证据清单

- Gazebo 中 Burger RGB-D 相机与检测演示目标的同框截图。
- `rqt_image_view` 中带类别、置信度、框和距离的结果。
- 原始图像与检测结果话题同时存在的 `ros2 topic list` 输出。
- 一条 `/perception/yolo/detections` JSON 样例。
- `perception_smoke_check` 全 PASS 终端截图。
- `mode:=original` 启动时无 YOLO 节点的回归截图。

只有上述运行态证据齐全后，进度看板才从“实现完成、待 Ubuntu 现场
验收”改为“已验收”。

## 9. 已知边界

- COCO 预训练模型不能稳定识别由简单几何体构成的 Gazebo 障碍物，因此
  海报只用于可重复验收，不代表训练数据。
- CPU 虚拟机上不追求原始 15 Hz 图像逐帧推理；`max_processing_hz` 默认为
  5 Hz，原始摄像头话题仍按原频率发布。
- 结果距离是检测框内深度中位数；对于平面海报，它表示海报平面
  距离，不是照片中真实物体的三维距离。
