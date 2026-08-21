# 实验日志

## 2026-07-24｜冲刺第 1 周工程启动

- 对应里程碑：自建 Gazebo 世界
- 环境：当前工作区完成静态工程准备；运行环境固定为 Ubuntu 22.04 /
  ROS2 Humble / Gazebo Classic / TurtleBot3 Burger

### 问题

如何建立一个不依赖本机绝对路径、可供 SLAM 与 Nav2 后续复用的最小仿真工程？

### 假设

使用 package share 路径加载 world，复用 TurtleBot3 Humble 官方
`robot_state_publisher.launch.py` 和 `spawn_turtlebot3.launch.py`，可以避免复制
机器人模型并减少版本偏差。12 m × 8 m 边界和 2.4 m 中央门应给 Burger 留出
足够通行空间。

### 本次变更

- 创建 `ament_python` package 和资源安装规则。
- 创建封闭 world、两个区域和三个不同障碍物。
- 创建自定义 simulation launch。
- 创建传感器/TF 冒烟检查节点。
- 创建静态验收测试与现场证据清单。

### 当前结果

静态验收测试 `7 passed`，package manifest 与 SDF world 均通过 XML 语法检查，
setup metadata 可读取为 `tb3_nav_assessment 0.1.0`。当前机器没有 ROS2、
colcon、Gazebo，因此尚需在 Ubuntu ROS2 Humble 主机上执行：

1. 官方 empty world 10 分钟基线。
2. 自建 world 构建与启动。
3. teleop 全区域遍历。
4. 自动冒烟检查。
5. 冷启动与清构建复现。

### 下一步

在 Ubuntu 主机按根目录 README 的构建、快速运行和自动验证章节执行，保存版本、topic、TF、world 全景和
检查日志；若中央门或出生点存在物理问题，只调整 world 坐标后重新验收。

## 2026-08-13｜附加挑战一 RGB-D 相机工程实现

- 对应任务：老师选做题“自定义相机”
- 原则：与基础 `simulation.launch.py` 隔离，避免破坏原始 Burger 仿真基线

### 本次变更

- 新增 Burger RGB-D URDF/Xacro，相机包含质量、惯量、可视和碰撞几何。
- 补回自定义 URDF 生成 Gazebo 实体所需的差速驱动、关节状态、IMU 和激光插件。
- 新增独立 `rgbd_simulation.launch.py`，从 `robot_description` 生成机器人。
- 约定 RGB、深度、两组内参和点云 topic，并建立 camera optical TF。
- 新增 `rgbd_smoke_check`，自动检查数据有效性、频率与 TF。
- 新增挑战说明、现场验收证据清单和静态测试。

### 当前结果

代码与资源结构已完成；当前工作区不是 Ubuntu ROS2 Humble + Gazebo Classic
运行主机，因此运行态结果必须以现场 `rgbd_smoke_check` 的 PASS 日志、Gazebo
截图、RViz 图像/点云和 TF 输出为准。在这些证据产生前，进度标为“实现完成、
待现场验收”，不写成已通过。

## 2026-08-16｜附加挑战二 A* 全局规划器工程实现

- 对应任务：老师选做题“自定义规划器”
- 原则：独立 C++ package 和 Nav2 launch，不覆盖基础仿真或稳定导航参数

### 问题与假设

直接把 A* 写成普通脚本只能展示算法，不足以证明“集成到导航栈”。采用
`nav2_core::GlobalPlanner` 插件接口，由 planner server 通过 `pluginlib`
加载，才能让 RViz 目标、`NavigateToPose` 和 `ComputePathToPose` 使用同一
自定义规划器。

### 本次变更

- 新增独立 `tb3_astar_planner` / `ament_cmake` package 与 plugin XML。
- 实现基于 Nav2 global costmap 的 A* 搜索、父链回溯和 Path 生成。
- 支持 4/8 邻域、未知区域策略、inflation cost 惩罚和目标容差。
- 对角移动必须两个正交侧单元均可通行，防止路径穿过墙角。
- 新增独立 Nav2 launch，在临时文件中只覆盖 planner server 配置。
- 新增 `astar_smoke_check`，请求命名 `AStar` 插件规划必须绕墙的路径，
  并检查地图占用、绕行长度、终点误差与 action 结果。

### 当前结果

ROS 2 Humble 的 `GlobalPlanner`、代价地图和异常接口已按官方头文件核对。
本地静态验收通过；当前机器无 ROS 2 / Nav2 运行环境，因此仍保持
“实现完成、待 Ubuntu 现场验收”状态。最终通过必须同时有插件加载日志、
RViz 绕墙路径、`A* OPTIONAL CHALLENGE CHECK: PASS` 与完整导航录屏。

## 2026-08-19｜附加挑战二 Ubuntu 现场验收

- 环境：Ubuntu 22.04 VirtualBox、ROS 2 Humble、Gazebo Classic、TurtleBot3 Burger
- workspace：`/home/zz/assessment_nav_ws`

### 验收结果

- `colcon build --packages-select tb3_astar_planner` 成功，静态验收 `6 passed`。
- planner server 加载 `tb3_astar_planner/AStarPlanner`，lifecycle 状态为 `active [3]`。
- 命名 `AStar` 的 `ComputePathToPose` 验收 PASS：187 个 pose、11.32 m、
  终点误差 0.000 m，路径不经过占用单元且确实绕过直线障碍。
- TurtleBot3 默认行为树请求的 `GridBased` ID 同样 PASS；该 ID 指向同一
  自定义 A* 实现，不是 Navfn 回退。
- RViz 实际导航显示 `Feedback: reached`，剩余距离 0.05 m、用时 7 s、
  recoveries 0，机器人能在目标处停止。

### 现场兼容修正

1. 系统的 Humble `nav2_behaviors` 使用 `nav2_behaviors/ClassName`，而较新
   TurtleBot3 YAML 使用 `nav2_behaviors::ClassName`；启动时已自动规范化。
2. 默认 NavigateToPose 行为树请求 `GridBased`；已将 `AStar` 和
   `GridBased` 两个 ID 同时映射到自定义 A* 插件。
3. 新版 TurtleBot3 YAML 遗漏 `use_sim_time`，导致系统时间路径与仿真
   TF 相差；启动时已向所有 Nav2 `ros__parameters` 注入仿真时间。

### 证据

- `notes/assets/astar_challenge/astar_named_planner_pass_2026-08-19.png`
- `notes/assets/astar_challenge/gridbased_alias_pass_2026-08-19.png`
- `notes/assets/astar_challenge/rviz_navigation_reached_2026-08-19.png`

## 2026-08-19｜附加挑战三预训练视觉感知实现

- 对应任务：老师选做题“视觉感知”
- 原则：复用 RGB-D，使用真实预训练模型，保留原始模式和原图像 topic

### 问题与选型

规划要求“预训练目标检测”，所以未用 HSV 颜色分割代替模型。为避免
Ubuntu 虚拟机安装 PyTorch 和现场下载权重，将官方 COCO 预训练 YOLOv5n
权重导出为 ONNX opset 12，用 OpenCV DNN 完成 letterbox、推理、类别置信度
和 NMS 后处理。

### 本次变更

- 新增 `yolo_object_detector`，读取 RGB-D，发布带框图像与 JSON 检测。
- 检测框内有效深度中位数作为目标距离，无同步深度时仍可做 RGB 检测。
- 新增 `visual_perception.launch.py`，支持 `original` / `detection` 双模式。
- 检测模式可按条件生成官方 COCO 海报，便于离线重复验收。
- 新增 `perception_smoke_check`，检查原 RGB、带框图像、JSON、频率与至少一个检测。
- 新增模型来源/SHA-256 说明、专用文档和静态/真模型推理测试。

### 当前结果

本地使用打包的 ONNX 对官方演示图做真实 OpenCV DNN 推理，能检出
`bus` 和多个 `person`，置信度、边界框和标注图测试通过。Ubuntu 现场结果见
下节，当前状态已更新为“实现完成、现场验收通过”。

## 2026-08-20｜附加挑战三 Ubuntu 现场验收

- 环境：Ubuntu 22.04 VirtualBox、ROS 2 Humble、Gazebo Classic、TurtleBot3 Burger
- workspace：`/home/zz/assessment_nav_ws`

### 验收结果

- `colcon build --symlink-install --packages-select tb3_nav_assessment` 成功。
- Ubuntu 静态测试为 `13 passed, 1 skipped`；跳过项不影响运行功能。
- `mode:=original gui:=false` 成功生成 RGB-D Burger；只出现原始
  `/camera/color/image_raw`，没有 `/perception/yolo/*`，证明原模式隔离保留。
- 检测画面实际检出 1 个 `bus` 和 3 个 `person`；示例距离约 `1.67 m`，
  单帧推理约 `210 ms`。
- 20 秒自动验收观测 raw=51、annotated=36、valid summaries=36，标注频率
  `1.89 Hz`，检测标签为 `bus, person`，全部检查 PASS。
- 最终结果：`VISUAL PERCEPTION OPTIONAL CHALLENGE CHECK: PASS`。

### 现场兼容修正

1. 将 YOLO26n 替换为官方 YOLOv5n v7.0、ONNX opset 12，并常量折叠，
   兼容 Ubuntu 自带 OpenCV 4.5.4。
2. 安装器改用 Python AST 定位 `data_files`，兼容右括号同行的旧
   `setup.py`，同时规范新增资源行格式。
3. Humble 中 Node 的 `subscriptions` 为只读属性；验收脚本改用私有变量名。
4. Humble 日志调用位置不能动态切换 severity；改为独立 INFO/ERROR 分支。
5. 标注图验收订阅使用 reliable QoS，避免虚拟机中大图消息被丢弃。
6. VirtualBox 的 Gazebo GUI 无响应时使用 `gui:=false`；如仿真暂停，调用
   `/unpause_physics` 后相机和检测流即可恢复。

### 证据

- `docs/evidence/tests/ubuntu_static_tests_pass_2026-08-20.png`
- `docs/evidence/perception/ubuntu_original_mode_spawn_pass_2026-08-20.png`
- `docs/evidence/perception/ubuntu_original_mode_topic_pass_2026-08-20.png`
- `docs/evidence/perception/ubuntu_yolov5_rgbd_detection_2026-08-20.png`
- `docs/evidence/perception/ubuntu_perception_smoke_pass_2026-08-20.png`

## 2026-08-20｜附加挑战四前沿自主探索实现

- 对应任务：老师选做题“自主环境探索”
- 原则：复用稳定自建 world，把在线 SLAM、Nav2 和自研 frontier-based 节点
  放入独立入口，不改基础仿真和前三项挑战

### 算法与状态机

- 前沿定义为已知自由栅格且四邻域含未知栅格；使用八邻域聚类去除零散噪声。
- 从机器人当前栅格在已知自由空间做可达性搜索，对角移动禁止切过障碍墙角。
- 候选目标同时满足占用栅格安全距离，朝向未知区，并按信息增益减路径距离排序。
- 通过 `NavigateToPose` 依次执行目标；拒绝、失败和超时目标进入带过期时间的
  空间黑名单，避免原地重试死循环。
- 无前沿稳定 15 秒后完成；若建图已超过 80 m²，且仅剩不超过
  40 个无可达安全目标的 SLAM 边缘单元，同样经过 15 秒保持期后
  正常完成；完成后节点直接写出标准 trinary PGM/YAML 地图。

### 工程变更

- 新增纯算法模块 `frontier.py`、ROS 节点 `frontier_explorer.py` 和运行验收节点。
- 新增 SLAM/探索参数与 `autonomous_exploration.launch.py`；Nav2 使用
  `navigation_launch.py`，由 SLAM Toolbox 提供实时 `/map` 和 `map → odom`，
  不启动 AMCL 或静态 map server。
- Nav2 启动时从已验证的 TurtleBot3 Burger 参数生成临时配置，只注入
  `use_sim_time`、实时地图更新、未知空间跟踪和“禁止穿越未知区”等必要项。
- 兼容已验证环境中的“新版 TurtleBot3 参数 + Humble Nav2”组合：Navfn 和
  behavior 插件名在启动时统一转换为 Humble 使用的 `/` 形式。
- 新增可视化、状态 JSON、两阶段自动验收、单元测试、复现文档和 Ubuntu 增量包。

### Ubuntu 现场结果（2026-08-20 至 2026-08-21）

- Ubuntu 22.04 / ROS 2 Humble 上增量安装、构建通过，挑战四专项测试
  `10 passed`，当前工区全 package 测试 `28 passed`。
- 首轮现场测试发现 Nav2 容差内的 0.10 m 近距离目标会导致机器人
  摆动但无位移；加入 0.80 m 最小目标距离和聚类内远端安全点选择后，
  机器人产生连续位移并自主穿过中部通道。
- 运行中验收 PASS：已知面积 33.00 m²、前沿 416、目标 7，成功 5、
  失败 1，地图、SLAM 和前沿处理全部正常。
- 最终自主探索完成：发送 25 个目标，成功 22、失败 3；仅剩 6 个
  无可达安全目标的残余前沿，触发高覆盖率完成条件。
- 自动保存
  `/home/zz/assessment_nav_ws/maps/autonomous_exploration.yaml`
  和 `.pgm`；最终检查中探索完成、最小成功目标、YAML 和图像文件全部
  PASS，总结为 `AUTONOMOUS EXPLORATION OPTIONAL CHALLENGE CHECK: PASS`。

### 证据

- `docs/evidence/autonomous_exploration/ubuntu_autonomous_motion_2026-08-20.png`
- `docs/evidence/autonomous_exploration/ubuntu_running_smoke_pass_2026-08-20.png`
- `docs/evidence/autonomous_exploration/ubuntu_completion_and_map_save_2026-08-20.png`
- `docs/evidence/autonomous_exploration/ubuntu_final_smoke_pass_2026-08-21.png`
