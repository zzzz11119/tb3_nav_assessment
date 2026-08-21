# 附加挑战四｜SLAM + Nav2 前沿自主探索与完全建图

## 1. 验收目标

本挑战新增独立 `autonomous_exploration.launch.py`，不改动已经稳定的
`simulation.launch.py`、RGB-D 和视觉感知入口。一次启动完成：

```text
自建封闭 Gazebo world
        ↓  /scan、/odom、TF、/clock
SLAM Toolbox 在线异步建图（发布 /map 与 map → odom）
        ↓  实时 OccupancyGrid
frontier_explorer 提取、聚类、筛选并排序前沿
        ↓  NavigateToPose action
Nav2 在已知自由空间内规划和避障
        ↓  地图持续扩大
无前沿稳定 15 秒 → 探索完成 → 自动保存 PGM + YAML
```

通过标准为：

1. 机器人没有人工发送目标点，能连续自主选择并到达多个前沿。
2. 前沿目标必须位于已知自由空间、从机器人当前位置可达，并远离占用栅格。
3. Nav2 只在已知区域规划，不允许为了接近前沿直接穿越未知区域或障碍物。
4. 目标失败、被拒绝或超时后不会死循环，会临时加入黑名单并选择其他前沿。
5. 封闭世界中无前沿持续 15 秒后输出完成状态，并自动保存地图。
6. 自动验收输出
   `AUTONOMOUS EXPLORATION OPTIONAL CHALLENGE CHECK: PASS`，退出码为 `0`。

## 2. 前沿算法

占据栅格含三类单元：未知 `-1`、自由 `0—20`、占用 `65—100`。算法每次收到
SLAM 地图后执行：

1. 找出“自身已知自由、四邻域至少一个未知”的前沿单元。
2. 对前沿单元做八邻域连通域聚类，过滤少于 8 个栅格的噪声前沿。
3. 从机器人栅格开始，只通过已知自由空间做可达性搜索；对角移动不能切墙角。
4. 过滤距占用栅格不足 `0.22 m`，或距机器人不足 `0.80 m` 的候选；后者避免
   SLAM 在机器人附近形成未知小洞时反复发送“原地成功”目标。
5. 在每个聚类中选择路径距离最远的安全可达单元，朝向该目标四邻域中的未知
   区域，确保目标能产生实际位移和新增观测。
6. 以“信息增益 - 路径距离”打分，优先选择能扩展更多地图且代价合理的目标。
7. 每个成功目标后至少等待一版新地图，再重新选点，避免在旧地图上重复发送
   已经到达的前沿。

算法核心在 `tb3_nav_assessment/frontier.py`，不依赖 ROS，可在普通 Python
环境做确定性单元测试。ROS 节点只负责地图/TF 接入、Nav2 action 状态机、
可视化、失败恢复和地图保存。

## 3. 构建

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

cd ~/assessment_nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select tb3_nav_assessment
source install/setup.bash
```

静态测试：

```bash
python3 -m pytest \
  src/tb3_nav_assessment/test/test_autonomous_exploration_challenge.py -q
```

## 4. 一键启动

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/assessment_nav_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch tb3_nav_assessment autonomous_exploration.launch.py \
  map_save_path:=$HOME/assessment_nav_ws/maps/autonomous_exploration
```

VirtualBox 中 Gazebo/RViz 图形性能不足时：

```bash
ros2 launch tb3_nav_assessment autonomous_exploration.launch.py \
  gui:=false rviz:=false \
  map_save_path:=$HOME/assessment_nav_ws/maps/autonomous_exploration
```

`start_simulation:=false` 可复用已经运行的同一台 TurtleBot3 仿真；不要同时
启动两个 Gazebo 或两个 `slam_toolbox`。

## 5. 观察状态

另开一个已 source 的终端：

```bash
ros2 topic echo /exploration/status
```

状态是 JSON，关键字段包括：

| 字段 | 含义 |
|---|---|
| `state` | 等待、选点、导航、确认完成、完成、停滞或错误 |
| `frontier_cells` | 当前原始前沿栅格数 |
| `frontier_clusters` | 当前连通前沿数量 |
| `eligible_clusters` | 通过尺寸、可达性和安全距离筛选的数量 |
| `completion_candidate` | 当前是否已满足无前沿或高覆盖率残余前沿完成条件 |
| `goals_sent/succeeded/failed` | Nav2 前沿目标统计 |
| `known_area_m2` | 当前已知栅格面积，用于观察地图增长而非替代完整性判断 |
| `saved_map_yaml/image` | 完成后自动保存的两个地图文件 |

RViz 增加 `MarkerArray`，topic 选 `/exploration/frontiers`：蓝点是前沿，绿色
箭头是当前首选目标，其他橙色箭头是备选目标。

## 6. 两阶段自动验收

启动约 20—40 秒后先检查系统正在真实处理地图和前沿：

```bash
ros2 run tb3_nav_assessment exploration_smoke_check --ros-args \
  -p timeout_sec:=60.0 \
  -p require_complete:=false \
  -p min_known_area_m2:=8.0
echo $?
```

让机器人继续探索。最终验收会等待“无前沿”完成状态，检查成功目标数、地图内容
以及自动保存的 YAML/PGM：

```bash
ros2 run tb3_nav_assessment exploration_smoke_check --ros-args \
  -p timeout_sec:=900.0 \
  -p require_complete:=true \
  -p min_known_area_m2:=60.0 \
  -p min_successful_goals:=2 \
  -p require_saved_map:=true
echo $?
```

只有打印 `PASS` 且退出码为 `0` 才算自动验收通过。`60 m²` 是针对当前
`12 m × 8 m` 封闭 world 的现场阈值；如果 SLAM 输出栅格边界裁剪方式不同，
可依据 RViz 完整地图把阈值小幅调整，但不能关闭完成状态、成功目标或地图文件
检查。

## 7. 参数与调优顺序

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `min_frontier_size` | 8 cells | 过滤零散未知噪声 |
| `min_goal_clearance_m` | 0.22 m | 前沿目标距占用单元的最小距离 |
| `min_goal_distance_m` | 0.80 m | 过滤落在 Nav2 到达容差附近的无效目标 |
| `information_gain_weight` | 2.0 | 大前沿的奖励 |
| `distance_weight` | 1.0 | 长路径的惩罚 |
| `goal_timeout_sec` | 120 s | 单个目标最长执行时间 |
| `blacklist_radius_m` | 0.55 m | 失败目标附近暂时禁选范围 |
| `blacklist_expiry_sec` | 180 s | 地图变化后允许重新尝试的周期 |
| `completion_timeout_sec` | 15 s | 完成候选状态保持多久才宣布完成 |
| `max_residual_frontier_cells` | 40 cells | 高覆盖率下可接受的不可用残余前沿上限 |
| `min_completion_known_area_m2` | 80 m² | 启用残余前沿完成判定的最低已知面积 |

若机器人频繁贴墙，先增大 `min_goal_clearance_m` 到 `0.26—0.30`；若门洞附近
只剩很短的有效前沿，再将 `min_frontier_size` 逐步减到 `5—6`。不要先允许 Nav2
穿越未知空间，否则“能到目标”不再证明前沿探索安全。

## 8. 证据清单

Ubuntu 22.04 / ROS 2 Humble 现场验收已完成：机器人自主穿过中部通道，
最终发送 25 个目标，成功 22、失败 3，仅剩 6 个不可达残余前沿。
完成地图 YAML/PGM 已自动保存，最终验收结果为 `PASS`。现场截图已归档至
`docs/evidence/autonomous_exploration/`。

- Gazebo 自建封闭 world 与机器人自主移动录屏，全程没有人工发送目标。
- RViz 同屏显示 `/map`、机器人、Nav2 路径以及 `/exploration/frontiers`。
- `/exploration/status` 中目标数增加、已知面积增长、最终状态为 `complete`。
- 运行中和完成态两个 `exploration_smoke_check` PASS 截图及退出码。
- 自动生成的 `autonomous_exploration.yaml + .pgm`，重新加载后轮廓完整。
- 至少记录一次失败目标黑名单或说明本轮没有触发恢复。

## 9. 常见问题

- 一直 `waiting_for_map`：检查 `/map`、`/scan` 和 `slam_toolbox`，确认 Gazebo
  未暂停。
- 一直 `waiting_for_tf`：检查 `map → odom → base_footprint`；SLAM、Nav2 和
  explorer 必须全部使用仿真时间。
- 一直 `waiting_for_nav2`：检查 `controller_server`、`planner_server`、
  `bt_navigator` 的 lifecycle 是否为 `active`。
- planner server 报 `class ... does not exist`：必须使用本挑战 launch；它会把
  新版 TurtleBot3 YAML 的 Navfn/behavior 插件名规范化为 Humble 的 `/` 形式。
- `frontiers remain but none is reachable and safe`：先看门洞是否已扫描为自由。
  若已知面积还小，再小幅降低安全距离或最小聚类尺寸；若已知面积超过
  `80 m²`且仅剩不超过 40 个无可行目标的边界单元，程序会经过
  15 秒保持期后正常完成并保存地图。不能直接把未知区设为可通行。
- 机器人在同一区域反复：提高距离惩罚或检查失败黑名单；同时确认 `/map_updates`
  正常进入 Nav2 global costmap。
- 地图过早完成：确认 LaserScan 最大量程与 world 尺寸匹配；完成态必须同时通过
  RViz 完整轮廓和已知面积阈值，不能只看“无前沿”。

## 10. 参考接口

- SLAM Toolbox Humble `online_async_launch.py` 与官方异步建图参数。
- Nav2 Humble `navigation_launch.py`（只启动导航节点，不启动 AMCL/map server）。
- TurtleBot3 Navigation2 Burger 参数作为稳定基线，启动时仅覆盖在线地图所需项。
