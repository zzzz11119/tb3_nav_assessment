# 附加挑战二｜Nav2 自定义 A* 全局规划器

## 1. 验收目标

本挑战不覆盖已稳定的 `simulation.launch.py` 和 Nav2 基线参数。通过标准为：

1. 以 `nav2_core::GlobalPlanner` 插件实现 A*，由 `pluginlib` 动态加载。
2. 使用 Nav2 全局代价地图，拒绝越界、致命障碍和禁止的未知区域。
3. 支持 4/8 邻域；8 邻域时检查对角两侧单元，不允许“切墙角”。
4. 路径代价同时考虑距离与 inflation cost，更倾向远离障碍物。
5. 独立 `astar_navigation.launch.py` 只向已有 Nav2 参数注入规划器配置。
6. 通过 `ComputePathToPose` 请求一条必须绕墙的路径，自动验收退出码为 `0`。

## 2. 算法设计

- 开集：按 `f(n) = g(n) + h(n)` 排序的最小优先队列。
- `g(n)`：移动距离 × `(1 + cost_penalty × normalized_cost)`。
- `h(n)`：到目标的欧氏距离，不高估空间最短距离。
- 目标可行时必须到达目标单元；目标被占用时才启用 `tolerance`。
- 返回的每个 pose 都有 `map` frame、仿真时间戳和沿路径切线的朝向。

## 3. 构建

将 `tb3_nav_assessment` 和 `tb3_astar_planner` 都放在同一 ROS 2 workspace 的
`src/` 中，然后：

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

cd ~/assessment_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

确认插件已导出：

```bash
ros2 pkg plugins --package nav2_core | grep tb3_astar_planner
```

期望包含 `tb3_astar_planner/AStarPlanner`。

## 4. 启动

终端 A 启动原稳定仿真：

```bash
source ~/assessment_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_nav_assessment simulation.launch.py
```

终端 B 使用已保存地图启动独立 A* Nav2 入口：

```bash
source ~/assessment_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_astar_planner astar_navigation.launch.py \
  map:=$HOME/maps/assessment.yaml
```

启动文件会读取 TurtleBot3 Burger 的完整 Nav2 参数，在临时文件中仅替换
`planner_server` 部分，并将 behavior plugin 名称规范化为 Humble 使用的
`nav2_behaviors/ClassName` 形式，关闭时删除临时文件。若已有验证过的项目 Nav2
参数，应显式传入：

某些较新的 TurtleBot3 `burger.yaml` 不再显式包含 `use_sim_time`，但 Humble
仿真仍必须让所有 Nav2 节点使用 `/clock`。本启动文件会遍历所有
`ros__parameters` 区段并注入 `use_sim_time:=true`，避免系统时间路径与仿真时间
TF 之间出现 `Transform data too old`。

规划器同时注册 `AStar` 和 `GridBased` 两个 ID，但它们都指向
`tb3_astar_planner/AStarPlanner`。这保留了 TurtleBot3 默认行为树硬编码请求的
`GridBased` 稳定接口，不会回退到 Navfn。自动验收仍显式请求 `AStar`。

```bash
ros2 launch tb3_astar_planner astar_navigation.launch.py \
  map:=$HOME/maps/assessment.yaml \
  base_params_file:=$HOME/assessment_ws/src/tb3_nav_assessment/config/nav2.yaml
```

在 RViz 设置初始位姿后，可继续用 `Nav2 Goal` 验证完整导航。

## 5. 参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `allow_unknown` | `false` | 是否允许路径穿过未知单元 |
| `use_eight_connected` | `true` | 在 4/8 邻域搜索之间切换 |
| `tolerance` | `0.35 m` | 目标单元被占用时的近邻容差 |
| `cost_penalty` | `2.0` | inflation cost 在 `g(n)` 中的权重 |
| `max_iterations` | `1000000` | 防止异常地图上无限搜索 |
| `lethal_cost` | `253` | 大于等于此代价时不可通行 |

可分别调整 [`astar_plugin.yaml`](../config/astar_plugin.yaml) 和启动文件中的
`PLUGIN_PARAMETERS`。演示前应保持两处一致。

## 6. 一键验收

在 Nav2 已 active 且 `/map` 可用的新终端运行：

```bash
source ~/assessment_ws/install/setup.bash
ros2 run tb3_astar_planner astar_smoke_check
echo $?
```

默认从 `(-4.5, 3.0)` 规划到 `(4.5, 3.0)`。在本项目的双区 world 中，
两点的直线被中间隔墙截断，合法路径必须向中央门洞绕行。验收节点会检查：

- `AStar` 命名规划器接受并成功完成 `ComputePathToPose`；
- 路径非空、坐标有限、frame 和目标误差正确；
- 所有路径点位于已知非障碍地图单元；
- 起终点直线确实穿墙，A* 结果长度确实形成绕行；
- 全部通过后打印 `A* OPTIONAL CHALLENGE CHECK: PASS` 并返回 `0`。

如地图边界不同，可用 ROS 参数替换起终点，但应继续选择一组“直线被障碍、
仍存在可行绕路”的点：

```bash
ros2 run tb3_astar_planner astar_smoke_check --ros-args \
  -p start_x:=-4.0 -p start_y:=2.8 \
  -p goal_x:=4.0 -p goal_y:=2.8
```

## 7. 证据清单

- `ros2 pkg plugins --package nav2_core` 显示自定义插件。
- planner server 日志显示加载 `AStar` / `tb3_astar_planner::AStarPlanner`。
- RViz 中 Global Plan 通过中央门洞而不穿墙的截图。
- `astar_smoke_check` 全部 PASS 和退出码 `0` 的终端截图。
- 机器人用该路径完成一次实际 Nav2 导航的录屏。

## 8. 常见问题

- `class ... does not exist`：重新构建后 source 当前 workspace 的 `install/setup.bash`。
- `planner AStar is not a valid planner`：确认使用的是 `astar_navigation.launch.py`，
  并检查启动日志中的临时参数文件。
- `planner GridBased is not a valid planner`：TurtleBot3 默认行为树会请求该 ID；
  确认启动日志中 `AStar` 和 `GridBased` 均已注册。
- `Transform data too old`：确认 `/amcl`、`/planner_server`、
  `/controller_server` 的 `use_sim_time` 均为 `True`，再重新设置 AMCL 初始位姿。
- 起点或终点被占用：先在 RViz 的 costmap 中检查坐标；不要用墙边界点。
- 路径贴墙：优先检查 global costmap 的 inflation radius，再小幅提高
  `cost_penalty`。
- 小地图搜索仍超时：检查地图原点、TF、分辨率和目标是否在封闭障碍中。
