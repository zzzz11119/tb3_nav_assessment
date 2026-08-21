# SLAM 多场景对比实验方案

## 1. 研究问题

本实验回答三个问题：

1. SLAM Toolbox 在简单、一般和复杂场景中的地图质量与稳定性如何变化？
2. SLAM Toolbox 的 asynchronous / synchronous 模式有何取舍？
3. 在相同输入数据下，SLAM Toolbox 与 Cartographer 的建图质量、资源开销和导航可用性有何差异？

## 2. 实验层级

### L0｜考核必做

- 场景：`assessment_world.world`
- 算法：SLAM Toolbox asynchronous
- 产出：可供 Nav2 使用的 `.yaml + .pgm` 地图

### L1｜老师建议的最小研究增强

- 场景：`simple_room.world`、`assessment_world.world`
- 算法：SLAM Toolbox asynchronous、Cartographer 2D
- 每个组合至少重复 2 次

### L2｜完整对比

- 场景：`simple_room.world`、`assessment_world.world`、`maze_world.world`
- 算法/模式：
  - SLAM Toolbox asynchronous
  - SLAM Toolbox synchronous
  - Cartographer 2D
- 每个组合重复 3 次

8 月 16 日前优先完成 L0；L1 在不影响 Nav2 和 Python 多目标导航时完成；L2 可作为后续研究。

## 3. 场景梯度

| 场景 | 设计 | 主要挑战 | 研究用途 |
|---|---|---|---|
| 简单房间 | 单个矩形房间，少量障碍物 | 基线、低歧义 | 检查安装、配置和理想条件表现 |
| 双区考核场景 | 两个区域、门洞、3 个障碍物 | 回环、通道与局部遮挡 | 考核主场景和中等复杂度 |
| 迷宫 | 多走廊、相似转角、局部窄通道 | 感知混淆、累计漂移、回环 | 检查算法在重复结构中的鲁棒性 |

场景复杂度逐级增加，但机器人通行宽度不得低于当前 Nav2 安全设计要求。

## 4. 公平对比协议

### 固定条件

- Ubuntu、ROS2、Gazebo 和 TurtleBot3 版本固定。
- TurtleBot3 Burger、出生点、激光频率固定。
- 每个场景使用同一条运动路线和近似速度。
- 地图分辨率、最大/最小激光距离尽量统一。
- 每次实验从冷启动开始，记录完整参数和 commit。

### 推荐的共同输入

在不启动 SLAM 的情况下记录仿真输入：

```bash
ros2 bag record \
  /scan /odom /tf /tf_static /clock /cmd_vel \
  -o bags/<scene>_input
```

之后关闭 Gazebo，使用 `ros2 bag play ... --clock` 分别驱动不同 SLAM 节点。不要在输入 bag 中录制 `/map` 或某个算法发布的 `map → odom`，避免污染对比。

若离线 TF 或时钟回放不稳定，则退回“固定路径控制脚本 + 同一速度参数”，并在报告中说明限制。

## 5. 指标

### 地图质量

- 区域覆盖是否完整
- 外墙、内墙和障碍物是否清晰
- 双墙/鬼影数量
- 墙体厚度是否一致
- 回到起点闭环后是否发生明显跳变
- 地图能否保存并被 Nav2 正常加载

### 稳定性

- 成功建图次数 / 总次数
- TF timeout、scan dropped、tracking loss 次数
- 对快速转弯、长走廊和相似转角的敏感程度

### 效率

- 建图总耗时
- CPU 平均值/峰值
- 内存平均值/峰值
- 是否能跟上仿真实时速度

### 下游导航价值

- 相同目标点的 Nav2 成功率
- 平均导航时间
- 全局路径是否穿墙或进入未知区域
- 定位是否稳定

下游导航成功率是最重要的综合指标，地图“看起来漂亮”不能代替可导航性。

## 6. 结果记录表

| 场景 | 算法/模式 | 轮次 | 完整度 | 鬼影数 | 建图时间 | CPU 峰值 | 内存峰值 | Nav2 成功率 | 备注 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| simple | Toolbox async | 1 |  |  |  |  |  |  |  |
| simple | Cartographer | 1 |  |  |  |  |  |  |  |
| assessment | Toolbox async | 1 |  |  |  |  |  |  |  |
| assessment | Cartographer | 1 |  |  |  |  |  |  |  |

完整度和鬼影数应事先定义统一判定规则；无法可靠量化的项目采用“好/中/差”并附地图截图证据。

## 7. 预期分析框架

不要预设某个算法一定更好。最终结论按场景分别回答：

- 哪个算法在简单房间最省资源？
- 哪个算法在双区场景闭环和边界表现更稳定？
- 哪个算法在迷宫的重复结构中更容易漂移？
- synchronous 相比 asynchronous 是否减少丢帧，但增加计算压力？
- 哪张地图最适合作为 Nav2 最终地图，为什么？

## 8. 文件组织

```text
worlds/
├── simple_room.world
├── assessment_world.world
└── maze_world.world
config/
├── slam_toolbox_async.yaml
├── slam_toolbox_sync.yaml
└── cartographer_2d.lua
maps/benchmark/
├── simple/
├── assessment/
└── maze/
docs/results/
├── slam_runs.csv
├── map_quality_notes.md
└── figures/
```

大型 bag 不提交 GitHub；只提交 `source_manifest`、录制命令、bag 信息和结果摘要。

## 9. 停止条件

出现以下任一情况，暂停研究增强并回到考核主线：

- 08-06 前主场景的 SLAM Toolbox 地图仍不能稳定保存/加载。
- 08-10 前 Nav2 单点导航仍未通过。
- 新算法安装或 TF 适配连续占用超过半天。
- 对比实验破坏已经稳定的考核配置。

研究配置与考核稳定配置必须使用不同文件，禁止在最后一周直接覆盖稳定参数。
