# 冲刺第 1 周验收记录

状态：`[ ]` 待验证，`[x]` 已由命令、结果和证据共同确认。

## A. 官方基线

- [x] Ubuntu 22.04、ROS2 Humble、Gazebo 和 TurtleBot3 版本已记录
- [x] 官方 `empty_world.launch.py` 可启动
- [x] Burger 连续运行 10 分钟无持续错误
- [x] teleop 可正常前进、后退和转向
- [x] `/scan`、`/odom`、`/tf`、`/tf_static`、`/clock` 正常
- [x] Gazebo 与 RViz 可同时显示

运行记录见 [`week01_validation.md`](week01_validation.md)。

## B. Package 与自建 world

- [x] `tb3_nav_assessment` package 骨架
- [x] `setup.py` 安装 launch/world/config/maps/rviz/docs 资源及分层文档
- [x] 12 m × 8 m 封闭外边界
- [x] 至少两个连通区域
- [x] 中央通道净宽 2.4 m
- [x] 3 个不同形状/尺寸障碍物
- [x] 墙和障碍物均有 collision 与 visual
- [x] `simulation.launch.py` 不使用本机绝对路径
- [x] 自动静态测试

静态结果：

```text
2026-07-24：
.......  [100%]
7 passed
```

复现命令：`python3 -m pytest projects/tb3_nav_assessment/test -q`

## C. 自建 world 现场验收

- [x] 自建 world 可从 install space 启动
- [x] Burger 在 `(-4.0, -2.5)` 正确生成
- [x] Burger 不穿模、不悬空、不持续滑动
- [x] 外墙能阻止 Burger 离开世界
- [x] teleop 可遍历西区、中央门和东区
- [x] 三个障碍物均可在 Gazebo 和 LaserScan 中辨认
- [x] 自动冒烟检查退出码为 0
- [x] 连续运行 10 分钟稳定
- [x] 删除 `build/install/log` 后可重新构建并启动

运行细节见 [`week01_validation.md`](week01_validation.md)，最终静态测试截图见
[`../evidence/tests/ubuntu_static_tests_pass_2026-08-20.png`](../evidence/tests/ubuntu_static_tests_pass_2026-08-20.png)。

## D. 工程与沟通

- [x] GitHub public repository 已发布
- [x] 新终端 clone 后可按 README 构建
- [x] 第一次 15 分钟 check-in 已完成
- [ ] 08-02 周报已完成
- [ ] milestone commit/tag 已创建

## 结论

当前结论：第 1 周核心技术验收已通过。周报和 milestone tag 属于过程管理项，
不影响本仓库已记录的仿真、传感器、重建和 GitHub 复现结果。
