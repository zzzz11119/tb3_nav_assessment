# 冲刺第 1 周验收记录

状态：`[ ]` 待验证，`[x]` 已由命令、结果和证据共同确认。

## A. 官方基线

- [ ] Ubuntu 22.04、ROS2 Humble、Gazebo 和 TurtleBot3 版本已记录
- [ ] 官方 `empty_world.launch.py` 可启动
- [ ] Burger 连续运行 10 分钟无持续错误
- [ ] teleop 可正常前进、后退和转向
- [ ] `/scan`、`/odom`、`/tf`、`/tf_static`、`/clock` 正常
- [ ] Gazebo 与 RViz 可同时显示

证据：

- `notes/assets/sprint01_official_empty_world.png`
- `notes/assets/sprint01_official_topics.txt`
- `notes/assets/sprint01_official_scan_hz.txt`
- `notes/assets/sprint01_versions.txt`

## B. Package 与自建 world

- [x] `tb3_nav_assessment` package 骨架
- [x] `setup.py` 安装 launch/world/config/maps/rviz/docs 资源
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

- [ ] 自建 world 可从 install space 启动
- [ ] Burger 在 `(-4.0, -2.5)` 正确生成
- [ ] Burger 不穿模、不悬空、不持续滑动
- [ ] 外墙能阻止 Burger 离开世界
- [ ] teleop 可遍历西区、中央门和东区
- [ ] 三个障碍物均可在 Gazebo 和 LaserScan 中辨认
- [ ] 自动冒烟检查退出码为 0
- [ ] 连续运行 10 分钟稳定
- [ ] 删除 `build/install/log` 后可重新构建并启动

证据：

- `notes/assets/sprint01_custom_world_overview.png`
- `notes/assets/sprint01_custom_world_rviz_scan.png`
- `notes/assets/sprint01_custom_world_tf.png`
- `notes/assets/sprint01_smoke_check.txt`
- `notes/assets/sprint01_clean_build.txt`

## D. 工程与沟通

- [ ] GitHub private repository 已创建
- [ ] 新终端 clone 后可按 README 构建
- [ ] 第一次 15 分钟 check-in 已完成
- [ ] 08-02 周报已完成
- [ ] milestone commit/tag 已创建

## 结论

当前结论：工程静态资产已经就绪；A、C、D 中涉及 Ubuntu/Gazebo、GitHub 和现场
证据的项目尚未执行，不得提前标记里程碑通过。
