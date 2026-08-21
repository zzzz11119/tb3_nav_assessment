# GitHub 阶段成果提交清单

本清单记录第一阶段在原有 GitHub 仓库基线之后补充的可复现成果。

## 已有远程基线

- 仓库：`zzzz11119/tb3_nav_assessment`（开发期间为 private，提交后已 public）
- 基线分支：`main`
- 本轮整理前远程 HEAD：`b0add3a`
- 已包含：自建 Gazebo world、SLAM 对比、Nav2 单目标/多目标和
  RGB-D 自定义相机。

## 本轮新增

- 附加挑战二：`optional_challenges/tb3_astar_planner/`，包含 A* Nav2
  global planner 插件、启动、配置、自动验收和文档。
- 附加挑战三：YOLOv5n ONNX 推理节点、RGB-D 距离、双模式
  launch、模型授权说明、配置、测试和 Ubuntu 验收证据。
- 附加挑战四：前沿提取/聚类/排序、Nav2 状态机、失败黑名单、
  高覆盖率完成判定、地图自动保存、测试、文档和 Ubuntu 验收证据。
- 地图：SLAM 主地图候选文件与自主探索最终 YAML/PGM。
- 工程记录：实验日志、挑战复现文档、阶段进度看板与精选现场截图。

## 现场验收摘要

- A* 插件：规划 187 个 pose、路径 11.32 m，绕墙检查 PASS。
- YOLO RGB-D：`raw=51`、`annotated=36`、约 1.89 Hz，检出
  `bus` / `person`，自动验收 PASS。
- 前沿探索：25 个目标，成功 22、失败 3，地图 YAML/PGM 自动保存，
  最终完成态验收 PASS。
- Ubuntu 主 package 静态/单元测试：`30 passed, 1 skipped`。
- Nav2 A* package 静态测试：`6 passed`。

## 明确不提交

- `build/`、`install/`、`log/`、`__pycache__/`、`.pytest_cache/`。
- `.exploration_install_backup_*`、`.perception_install_backup_*` 等安装备份。
- 传输 bundle、热修复 ZIP、rosbag 和微信临时文件。
