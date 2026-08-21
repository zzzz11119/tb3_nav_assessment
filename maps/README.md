# 地图文件说明

每张地图由同名 YAML 和 PGM 组成，两者必须保持在同一目录且不要单独改名。

| 文件前缀 | 用途 | 状态 |
|---|---|---|
| `assessment_map` | 正式考核使用的 SLAM Toolbox 地图 | 最终基线 |
| `autonomous_exploration` | 附加挑战四自动探索生成的地图 | 最终成果 |
| `assessment_map_candidate` | 正式地图确定前保留的候选版本 | 对照记录 |
| `simple_room_slam_toolbox` | 简化场景的 SLAM Toolbox 对比地图 | 基准实验 |
| `simple_room_cartographer` | 简化场景的 Cartographer 对比地图 | 基准实验 |

算法对比说明见
[`docs/validation/week02_slam_comparison.md`](../docs/validation/week02_slam_comparison.md)，
自主探索说明见
[`docs/challenges/autonomous_exploration_challenge.md`](../docs/challenges/autonomous_exploration_challenge.md)。
