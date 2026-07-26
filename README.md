# tb3_nav_assessment

ROS2 Humble TurtleBot3 自主导航第一阶段考核项目。

## 环境

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- TurtleBot3 Burger
- Python 3.10

## 构建

```bash
cd ~/assessment_nav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动自建仿真场景

```bash
cd ~/assessment_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_nav_assessment simulation.launch.py
```

## 场景内容

- 12 m × 8 m 封闭场地
- 东西两个独立区域
- 2.4 m 宽中央通道
- 立方体、长方体和圆柱体三种障碍物
- TurtleBot3 Burger 初始位置 `(-4.0, -2.5)`

## 已验证功能

- 自建 Gazebo world 正确加载
- TurtleBot3 正确生成并具备物理碰撞
- `/scan`、`/odom`、TF 和 `/clock` 正常
- teleop 可遍历东西两区
- 外墙和障碍物碰撞有效
- RViz 可正常显示 LaserScan
- 删除 `build/`、`install/` 和 `log/` 后可重新构建并启动

## 项目文档

- [自建场景设计](docs/world_plan.md)
- [第一周仿真验证记录](docs/week01_validation.md)

## 当前进度

- [x] 创建 `ament_python` package
- [x] 建立项目目录结构
- [x] 配置 `.gitignore`
- [x] 创建 GitHub private repository
- [x] 完成官方 empty world 基线验证
- [x] 创建自定义 Gazebo world
- [x] 集成 TurtleBot3 启动文件
- [x] 完成传感器、碰撞和全区域 teleop 验证
- [x] 完成干净重建验证
- [ ] 上传本地提交到 GitHub
