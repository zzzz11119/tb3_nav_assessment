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

## 启动官方仿真基线

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_nav_assessment simulation.launch.py
```

## 当前进度

- [x] 创建 `ament_python` package
- [x] 建立项目目录结构
- [x] 配置 `.gitignore`
- [x] 创建 GitHub private repository
- [x] 通过自己的 package 启动官方 empty world
- [ ] 上传本地提交到 GitHub
- [ ] 创建自定义 Gazebo world
