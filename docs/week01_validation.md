# 第一周仿真环境与自建场景验证记录

日期：2026-07-26  
平台：Ubuntu 22.04 / ROS2 Humble / Gazebo Classic 11 / TurtleBot3 Burger

## 1. 官方仿真基线

通过官方 TurtleBot3 Gazebo 启动文件完成空世界测试。

验证结果：

- TurtleBot3 Burger 成功生成
- `/clock` 正常发布仿真时间
- `/odom` 正常发布里程计数据
- `/scan` 正常发布 LaserScan 数据
- `/tf` 和 `/tf_static` 正常发布
- `odom -> base_footprint -> base_scan` 坐标变换正常
- 官方空世界中 `/scan` 发布频率约为 4.2 Hz
- teleop 前进、转向和停止控制正常
- RViz Global Status 与 LaserScan Status 均为 OK

## 2. 自建 Gazebo World

场地内部尺寸为 12 m × 8 m，包含：

- 四面封闭外墙
- 将场地分成东西两区的中央隔墙
- 宽度为 2.4 m 的中央通道
- 西北区域红色立方体
- 东北区域黄色长方体
- 东南区域绿色圆柱体
- TurtleBot3 初始位置 `(-4.0, -2.5)`

`gz sdf -k worlds/assessment_world.world` 检查结果为 `Check complete`。

## 3. 集成启动验证

使用以下命令启动自建场景：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch tb3_nav_assessment simulation.launch.py
```

验证结果：

- 自建 world 正确加载
- TurtleBot3 在指定初始位置成功生成
- 差速驱动和关节状态插件正常运行
- `/scan` 在自建场景中稳定约为 3.46～3.50 Hz
- LaserScan 包含有效有限距离值
- RViz 中墙体和障碍物轮廓显示正常

## 4. 运动与碰撞验证

通过 TurtleBot3 teleop 完成以下测试：

- 从西区出发
- 经过中央通道进入东区
- 到达红色立方体附近
- 到达黄色长方体附近
- 到达绿色圆柱体附近
- 覆盖东西两区主要可行驶区域
- 障碍物与外墙均能阻止机器人穿越
- 测试结束时线速度和角速度均恢复为 0

进入东区后里程计位置曾达到：

- `x = 1.0247`
- `y = -1.8555`

其中 `x > 0`，证明机器人已通过中央通道进入东区。

## 5. 当前结论

第一周仿真基础功能验证通过：

- ROS2 与 Gazebo 环境可用
- TurtleBot3 仿真、传感器、TF 和里程计工作正常
- 自建 world 的结构、碰撞和障碍物有效
- 集成启动文件可正常启动完整场景
- 机器人可通过 teleop 遍历全部设计区域

## 6. 后续事项

- 完成删除构建产物后的干净重建测试
- 整理关键截图和运行日志
- 在网络恢复后将本地提交推送到 GitHub
