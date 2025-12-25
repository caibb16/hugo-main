---
title: Turtlebot3_test
description: 记录Turtlebot3的相关命令
date: 2025-12-06
categories:
    - 命令
tags: ['Turtlebot3', 'ROS2', 'SLAM']
---

## 启动 TurtleBot3 基础世界仿真
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
## 启动 slam_toolbox
slam_toolbox 运行在线 SLAM ：  
```bash
ros2 launch slam_toolbox online_sync_launch.py
```
## 启动 cartographer
cartographer建图：  
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
## 运行 RViz2 可视化 SLAM（cartographer自带rviz，无需启动）
```bash
ros2 run rviz2 rviz2
```
## 键盘控制机器人运动
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
## 保存地图供nav2使用
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```


