---
title: Turtlebot3_test
description: 记录Turtlebot3的相关命令
date: 2025-12-06
categories:
    - 工具
tags: ['Turtlebot3', 'ROS2', 'SLAM','命令速查']
---
## 前置准备
运行相关命令前需先安装对应功能包，如turtlebot3、slam_toolbox、cartographer、nav2等。  
若从源码编译安装，请确保已编译对应功能包。
编译命令如下：
```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select <package_name>
```
source命令如下，可添加至~/.bashrc文件中：
```bash
source ~/colcon_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
```
## 启动脚本，加载TurtleBot3及世界模型
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
## 启动nav2导航
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

