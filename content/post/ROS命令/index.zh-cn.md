+++
date = '2026-02-05T10:24:25+08:00'
draft = false
title = 'ROS命令'
categories = ['工具']
tags = ['ROS', '命令速查']
+++

本文档记录了 ROS 机器人（Go2）的常用操作命令，包括 SSH 连接、建图、导航、控制等完整流程。

## SSH 连接信息

常用设备的 SSH 连接地址：

```bash
# 9025 设备
ssh wheeltec@192.168.110.182

# 9004 设备
ssh wheeltec@192.168.110.181

# 其他设备
ssh wheeltec@192.168.168.50
# 密码: dongguan

ssh firefly@192.168.168.168
```

## 网络配置

### WiFi 连接

```bash
# 连接到办公室 WiFi
sudo nmcli dev wifi connect "WL-OFFICE" password "wlwl1102"
```

### 查看网络信息

```bash
# 方式一
ifconfig

# 方式二
ip a
```

## Docker 容器管理

### 查看和启动容器

```bash
# 查看所有容器
docker ps -a

# 启动机器人运行容器
docker start robot_runner
```

## 建图流程

### 1. 启动建图程序

```bash
# 进入容器
docker exec -it robot_runner bash

# 运行在线 SLAM 建图
/root/ws_nav/src/go2_real/1_run_online_slam.sh
```

> **提示**: 手持遥控器控制狗开始行走扫图

### 2. 点云图处理

```bash
# 在容器内编辑地图
docker exec -it robot_runner bash
/root/ws_nav/src/go2_real/2_edit_map.sh

# 退出容器
exit
```

### 3. 地图转换与部署

**在宿主机上执行（不要在容器内）**

```bash
# 转换 PCD 到 PGM 格式
cd ~/ws_nav/src/go2_real
bash 2.1_pcd2pgm.sh

# 创建导航目录
mkdir -p ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pgm

# 复制最新地图到导航目录
cp /home/wheeltec/maps/$(ls -t /home/wheeltec/maps/ | head -1)/* \
   ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pgm/

# 确认地图文件
ls -lh ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pgm/
```

### 4. 上传地图到系统

```bash
# 停止守护进程
sudo bash /opt/vasystem/daemon.sh stop

# 启动守护进程
sudo bash /opt/vasystem/daemon.sh start
```

## 导航操作

### 启动导航

> **重要**: 启动导航前，将狗移动到建图起点附近，且保持朝向与建图时一致

```bash
# 进入容器
docker exec -it robot_runner bash

# 启动导航
/root/ws_nav/src/go2_real/3_nav_start.sh
```

等待日志输出控制文本后，表示启动成功。

### ROS2 节点启动

**必须在容器外启动**

```bash
cd ~/ws_ros2/
bash start.sh

# 或手动 source 环境
cd ~/ws_ros2
source install/setup.bash
```

## 导航控制命令

### 查询当前位置

```bash
# 方式一（推荐）
rosrun tf tf_echo map base_link

# 方式二
# bash /root/ws_nav/src/tools/query_current_pose.sh
```

### 发布导航目标点

```bash
# 格式: publish_clicked_point.sh x y theta
bash /root/ws_nav/src/tools/publish_clicked_point.sh 0.5 0.5 0
```

### 紧急停止

```bash
# 方式一：脚本停止
/root/ws_nav/src/tools/emergency_stop.sh

# 方式二：发布零速度
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 巡检状态控制

### 启动巡检

```bash
ros2 topic pub /inspecting_state std_msgs/msg/Bool '{data: true}' -r 1
```

### 关闭巡检

```bash
ros2 topic pub /inspecting_state std_msgs/msg/Bool '{data: false}' -r 1
```

## 话题监听

### 实时位置监听

```bash
rostopic echo /topology/real_time_point
```

### 定位信息查询（容器内）

```bash
docker exec -it robot_runner bash
cd ~/ws_nav_node/
source devel/setup.bash
rostopic echo /topology/real_time_point
```

## 地图导出与备份

### 创建地图备份

```bash
# 创建导出目录
mkdir -p map_export

# 复制所有地图相关文件
cp ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pgm/* map_export/
cp ~/ws_nav/src/go2_real/navigation_3d/global_planner/pcd_map/pcd/aft_pgo_free_space.pcd map_export/

# 打包（带时间戳）
tar -czf robot_map_$(date +%Y%m%d_%H%M%S).tar.gz map_export/

# 查看打包文件
ls -lh robot_map_*.tar.gz
```

### 下载地图到本地

**在本地电脑上执行**

```bash
scp wheeltec@192.168.110.148:~/robot_map_*.tar.gz .
```

## 配置文件管理

### 定位参数配置

```bash
# 查看定位配置
cat ~/ws_nav/src/go2_real/navigation_3d/fast_lio_nav/open3d_loc/config/loc_param_go2.yaml

# 编辑定位配置
nano ~/ws_nav/src/go2_real/navigation_3d/fast_lio_nav/open3d_loc/config/loc_param_go2.yaml
```

### 地图发布脚本

```bash
# 添加执行权限
chmod +x ~/publish_map.py
```

## 编译相关

### 编译指定包

```bash
# 只编译指定包及其依赖
catkin_make --only-pkg-with-deps <包名>
```

---

## 常用工作流程总结

1. **建图流程**: 启动建图 → 遥控器控制行走 → 编辑地图 → 转换格式 → 上传系统
2. **导航流程**: 移动到起点 → 启动导航 → 启动 ROS2 节点 → 发布目标点
3. **备份流程**: 复制地图文件 → 打包 → 下载到本地

> **提示**: 所有涉及机器人运动的操作都需要先启动 ROS2 节点