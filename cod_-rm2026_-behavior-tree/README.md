# COD RM2026 Behavior Tree

基于 ROS 2 和 BehaviorTree.CPP 的 RoboMaster 2026 赛季哨兵机器人行为决策系统。

## 项目简介

本项目为 RoboMaster 哨兵机器人提供自主决策能力，通过行为树（Behavior Tree）框架实现导航、巡逻、血量监测、区域判断等策略逻辑，并通过串口与嵌入式系统通信获取裁判系统数据。

## 环境要求

本项目基于 Ubuntu 22.04 及以上版本开发，需要安装 ROS 2（支持 Humble / Iron / Jazzy）以及对应版本的 Nav2 导航栈。行为树框架使用 BehaviorTree.CPP 4.x，代码采用 C++20 标准编写，构建工具要求 CMake 3.8 以上。

## 环境配置

### 1. 安装 ROS 2

参考 [ROS 2 官方文档](https://docs.ros.org/en/humble/Installation.html) 安装对应版本。

### 2. 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-nav2-msgs \
  ros-${ROS_DISTRO}-behaviortree-cpp \
  ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-example-interfaces \
  libfmt-dev \
  libboost-dev
```

### 3. 克隆项目

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src # 创建工作环境
git clone https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree.git
```

## 编译

```bash
cd ~/ros2_ws
# 编译全部包
colcon build --symlink-install

# 或仅编译指定包
colcon build --symlink-install --packages-select rm_interfaces cod_serial cod_behavior

# 加载环境
source install/setup.bash
```

> **注意**：首次编译需先编译 `rm_interfaces`，因为其他包依赖该消息包：
> ```bash
> colcon build --packages-select rm_interfaces
> source install/setup.bash
> colcon build
> ```

## 运行

### 前提条件

运行行为树前需要先启动 Nav2 导航栈（提供 action server）：

```bash
# 启动 Nav2（根据实际机器人配置调整）
ros2 launch nav2_bringup bringup_launch.py
```

### 启动行为树决策系统

```bash
source install/setup.bash

# 主决策模式（行为树 + 串口通信）也是单点导航模式
ros2 launch cod_behavior cod.launch.py

# 多点导航模式（行为树 + 串口通信）
ros2 launch cod_behavior multiplenav.launch.py
```

### 单独运行节点

```bash
# 仅运行行为树节点（需手动指定参数）
ros2 run cod_behavior tree_1

# 运行测试节点（模拟裁判系统发布消息）
ros2 run cod_behavior test_1
```

### Groot2 可视化

行为树运行时会在端口 `5555` 开启 Groot2 Publisher，可使用 [Groot2](https://www.behaviortree.dev/groot/) 连接实时查看行为树状态。

## 配置说明

### 导航点位配置

编辑 `COD_Behavior/launch/cod_pose.yaml` 设置各策略导航点：

```yaml
cod_behavior:
  ros__parameters:
    cod_bt_path: "COD_Behavior/cod_bt/cod_tree.xml"  # 行为树文件路径
    nav_pose:
      home:                    #补给区位置坐标
        frame_id: "map"
        position: { x: 0.0, y: 0.0, z: 0.0 }
        orientation: { w: 1.0 }
      main:                    #中心增益点位置坐标（可根据实际场地更改）
        frame_id: "map"
        position: { x: 4.0, y: -3.4, z: 0.0 }
        orientation: { w: 1.0 }
```

### 多点导航航点文件

航点文件为 CSV 格式，位于 `COD_Behavior/launch/` 下：

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
0,0.317783,-3.43582,0,0,0,0,1,
1,2.0,-1.0,0,0,0,0,1,
```

### 行为树 XML

行为树定义文件位于 `COD_Behavior/cod_bt/`，可使用 Groot2 编辑器打开 `Project.btproj` 进行可视化编辑。

## 部署

### 1. 目标机器编译部署

```bash
# 在目标机器（如 NUC / Jetson）上克隆并编译
cd ~/ros2_ws/src
git clone https://gitee.com/codnavgation/cod_-rm2026_-behavior-tree.git -b test1 #用于当前备赛调试，最终比赛用的是master分支
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### 2. 检查运行状态

```bash
# 查看服务状态
sudo systemctl status cod-behavior.service

# 查看日志
journalctl -u cod-behavior.service -f
```

## License

本项目中 `BehaviorTree.ROS2` 子模块遵循 MIT 协议，其余代码参见各包 LICENSE 文件。