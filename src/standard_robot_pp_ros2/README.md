# standard_robot_pp_ros2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. Introduction

standard_robot_pp_ros2 是配合 [StandardRobot++](https://gitee.com/SMBU-POLARBEAR/StandardRobotpp.git) 下位机控制使用的机器人驱动，提供了机器人的控制接口、数据接口。

本项目获取下位机的 packet 并发布为 topic，并将下位机处理后的动态关节信息数据发布到 `joint_states` 话题，通过 [joint_state_publisher](https://github.com/ros/joint_state_publisher/tree/ros2/joint_state_publisher) 和 [robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/humble) 建立整车 TF 树（包含 static 和 dynamic）。

![frames](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames.5xaq4wriyy.webp)

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

```bash
sudo pip install vcstool2
pip install xmacro
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2.git src/standard_robot_pp_ros2
```

```bash
vcs import src < src/standard_robot_pp_ros2/dependencies.repos
vcs import src < src/pb2025_robot_description/dependencies.repos
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2.4 Running

1. 配置 udev，用来定向下位机 RoboMaster C 型开发板串口硬件并给予串口权限

    > 本命令在一台主机中只需要运行一次，无需重复运行。

    ```bash
    ./script/create_udev_rules.sh
    ```

2. 构建程序

    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

3. 运行上下位机通讯

    Tips: 如需开启 RViz 可视化，请添加 `use_rviz:=True` 参数。

    ```bash
    ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
    ```

### 2.5 Launch Arguments

| 参数 | 描述 | 类型 | 默认值 |
|-|-|-|-|
| `namespace` | 顶级命名空间 | string | "" |
| `params_file` | 用于所有启动节点的 ROS2 参数文件的完整路径 | string | [vision_params.yaml](./config/standard_robot_pp_ros2.yaml) |
| `robot_name` | 要使用的机器人 xmacro 文件名 | string | "pb2025_sentry_robot" |
| `use_rviz` | 是否启动 RViz | bool | True |
| `use_respawn` | 如果节点崩溃，是否重新启动。本参数仅 `use_composition:=False` 时有效 | bool | False |
| `log_level` | 日志级别 | string | "info" |

## 3. 协议结构

### 3.1 数据帧构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|frame_header|4|帧头|
|time_stamp|4|时间戳（基于下位机运行时间）|
|data|n|数据段|
|checksum|2|校验码|

### 3.2 帧头构成

|字段|长度 (Byte)|备注|
|:-:|:-:|:-:|
|sof|1|数据帧起始字节，固定值为 0x5A|
|len|1|数据段长度|
|id|1|数据段id|
|crc|1|数据帧头的 CRC8 校验|

### 3.3 data 数据段内容

详见飞书文档 [上下位机串口通信数据包](https://aafxu50hc35.feishu.cn/docx/HRh5dOjrMor4maxi3Xscvff6nCh?from=from_copylink)

## 4. 致谢

串口通信部分参考了 [rm_vision - serial_driver](https://github.com/chenjunnn/rm_serial_driver.git)，通信协议参考 DJI 裁判系统通信协议。

## 串口软件包总结笔记
### 1.概述
数据接收：下位机发过来的数据被上位机解析（主要是裁判系统数据），通过话题发布到pb_rm_interfaces包中，此数据用于pb2025_sentry_behavior的决策依据。

数据发送：通过话题，向下位机发送底盘速度，包含前后方向与左右方向速度与角速度。

### 2. 核心架构
主要组件
standard_robot_pp_ros2/
├── 串口通信模块 (serial_driver)
├── 数据接收线程 (receiveData)
├── 数据发送线程 (sendData)
├── 串口保护线程 (serialPortProtect)
├── CRC校验模块 (crc8_crc16)
└── 云台管理器 (gimbal_manager)

通信协议结构
[帧头4字节] + [时间戳4字节] + [数据段n字节] + [校验码2字节]

帧头结构：
SOF (1字节): 起始标志 0x5A
len (1字节): 数据段长度
id (1字节): 数据段ID
crc (1字节): 帧头CRC8校验

### 3.数据类型
接收数据包类型
ID	    名称	                 功能
0x01	DEBUG	                调试数据
0x02	IMU	                    IMU姿态数据（yaw/pitch/roll及角速度）
0x03	ROBOT_STATE_INFO	    机器人部位类型和状态
0x04	EVENT_DATA	            比赛事件数据
0x06	ALL_ROBOT_HP	        全场机器人血量
0x07	GAME_STATUS	            比赛状态和剩余时间
0x08	ROBOT_MOTION	        机器人速度矢量
0x09	GROUND_ROBOT_POSITION	地面机器人位置
0x0A	RFID_STATUS	            RFID状态（增益点占领情况）
0x0B	ROBOT_STATUS	        机器人状态
0x0C	JOINT_STATE	            关节状态
0x0D	BUFF	                Buff状态

发送数据包类型
ID	    名称	         功能
0x01	ROBOT_CMD	    机器人控制命令（底盘速度、云台角度、射击指令等）

### 4.工作流程
一、初始化
1. 加载参数（串口配置、波特率等）
2. 创建发布者（IMU、机器人状态、裁判系统数据等）
3. 创建订阅者（速度命令、云台命令、射击命令等）
4. 启动三个线程：
   - receiveData(): 接收串口数据
   - sendData(): 发送控制命令
   - serialPortProtect(): 监控串口状态

二、数据接收流程
while (rclcpp::ok()) {
  1. 读取帧头（4字节）
  2. CRC8校验帧头
  3. 根据长度读取数据段
  4. CRC16校验完整数据包
  5. 根据ID解析数据
  6. 发布到对应的ROS2话题
}

三、数据发送流程
定时循环：
  1. 从订阅回调获取命令数据
  2. 组装数据包（添加帧头、时间戳）
  3. 计算CRC校验码
  4. 通过串口发送

### 5.ROS2接口
5.1 发布的话题（处理串口接收的数据发布到pb_rm_interfaces）
话题名	                        消息类型	                      说明
/serial/imu	                    sensor_msgs/Imu	                IMU数据
/serial/gimbal_joint_state	    sensor_msgs/JointState	        云台关节状态
/serial/robot_state_info	    pb_rm_interfaces/RobotStateInfo	机器人部位信息
/serial/robot_motion	        geometry_msgs/Twist	            机器人运动速度
/referee/event_data	            pb_rm_interfaces/EventData	    裁判系统事件
/referee/robot_status	        pb_rm_interfaces/RobotStatus	机器人状态
/referee/game_status	        pb_rm_interfaces/GameStatus	    比赛状态

5.2 订阅的话题
话题名	                 消息类型	                 说明
/cmd_vel_nav2_result	geometry_msgs/Twist	        底盘速度命令
/cmd_gimbal_joint	    sensor_msgs/JointState	    云台关节命令
/cmd_spin	            example_interfaces/UInt8	射击命令
/tracker/target	        auto_aim_interfaces/Target	视觉追踪目标

### 6.关键代码
6.1 CRC校验
使用 CRC8 校验帧头，CRC16 校验完整数据包，确保数据传输可靠性。

// 帧头CRC8校验
bool crc8_ok = crc8::verify_CRC8_check_sum(
  reinterpret_cast<uint8_t *>(&header_frame), 
  sizeof(header_frame)
);

// 完整数据包CRC16校验
bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);

6.2 数据解析
使用模板函数从字节流转换为结构体：
template<typename T>
T fromVector(const std::vector<uint8_t> & data) {
  T packet;
  std::copy(data.begin(), data.end(), 
    reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

### 7.文件介绍
standard_robot_pp_ros2/
├── CMakeLists.txt                    # CMake构建配置
├── package.xml                       # ROS2包清单和依赖声明
│
├── config/
│   └── standard_robot_pp_ros2.yaml  # 参数配置（串口、波特率等）
│
├── include/standard_robot_pp_ros2/
│   ├── standard_robot_pp_ros2.hpp   # 主节点类定义
│   ├── packet_typedef.hpp           # 通信协议数据结构定义
│   ├── crc8_crc16.hpp              # CRC校验算法声明
│   ├── gimbal_manager.hpp          # 云台管理器节点定义
│   └── robot_info.hpp              # 机器人部位模型定义
│
├── src/
│   ├── standard_robot_pp_ros2.cpp   # 主节点实现（串口通信、数据收发）
│   ├── crc8_crc16.cpp              # CRC校验算法实现
│   └── gimbal_manager.cpp          # 云台管理器实现
│
├── launch/
│   └── standard_robot_pp_ros2.launch.py  # ROS2启动文件
│
└── script/
    └── create_udev_rules.sh        # 串口权限配置脚本

1. CMakeLists.txt 🔧
作用：CMake 构建配置文件

定义项目名称和C++标准（C++14）
查找ROS2依赖包
编译生成共享库 libstandard_robot_pp_ros2.so
注册两个ROS2组件节点：
standard_robot_pp_ros2_node（主驱动节点）
gimbal_manager_node（云台管理节点）
配置测试和安装规则

2. package.xml 📦
作用：ROS2 包清单文件

定义包的元数据（名称、版本、维护者、许可证）
声明所有依赖项：
构建依赖：ament_cmake
运行依赖：rclcpp、serial_driver、tf2_ros 等
接口依赖：pb_rm_interfaces、auto_aim_interfaces
声明测试依赖

📂 include/standard_robot_pp_ros2/ （头文件目录）
10. standard_robot_pp_ros2.hpp 🏗️
作用：主节点类定义

定义 StandardRobotPpRos2Node 类
包含所有发布者、订阅者声明
定义串口通信相关成员变量
声明数据接收/发送/保护线程函数
11. packet_typedef.hpp 📊
作用：通信协议数据结构定义

定义所有接收数据包结构（IMU、机器人状态、裁判系统等）
定义发送数据包结构（SendRobotCmdData）
定义帧头结构和数据包ID常量
提供模板函数用于数据类型转换
12. crc8_crc16.hpp ✅
作用：CRC校验算法声明

CRC8 校验（用于帧头）
CRC16 校验（用于完整数据包）
提供计算和验证函数
13. gimbal_manager.hpp 🎯
作用：云台管理器节点定义

定义 GimbalManagerNode 类
支持位置控制和速度控制两种模式
管理云台pitch和yaw轴的状态和控制
14. robot_info.hpp 🤖
作用：机器人部位模型定义

定义机器人各部位的类型映射：
底盘类型：无/麦轮/全向轮/舵轮/平衡
云台类型：无/yaw_pitch直连
发射机构类型：无/摩擦轮/气动
机械臂类型
自定义控制器类型

📂 src/ （源文件目录）
15. standard_robot_pp_ros2.cpp 🚀
作用：主节点实现（870行）

实现串口初始化和配置
实现三个工作线程：
receiveData()：接收并解析串口数据
sendData()：定时发送控制命令
serialPortProtect()：监控串口状态，自动重连
实现所有回调函数（速度、云台、射击等）
实现数据发布函数（IMU、机器人状态等）
16. crc8_crc16.cpp 🔐
作用：CRC校验算法实现

CRC8和CRC16查找表
校验码计算和验证函数实现
17. gimbal_manager.cpp 🎮
作用：云台管理器实现

接收云台控制命令
更新云台状态（位置/速度）
发布关节状态到 /cmd_gimbal_joint

📂 config/ （配置文件目录）
18. standard_robot_pp_ros2.yaml ⚙️
作用：ROS2参数配置文件

📂 launch/ （启动文件目录）
19. standard_robot_pp_ros2.launch.py 🚦
作用：ROS2 Launch 启动脚本

加载参数文件
启动主驱动节点
启动 joint_state_publisher（合并关节状态）
启动 robot_state_publisher（发布TF树）
可选启动 RViz 可视化
支持命名空间、日志级别等参数配置
📂 script/ （脚本目录）
20. create_udev_rules.sh 🔌
作用：配置串口设备权限

创建 udev 规则文件
将 RoboMaster C 型开发板固定为 ttyACM0
设置串口权限为 0777
将当前用户添加到 dialout 组

📊 文件依赖关系图
CMakeLists.txt
    ├─→ src/*.cpp (编译)
    ├─→ include/*.hpp (头文件)
    └─→ package.xml (依赖声明)

launch/standard_robot_pp_ros2.launch.py
    ├─→ config/standard_robot_pp_ros2.yaml (参数)
    └─→ standard_robot_pp_ros2_node (节点)

standard_robot_pp_ros2_node
    ├─→ serial_driver (串口通信)
    ├─→ crc8_crc16 (校验算法)
    └─→ packet_typedef (数据结构)

🎯 核心文件重要程度
⭐⭐⭐ 必须理解
standard_robot_pp_ros2.cpp - 主逻辑实现
packet_typedef.hpp - 通信协议
standard_robot_pp_ros2.yaml - 配置参数
⭐⭐ 重要理解
standard_robot_pp_ros2.hpp - 类结构
crc8_crc16.cpp - 校验算法
standard_robot_pp_ros2.launch.py - 启动流程
⭐ 了解即可
gimbal_manager.cpp - 云台管理
robot_info.hpp - 部位定义
其他配置文件 - 代码规范
💡 学习建议
先看协议：packet_typedef.hpp → 理解数据格式
再看主逻辑：standard_robot_pp_ros2.cpp → 理解工作流程
最后实践：修改配置 → 运行程序 → 观察效果