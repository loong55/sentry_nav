# pb_rm_interfaces

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/pb_rm_interfaces/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb_rm_interfaces/actions/workflows/ci.yml)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

ROS2 interfaces (.msg, .srv, .action) used in the StandardRobot++ project.

## msg

云台和射击使用自定义消息类型，

* GimbalCmd.msg：云台控制命令，使用绝对位置，单位为弧度
* ShootCmd.msg：射击命令，包含射击子弹数
* 底盘控制命令使用 ROS2 的 `geometry_msgs/msg/Twist`。
* referee

    当前对应串口协议版本：[V1.7.0 (20241225)](https://terra-1-g.djicdn.com/b2a076471c6c4b72b574a977334d3e05/RoboMaster%20%E8%A3%81%E5%88%A4%E7%B3%BB%E7%BB%9F%E4%B8%B2%E5%8F%A3%E5%8D%8F%E8%AE%AE%E9%99%84%E5%BD%95%20V1.7.0%EF%BC%8820241225%EF%BC%89.pdf)
