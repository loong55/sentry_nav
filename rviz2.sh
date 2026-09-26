#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"  # 默认沿用当前工作区常用网段

ros2 run rviz2 rviz2