#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py 
