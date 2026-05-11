#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py slam:=True