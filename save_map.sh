#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

ros2 run nav2_map_server map_saver_cli -f 2