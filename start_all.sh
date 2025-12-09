#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=1 slam:=False use_robot_state_pub:=True &
# ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py &

sleep 5

gnome-terminal -- ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py 
    