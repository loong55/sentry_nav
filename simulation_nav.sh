#!/bin/bash
export QT_QPA_PLATFORM=xcb

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False