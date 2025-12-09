#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py slam:=True