#!/bin/bash
export QT_QPA_PLATFORM=xcb

source /opt/ros/humble/setup.bash
source install/setup.bash

# 先开启边建图边导航仿真，无需地图；如果要开启有先验地图的重定位导航仿真，slam:=False，并将world改为需要加载的pcd和pgm地图
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
	world:=rmuc_2025 \
	slam:=True \
	namespace:=red_standard_robot1 &
sleep 5

# Reuse sentry behavior tree to publish navigation goals for exploration.
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
	use_sim_time:=True \
	namespace:=red_standard_robot1