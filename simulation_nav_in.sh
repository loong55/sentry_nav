#!/bin/bash
export QT_QPA_PLATFORM=xcb

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

# gnome-terminal -- ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc_2026fix &
gnome-terminal -- ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc2026_in &
# gnome-terminal -- ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=gazebo_test_field &
sleep 10

ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
	slam:=True \
	namespace:=red_standard_robot1 &
sleep 5

# # Reuse sentry behavior tree to publish navigation goals for exploration.
# ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
# 	use_sim_time:=True \
# 	namespace:=red_standard_robot1