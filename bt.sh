#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

# 点位文件
# pose_file="${1:-pose.yaml}"
pose_file="${1:-pose_rmuc.yaml}"

# 实车行为树
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
	pose_params_file:="$(pwd)/src/pb2025_sentry_behavior/params/$pose_file" &

# 仿真行为树
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
    pose_params_file:="$(pwd)/src/pb2025_sentry_behavior/params/$pose_file" \
	use_sim_time:=True \
	namespace:=red_standard_robot1 