#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

# 指定位姿参数文件示例：
#     pose:=pose.yaml
# 指定行为树名称示例：
#     target_tree:=2026rmuc

ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
    pose:=pose_out.yaml \
    target_tree:=2026rmuc_out \
    use_sim_time:=True \
    namespace:=red_standard_robot1