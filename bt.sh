#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

# 实车行为树
# ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py 
# 指定位姿参数文件示例：
# ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
#     pose:=pose.yaml
# 指定行为树名称示例：
# ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
#     target_tree:=2026rmuc

# 仿真行为树（默认使用 params/pose.yaml）
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
    pose:=pose_test.yaml \
    target_tree:=2026test_multi \
    use_sim_time:=True \
    namespace:=red_standard_robot1