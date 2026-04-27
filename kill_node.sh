#!/bin/bash

# 首先关闭行为树，再关闭其他ros节点
# 注意：不使用 set -e，避免 pkill 找不到进程时脚本提前退出

PROCESS_NAMES=(
  "pb2025_sentry_behavior_client"
  "pb2025_sentry_behavior_server"
  "referee_sim_node"
)

echo "[INFO] 尝试优雅关闭进程: ${PROCESS_NAMES[*]}"
for name in "${PROCESS_NAMES[@]}"; do
  if pgrep -f "$name" >/dev/null 2>&1; then
    pkill -f "$name" || true
    echo "[INFO] 已发送 SIGTERM -> $name"
  else
    echo "[INFO] 未发现进程 -> $name"
  fi
done

sleep 1

echo "[INFO] 检查并强制关闭残留进程"
for name in "${PROCESS_NAMES[@]}"; do
  if pgrep -f "$name" >/dev/null 2>&1; then
    pkill -9 -f "$name" || true
    echo "[OK] 已强制关闭 -> $name"
  else
    echo "[OK] 已关闭 -> $name"
  fi
done

echo "--- ROS 2 Environment Reset Script ---"

# 1. 停止ROS 2守护进程，清理DDS域状态
echo "[1/4] Stopping ROS 2 daemon..."
ros2 daemon stop || true

# 2. 强制杀死所有仿真相关进程
echo "[2/4] Killing simulation and nav2 processes..."
SIM_PATTERNS=(
  "refree_sim.sh"
  "ros2 launch referee_sim referee_sim.launch.py"
  "referee_sim.launch.py"
  "referee_sim_node"
  "gazebo.sh"
  "simulation_nav.sh"
  "nav_no_map.sh"
  "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
  "ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py"
  "ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py"
  "ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py"
  "ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py"
  "bringup_sim.launch.py"
  "standard_robot_pp_ros2.launch.py"
  "rm_navigation_reality_launch.py"
  "rm_navigation_simulation_launch.py"
  "pb2025_sentry_behavior_launch.py"
  "ign"
  "gazebo"
  "ruby /opt/ros/humble/bin/ros2 launch"
  "launch_ros"
  "component_container_isolated"
  "pointlio_mapping"
  "slam_toolbox"
  "terrainAnalysis"
  "terrainAnalysisExt"
  "loam_interface"
  "sensor_scan_generation"
  "fake_vel_transform"
  "small_gicp"
  "pointcloud_to_laserscan"
  "standard_robot_pp_ros2"
  "rmu_gazebo_simulator"
  "joy_node"
  "pb_teleop_twist_joy_node"
  "livox_ros_driver2_node"
  "map_saver_server"
  "lifecycle_manager"
  "static_transform_publisher"
  "pb2025_nav_bringup"
  "pb2025_sentry"
  "rviz2"
)
for pattern in "${SIM_PATTERNS[@]}"; do
  pkill -f "$pattern" || true
done

# 3. 等待进程退出
echo "Waiting for processes to exit..."
sleep 3

# 强制清理残留
for pattern in "${SIM_PATTERNS[@]}"; do
  pkill -9 -f "$pattern" || true
done

# 4. 清理DDS共享内存，防止下次启动时参数加载失败
echo "[3/4] Cleaning up DDS shared memory..."
# 清理 FastDDS/CycloneDDS 遗留的共享内存段
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
rm -f /dev/shm/*ros* 2>/dev/null || true
# 清理 boost interprocess 共享内存（ign/gazebo 使用）
find /dev/shm -maxdepth 1 -name "*.bp" -delete 2>/dev/null || true
find /tmp -maxdepth 1 -name "launch_params_*" -delete 2>/dev/null || true

# 5. 重新启动ROS 2守护进程
echo "[4/4] Starting ROS 2 daemon..."
sleep 1
ros2 daemon start || true

echo "--- Reset Complete! ---"
echo "You can now run ./simulation_nav.sh"
