#!/bin/bash

# 这个脚本用于清理ROS 2环境，解决节点第二次启动失败的问题

set -euo pipefail

PROCESS_NAMES=(
  "pb2025_sentry_behavior_client"
  "pb2025_sentry_behavior_server"
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

# 这个脚本用于清理ROS 2环境，解决节点第二次启动失败的问题

echo "--- ROS 2 Environment Reset Script ---"

# 1. 停止ROS 2守护进程，清理DDS域状态
echo "[1/3] Stopping ROS 2 daemon..."
ros2 daemon stop

# 2. 强制杀死所有可能残留的ROS 2节点进程
#    -f 选项意味着匹配整个命令行，非常有效
echo "[2/3] Killing any remaining ROS 2 node processes..."
pkill -f "rclpy"      # 杀死Python节点
pkill -f "rclcpp"     # 杀死C++节点
# 你也可以加上你的特定节点名，例如：
# pkill -f "my_robot_node"

# 3. 等待2秒，确保资源释放
echo "Waiting for 2 seconds..."
sleep 2

# 4. 重新启动ROS 2守护进程
echo "[3/3] Starting ROS 2 daemon..."
ros2 daemon start

echo "--- Reset Complete! ---"
echo "You can now source your workspace and run your nodes again."
