#!/bin/bash

# 只关闭由 bt.sh 启动的行为树相关进程，不影响其他仿真/导航组件。
# 不使用 set -e，避免进程不存在时脚本提前退出。

PROCESS_PATTERNS=(
  "ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py"
  "pb2025_sentry_behavior_launch.py"
  "pb2025_sentry_behavior_client"
  "pb2025_sentry_behavior_server"
)

echo "[INFO] 尝试优雅关闭行为树相关进程"
for pattern in "${PROCESS_PATTERNS[@]}"; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -f "$pattern" || true
    echo "[INFO] 已发送 SIGTERM -> $pattern"
  else
    echo "[INFO] 未发现进程 -> $pattern"
  fi
done

sleep 1

echo "[INFO] 检查并强制关闭残留行为树进程"
for pattern in "${PROCESS_PATTERNS[@]}"; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -9 -f "$pattern" || true
    echo "[OK] 已强制关闭 -> $pattern"
  else
    echo "[OK] 已关闭 -> $pattern"
  fi
done

echo "[DONE] 行为树关闭完成"