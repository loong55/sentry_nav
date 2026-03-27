#!/usr/bin/env bash
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