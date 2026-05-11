#!/bin/bash

# 仅关闭 ROS_DOMAIN_ID=5 下启动的 ROS 相关进程，避免影响其他域。

TARGET_DOMAIN_ID=5

is_target_domain_pid() {
  local pid="$1"
  [[ -r "/proc/$pid/environ" ]] || return 1

  tr '\0' '\n' < "/proc/$pid/environ" | grep -qx "ROS_DOMAIN_ID=$TARGET_DOMAIN_ID"
}

collect_target_pids() {
  local pid
  for pid_dir in /proc/[0-9]*; do
    pid="${pid_dir##*/}"
    if is_target_domain_pid "$pid"; then
      printf '%s\n' "$pid"
    fi
  done
}

terminate_target_pids() {
  local signal="$1"
  local pid
  shift

  for pid in "$@"; do
    if kill "$signal" "$pid" 2>/dev/null; then
      echo "[INFO] 已发送 $signal -> PID $pid"
    fi
  done
}

echo "--- ROS Domain Reset Script ---"
echo "[INFO] 目标 ROS_DOMAIN_ID=$TARGET_DOMAIN_ID"

mapfile -t target_pids < <(collect_target_pids)

if [[ ${#target_pids[@]} -eq 0 ]]; then
  echo "[INFO] 未发现 ROS_DOMAIN_ID=$TARGET_DOMAIN_ID 的进程"
else
  echo "[1/3] 优雅关闭目标域进程..."
  terminate_target_pids -TERM "${target_pids[@]}"

  sleep 2

  mapfile -t remaining_pids < <(collect_target_pids)
  if [[ ${#remaining_pids[@]} -gt 0 ]]; then
    echo "[2/3] 强制关闭残留目标域进程..."
    terminate_target_pids -KILL "${remaining_pids[@]}"
  else
    echo "[2/3] 目标域进程已全部退出"
  fi
fi

echo "[3/3] 重置 ROS 2 daemon (ROS_DOMAIN_ID=$TARGET_DOMAIN_ID)..."
export ROS_DOMAIN_ID="$TARGET_DOMAIN_ID"
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo "--- Reset Complete! ---"
echo "仅处理了 ROS_DOMAIN_ID=$TARGET_DOMAIN_ID 的进程"
