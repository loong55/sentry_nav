#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"  # 默认沿用当前工作区常用网段

DEFAULT_TOPICS=(
    /tf
    /tf_static
    /plan
    /local_plan
    /global_costmap/costmap
    /global_costmap/costmap_updates
    /local_costmap/costmap
    /local_costmap/costmap_updates
)

show_help() {
    echo "用法: $0 [bag_name] [topic1 topic2 ...]"
    echo "示例:"
    echo "  $0"
    echo "  $0 patrol_out"
    echo "  $0 patrol_out /tf /tf_static /plan /local_plan"
    echo
    echo "说明:"
    echo "  1. 不传 bag_name 时，会自动使用时间戳命名。"
    echo "  2. 不传 topic 时，默认只录制导航复盘常用 topic。"
    echo "  3. 默认 topic: ${DEFAULT_TOPICS[*]}"
    echo "  4. 默认每 60 秒自动导出一份累计快照，每份都包含从开始录制到当前时刻的全部数据。"
    echo "  5. 可用 BAG_SNAPSHOT_SECONDS 覆盖自动保存间隔，兼容旧变量 BAG_SPLIT_SECONDS。"
    echo "  6. 自动快照会保存为同级目录: <bag_name>_autosave_时间戳"
    echo "  7. 退出时会额外补一份最终累计快照，避免不足 1 分钟时没有 autosave。"
    echo "  8. 按 Ctrl+C 正常停止后，原始 bag 目录也会保留完整数据。"
}

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

bag_root="${BAG_ROOT_DIR:-bags}"
mkdir -p "${bag_root}"

if ! command -v python3 >/dev/null 2>&1; then
    echo "[record_rosbag] 错误: 未找到 python3，无法执行累计快照导出。" >&2
    exit 1
fi

snapshot_interval="${BAG_SNAPSHOT_SECONDS:-${BAG_SPLIT_SECONDS:-60}}"

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_name="${1:-bag_${timestamp}}"
bag_path="${bag_root}/${bag_name}"
snapshot_prefix="${bag_path}_autosave"

bag_message_count() {
    BAG_PATH="$1" python3 <<'PY'
import glob
import os
import sqlite3

bag_path = os.environ["BAG_PATH"]
db_paths = sorted(glob.glob(os.path.join(bag_path, "*.db3")))

if not db_paths:
    print(-1)
    raise SystemExit(0)

total = 0
for db_path in db_paths:
    conn = sqlite3.connect(db_path)
    try:
        total += conn.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
    finally:
        conn.close()

print(total)
PY
}

snapshot_once() {
    local snapshot_dir="$1"
    local active_message_count

    active_message_count="$(bag_message_count "${bag_path}")"
    if [ "${active_message_count}" = "-1" ] || [ "${active_message_count}" -eq 0 ]; then
        return 3
    fi

    ACTIVE_BAG_PATH="${bag_path}" SNAPSHOT_DIR="${snapshot_dir}" python3 <<'PY'
import glob
import os
import sqlite3
import sys
import time

active_bag_path = os.environ["ACTIVE_BAG_PATH"]
snapshot_dir = os.environ["SNAPSHOT_DIR"]
db_paths = sorted(glob.glob(os.path.join(active_bag_path, "*.db3")))

if not db_paths:
    sys.exit(3)

os.makedirs(snapshot_dir, exist_ok=True)

for src in db_paths:
    dst = os.path.join(snapshot_dir, os.path.basename(src))
    if os.path.exists(dst):
        os.remove(dst)

    last_error = None
    for _ in range(5):
        try:
            src_conn = sqlite3.connect(src, timeout=30.0)
            try:
                dst_conn = sqlite3.connect(dst)
                try:
                    src_conn.backup(dst_conn)
                finally:
                    dst_conn.close()
            finally:
                src_conn.close()
            break
        except sqlite3.Error as exc:
            last_error = exc
            time.sleep(1)
    else:
        print(f"backup failed for {src}: {last_error}", file=sys.stderr)
        sys.exit(1)
PY

    ros2 bag reindex -s sqlite3 "${snapshot_dir}" >/dev/null
}

snapshot_loop() {
    while kill -0 "${recorder_pid}" 2>/dev/null; do
        sleep "${snapshot_interval}"

        if ! kill -0 "${recorder_pid}" 2>/dev/null; then
            break
        fi

        snapshot_dir="${snapshot_prefix}_$(date +%Y%m%d_%H%M%S)"
        echo "[record_rosbag] 自动导出累计快照到: ${snapshot_dir}"
        if ! snapshot_once "${snapshot_dir}"; then
            echo "[record_rosbag] 提示: 当前尚无消息可保存，本轮跳过快照。" >&2
        fi
    done
}

handle_stop() {
    trap - INT TERM
    echo
    echo "[record_rosbag] 正在停止录制，等待 ros2 bag 正常写入 metadata..."
    if kill -0 "${recorder_pid}" 2>/dev/null; then
        kill -INT "${recorder_pid}" 2>/dev/null || true
    fi
}

if [ $# -le 1 ]; then
    echo "[record_rosbag] 开始录制精简导航 topic 到: ${bag_path}"
    echo "[record_rosbag] topics: ${DEFAULT_TOPICS[*]}"
    echo "[record_rosbag] 每 ${snapshot_interval} 秒自动导出一份累计快照。"
    echo "[record_rosbag] 结束录制请按 Ctrl+C，等待命令正常退出后即保存完成。"
    ros2 bag record -o "${bag_path}" "${DEFAULT_TOPICS[@]}" &
else
    shift
    echo "[record_rosbag] 开始录制指定 topic 到: ${bag_path}"
    echo "[record_rosbag] topics: $*"
    echo "[record_rosbag] 每 ${snapshot_interval} 秒自动导出一份累计快照。"
    echo "[record_rosbag] 结束录制请按 Ctrl+C，等待命令正常退出后即保存完成。"
    ros2 bag record -o "${bag_path}" "$@" &
fi

recorder_pid=$!
trap handle_stop INT TERM

snapshot_loop &
snapshot_pid=$!

recorder_status=0
wait "${recorder_pid}" || recorder_status=$?

if kill -0 "${snapshot_pid}" 2>/dev/null; then
    kill "${snapshot_pid}" 2>/dev/null || true
fi
wait "${snapshot_pid}" 2>/dev/null || true

final_message_count="$(bag_message_count "${bag_path}")"
if [ "${final_message_count}" -gt 0 ]; then
    final_snapshot_dir="${snapshot_prefix}_$(date +%Y%m%d_%H%M%S)_final"
    echo "[record_rosbag] 导出最终累计快照到: ${final_snapshot_dir}"
    if ! snapshot_once "${final_snapshot_dir}"; then
        echo "[record_rosbag] 警告: 最终累计快照导出失败。" >&2
    fi
else
    echo "[record_rosbag] 警告: 本次录制未捕获到任何消息，原始 bag 将是空包。" >&2
fi

echo "[record_rosbag] 最终完整 bag: ${bag_path}"
echo "[record_rosbag] 自动快照目录前缀: ${snapshot_prefix}_时间戳"

exit "${recorder_status}"