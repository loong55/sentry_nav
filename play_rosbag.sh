#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"  # 默认沿用当前工作区常用网段

show_help() {
    echo "用法: $0 <bag_path_or_name> [ros2 bag play 参数 ...]"
    echo "示例:"
    echo "  $0 bag_20260520_120000"
    echo "  $0 bags/patrol_out"
    echo "  $0 patrol_out --loop"
    echo
    echo "说明:"
    echo "  1. 既可以传完整路径，也可以只传 bags 目录下的包名。"
    echo "  2. 额外参数会原样透传给 ros2 bag play。"
    echo "  3. 若目标 bag 的 metadata 不完整，会先自动 reindex。"
    echo "  4. 若目标 bag 为空，会自动尝试同名前缀下最新的非空 autosave 快照。"
}

bag_message_count() {
    BAG_PATH="$1" python3 <<'PY'
import glob
import os
import sqlite3
import sys

bag_path = os.environ["BAG_PATH"]
db_paths = sorted(glob.glob(os.path.join(bag_path, "*.db3")))

if not db_paths:
    print(-1)
    sys.exit(0)

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

prepare_bag() {
    local candidate_path="$1"
    local message_count

    message_count="$(bag_message_count "${candidate_path}")"
    if [ "${message_count}" = "-1" ]; then
        return 1
    fi

    if [ "${message_count}" -gt 0 ]; then
        echo "[play_rosbag] 检测到 ${message_count} 条消息，先重建 metadata: ${candidate_path}" >&2
        ros2 bag reindex -s sqlite3 "${candidate_path}" >/dev/null
        printf '%s\n' "${candidate_path}"
        return 0
    fi

    return 1
}

find_latest_autosave() {
    local base_path="$1"
    local parent_dir base_name autosave_dir

    parent_dir="$(dirname "${base_path}")"
    base_name="$(basename "${base_path}")"

    while IFS= read -r autosave_dir; do
        if resolved_path="$(prepare_bag "${autosave_dir}")"; then
            printf '%s\n' "${resolved_path}"
            return 0
        fi
    done < <(find "${parent_dir}" -maxdepth 1 -mindepth 1 -type d -name "${base_name}_autosave_*" | sort -r)

    return 1
}

if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

bag_input="$1"
shift

if [ -d "${bag_input}" ]; then
    bag_path="${bag_input}"
elif [ -d "bags/${bag_input}" ]; then
    bag_path="bags/${bag_input}"
else
    echo "[play_rosbag] 未找到 rosbag 目录: ${bag_input}"
    echo "[play_rosbag] 你可以传完整路径，或者传 bags 目录下已有的包名。"
    exit 1
fi

if resolved_path="$(prepare_bag "${bag_path}")"; then
    bag_path="${resolved_path}"
else
    echo "[play_rosbag] 目标 bag 没有可回放消息: ${bag_path}"
    if resolved_path="$(find_latest_autosave "${bag_path}")"; then
        bag_path="${resolved_path}"
        echo "[play_rosbag] 已自动切换到最新非空 autosave: ${bag_path}"
    else
        echo "[play_rosbag] 未找到可回放的 autosave 快照。"
        echo "[play_rosbag] 当前目录里的 db3/metadata 是空包，无法回放不存在的数据。"
        exit 1
    fi
fi

echo "[play_rosbag] 开始回放: ${bag_path}"
if [ $# -gt 0 ]; then
    echo "[play_rosbag] 额外参数: $*"
fi

ros2 bag play "${bag_path}" "$@"