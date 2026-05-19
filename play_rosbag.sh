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

echo "[play_rosbag] 开始回放: ${bag_path}"
if [ $# -gt 0 ]; then
    echo "[play_rosbag] 额外参数: $*"
fi

ros2 bag play "${bag_path}" "$@"