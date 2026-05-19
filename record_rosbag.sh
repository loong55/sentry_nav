#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"  # 默认沿用当前工作区常用网段

show_help() {
    echo "用法: $0 [bag_name] [topic1 topic2 ...]"
    echo "示例:"
    echo "  $0"
    echo "  $0 patrol_out"
    echo "  $0 patrol_out /tf /tf_static /cmd_vel"
    echo
    echo "说明:"
    echo "  1. 不传 bag_name 时，会自动使用时间戳命名。"
    echo "  2. 不传 topic 时，默认录制全部 topic。"
    echo "  3. 按 Ctrl+C 正常停止后，metadata.yaml 才会完整写入，bag 才算保存完成。"
}

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

bag_root="${BAG_ROOT_DIR:-bags}"
mkdir -p "${bag_root}"

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_name="${1:-bag_${timestamp}}"
bag_path="${bag_root}/${bag_name}"

if [ $# -le 1 ]; then
    echo "[record_rosbag] 开始录制全部 topic 到: ${bag_path}"
    echo "[record_rosbag] 结束录制请按 Ctrl+C，等待命令正常退出后即保存完成。"
    ros2 bag record -o "${bag_path}" -a
else
    shift
    echo "[record_rosbag] 开始录制指定 topic 到: ${bag_path}"
    echo "[record_rosbag] topics: $*"
    echo "[record_rosbag] 结束录制请按 Ctrl+C，等待命令正常退出后即保存完成。"
    ros2 bag record -o "${bag_path}" "$@"
fi