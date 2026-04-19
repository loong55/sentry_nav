#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

gnome-terminal -- ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py &

# 最多等待 15 秒检查 /tf_static 中是否出现 front_mid360。
# 若检测到关键静态 TF，就继续按原逻辑启动导航（不额外拉起 robot_state_publisher）。
#若超时未检测到，就自动把导航启动参数切到 use_robot_state_pub:=True 兜底，保证 base_footprint/front_mid360 和 gimbal_yaw/gimbal_yaw_fake 相关 TF 不再缺失。
wait_for_static_tf() {
	local timeout_sec=15
	local elapsed=0

	while [ "$elapsed" -lt "$timeout_sec" ]; do
		if timeout 2s ros2 topic echo --once /tf_static 2>/dev/null | grep -q "child_frame_id: front_mid360"; then
			return 0
		fi
		sleep 1
		elapsed=$((elapsed + 1))
	done

	return 1
}

USE_ROBOT_STATE_PUB=False
if wait_for_static_tf; then
	echo "[nav_no_map] TF ready: base_footprint/front_mid360 tree detected from upstream robot_state_publisher."
else
	echo "[nav_no_map] WARN: /tf_static not ready within timeout, enabling nav-side robot_state_publisher fallback."
	USE_ROBOT_STATE_PUB=True
fi

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=${USE_ROBOT_STATE_PUB} &
sleep 5
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py &    