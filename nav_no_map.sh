#!/bin/bash

# 可选值示例：nav2_params.yaml / nav2_params_mppi.yaml
# NAV2_PARAMS_NAME="nav2_params.yaml"
NAV2_PARAMS_NAME="nav2_params_mppi.yaml"

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

gnome-terminal -- ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py &


# 最多等待 15 秒检查 base_footprint -> front_mid360 是否已经可查询。
# 仅检查 /tf_static 中出现某个 child_frame_id 容易误判，因为单个静态 TF 片段存在
# 并不代表整条机器人 TF 链已经接通。
# 若超时仍无法查询到这条关键变换，就自动启用 nav 侧 robot_state_publisher 兜底。
wait_for_robot_tf() {
	local timeout_sec=15
	local elapsed=0

	while [ "$elapsed" -lt "$timeout_sec" ]; do
		if timeout 2s ros2 run tf2_ros tf2_echo base_footprint front_mid360 2>/dev/null | grep -q "Translation:"; then
			return 0
		fi
		sleep 1
		elapsed=$((elapsed + 1))
	done

	return 1
}

USE_ROBOT_STATE_PUB=False
if wait_for_robot_tf; then
	echo "[nav_no_map] TF ready: base_footprint -> front_mid360 transform is available from upstream robot_state_publisher."
else
	echo "[nav_no_map] WARN: base_footprint -> front_mid360 transform not ready within timeout, enabling nav-side robot_state_publisher fallback."
	USE_ROBOT_STATE_PUB=True
fi

echo "[nav_no_map] Using nav2 params: ${NAV2_PARAMS_NAME}"
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=${USE_ROBOT_STATE_PUB} params_name:=${NAV2_PARAMS_NAME} &
sleep 5

# ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py    