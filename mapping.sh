#!/bin/bash

# 可选值示例：nav2_params.yaml / nav2_params_mppi.yaml
NAV2_PARAMS_NAME="nav2_params.yaml"

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

# Remove known conflicting SDK libusb paths from LD_LIBRARY_PATH for ROS runtime.
if [ -n "${LD_LIBRARY_PATH}" ]; then
	CLEANED_LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -vE '^/opt/MVS/lib/(32|64)$' | awk 'NF && !seen[$0]++' | paste -sd ':' -)"
	export LD_LIBRARY_PATH="${CLEANED_LD_LIBRARY_PATH}"
fi

# Force system libusb to avoid symbol mismatch with libpcl_io.
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:${LD_PRELOAD}}"

echo "[mapping] Using nav2 params: ${NAV2_PARAMS_NAME}"
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True params_name:=${NAV2_PARAMS_NAME}