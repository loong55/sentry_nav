#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

# Remove known conflicting SDK libusb paths from LD_LIBRARY_PATH for ROS runtime.
if [ -n "${LD_LIBRARY_PATH}" ]; then
	CLEANED_LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -vE '^/opt/MVS/lib/(32|64)$' | awk 'NF && !seen[$0]++' | paste -sd ':' -)"
	export LD_LIBRARY_PATH="${CLEANED_LD_LIBRARY_PATH}"
fi

# Force system libusb to avoid symbol mismatch with libpcl_io.
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:${LD_PRELOAD}}"

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True