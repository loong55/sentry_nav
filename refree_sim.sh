#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rqt &
ros2 launch referee_sim referee_sim.launch.py
