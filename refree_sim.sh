#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rqt --force-discover &
# ros2 launch referee_sim referee_sim.launch.py
ros2 launch referee_sim referee_sim.launch.py namespace:=red_standard_robot1
