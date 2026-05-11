source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5  # 宿主机在 5 号网段

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=gazebo_test_field
# ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc_2026fix
# ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc_2026
# ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc_2025
# ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=rmuc_2024
