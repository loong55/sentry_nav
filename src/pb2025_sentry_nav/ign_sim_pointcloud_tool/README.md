# isaac_sim_pointcloud_tool

This package converts pointcloud in Ignition Gazebo to velodyne format. This is a ROS package. It subscribes to the LiDAR rostopic which is published by Ignition Gazebo. And it republish a LiDAR rostopoic in Velodyne format.

Some SLAM algorithm needs pointcloud in Velodyne format so that it can extract corner points. But Isaac ROS only send pointcloud contains XYZ information. This package helps to convert pointcloud to velodyne format.

# 艾萨克模拟点云工具

此包将Ignition Gazebo中的点云转换为Velodyne格式。这是一个ROS包。它订阅Ignition Gazebo发布的LiDAR rostopic。并且它以Velodyne格式重新发布LiDAR rostopic。

一些SLAM算法需要Velodyne格式的点云，以便提取角点。但是Isaac ROS只发送包含XYZ信息的点云。此包有助于将点云转换为Velodyne格式。
