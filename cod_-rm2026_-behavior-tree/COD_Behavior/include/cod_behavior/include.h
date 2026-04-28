#pragma once
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/tree_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <coroutine>
#include <unordered_map>
#include <chrono>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include <fstream>
#include <sstream>

#include <../../BehaviorTree.ROS2/behaviortree_ros2/include/behaviortree_ros2/bt_action_node.hpp>
//读取自定义的航点函数
geometry_msgs::msg::PoseStamped
loadPoseStamped(
    const rclcpp::Node::SharedPtr &node,
    const std::string &prefix) {
    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id =
            node->get_parameter(prefix + ".frame_id").as_string();

    pose.pose.position.x =
            node->get_parameter(prefix + ".position.x").as_double();
    pose.pose.position.y =
            node->get_parameter(prefix + ".position.y").as_double();
    pose.pose.position.z =
            node->get_parameter(prefix + ".position.z").as_double();
    pose.pose.orientation.w =
            node->get_parameter(prefix + ".orientation.w").as_double();
    return pose;
}

// 从 waypoint_editor 生成的 CSV 文件中加载航点列表
// CSV 格式: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,wait_sec
// wait_sec 列可选，缺省为 0（不等待）
inline bool loadWaypointsFromCSV(
    const std::string &filename,
    std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
    const std::string &frame_id = "map",
    std::vector<double> *wait_times = nullptr) {
        std::ifstream file(filename);
        if (!file.is_open()) {
                return false;
        }

        std::string line;
        // 跳过 CSV 表头
        std::getline(file, line);

        while (std::getline(file, line)) {
                if (line.empty()) continue;

                std::istringstream ss(line);
                std::string token;
                std::vector<std::string> tokens;

                while (std::getline(ss, token, ',')) {
                        tokens.push_back(token);
                }

                // 至少需要 8 列: id, pose_x, pose_y, pose_z, rot_x, rot_y, rot_z, rot_w
                if (tokens.size() < 8) {
                        continue;
                }

                try {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header.frame_id = frame_id;
                        pose.header.stamp = rclcpp::Time(0);

                        pose.pose.position.x = std::stod(tokens[1]);
                        pose.pose.position.y = std::stod(tokens[2]);
                        pose.pose.position.z = std::stod(tokens[3]);

                        pose.pose.orientation.x = std::stod(tokens[4]);
                        pose.pose.orientation.y = std::stod(tokens[5]);
                        pose.pose.orientation.z = std::stod(tokens[6]);
                        pose.pose.orientation.w = std::stod(tokens[7]);

                        waypoints.push_back(pose);

                        // 第 9 列 (index 9) 为 wait_sec，可选
                        if (wait_times) {
                                double wt = 0.0;
                                if (tokens.size() > 9 && !tokens[9].empty()) {
                                        wt = std::stod(tokens[9]);
                                }
                                wait_times->push_back(wt);
                        }
                } catch (const std::exception &) {
                        continue;
                }
        }

        file.close();
        return !waypoints.empty();
}