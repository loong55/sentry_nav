// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef PB2025_SENTRY_BEHAVIOR__CUSTOM_TYPES_HPP_
#define PB2025_SENTRY_BEHAVIOR__CUSTOM_TYPES_HPP_

namespace pb2025_sentry_behavior
{
inline geometry_msgs::msg::PoseStamped poseStampedFromString(const BT::StringView & key)
{
  auto parts = BT::splitString(key, ';');
  if (parts.size() == 7) {
    geometry_msgs::msg::PoseStamped output;
    output.pose.position.x = BT::convertFromString<double>(parts[0]);
    output.pose.position.y = BT::convertFromString<double>(parts[1]);
    output.pose.position.z = BT::convertFromString<double>(parts[2]);
    output.pose.orientation.x = BT::convertFromString<double>(parts[3]);
    output.pose.orientation.y = BT::convertFromString<double>(parts[4]);
    output.pose.orientation.z = BT::convertFromString<double>(parts[5]);
    output.pose.orientation.w = BT::convertFromString<double>(parts[6]);
    return output;
  }

  if (parts.size() == 3) {
    tf2::Quaternion quaternion;
    auto goal_yaw = BT::convertFromString<double>(parts[2]);
    quaternion.setRPY(0, 0, goal_yaw);

    geometry_msgs::msg::PoseStamped output;
    output.pose.position.x = BT::convertFromString<double>(parts[0]);
    output.pose.position.y = BT::convertFromString<double>(parts[1]);
    output.pose.position.z = 0.0;
    output.pose.orientation = tf2::toMsg(quaternion);
    return output;
  }

  throw BT::RuntimeError("Invalid PoseStamped input. Expected 'x;y;yaw' or 'x;y;z;qx;qy;qz;qw'");
}

inline std::string resolvePackageSharePath(const std::string & path)
{
  namespace fs = std::filesystem;

  if (path.empty()) {
    return path;
  }

  const fs::path candidate(path);
  if (candidate.is_absolute() || fs::exists(candidate)) {
    return candidate.string();
  }

  const fs::path package_share(
    ament_index_cpp::get_package_share_directory("pb2025_sentry_behavior"));
  const fs::path package_relative = package_share / candidate;
  if (fs::exists(package_relative)) {
    return package_relative.string();
  }

  return candidate.string();
}

inline bool loadWaypointsFromCSV(
  const std::string & file_path,
  std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
  const std::string & frame_id = "map")
{
  std::ifstream file(resolvePackageSharePath(file_path));
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::istringstream stream(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(stream, token, ',')) {
      tokens.push_back(token);
    }

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
    } catch (const std::exception &) {
      continue;
    }
  }

  return !waypoints.empty();
}
}  // namespace pb2025_sentry_behavior

namespace BT
{
template <>
geometry_msgs::msg::PoseStamped convertFromString(StringView key)
{
  return pb2025_sentry_behavior::poseStampedFromString(key);
}
}  // namespace BT

#endif  // PB2025_SENTRY_BEHAVIOR__CUSTOM_TYPES_HPP_
