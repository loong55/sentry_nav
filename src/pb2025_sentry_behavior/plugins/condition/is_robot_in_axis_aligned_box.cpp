// Copyright 2026 Huang Hao
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

#include "pb2025_sentry_behavior/plugins/condition/is_robot_in_axis_aligned_box.hpp"

#include <algorithm>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

namespace pb2025_sentry_behavior
{

IsRobotInAxisAlignedBoxCondition::IsRobotInAxisAlignedBoxCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
  ensureNode();
}

bool IsRobotInAxisAlignedBoxCondition::ensureNode()
{
  if (node_) {
    return true;
  }

  if (config().blackboard) {
    if (!config().blackboard->get("node", node_)) {
      if (auto * root_blackboard = config().blackboard->rootBlackboard()) {
        (void)root_blackboard->get("node", node_);
      }
    }
  }

  if (!node_) {
    return false;
  }

  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
  }

  return true;
}

BT::PortsList IsRobotInAxisAlignedBoxCondition::providedPorts()
{
  return {
    BT::InputPort<double>("min_x", "Lower x bound in global frame"),
    BT::InputPort<double>("max_x", "Upper x bound in global frame"),
    BT::InputPort<double>("min_y", "Lower y bound in global frame"),
    BT::InputPort<double>("max_y", "Upper y bound in global frame"),
    BT::InputPort<std::string>("global_frame", "map", "Global frame for bounds"),
    BT::InputPort<std::string>("robot_base_frame", "chassis", "Robot base frame")};
}

BT::NodeStatus IsRobotInAxisAlignedBoxCondition::tick()
{
  if (!ensureNode()) {
    throw BT::RuntimeError("Missing ROS node in blackboard with key [node]");
  }

  auto min_x = getInput<double>("min_x");
  auto max_x = getInput<double>("max_x");
  auto min_y = getInput<double>("min_y");
  auto max_y = getInput<double>("max_y");
  auto global_frame = getInput<std::string>("global_frame");
  auto robot_base_frame = getInput<std::string>("robot_base_frame");

  if (!min_x || !max_x || !min_y || !max_y) {
    RCLCPP_ERROR(node_->get_logger(), "IsRobotInAxisAlignedBox: missing bounds input");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  const std::string target_frame = global_frame ? global_frame.value() : "map";
  const std::string base_frame = robot_base_frame ? robot_base_frame.value() : "chassis";
  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, target_frame, base_frame, 0.2)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRobotInAxisAlignedBox: failed to get robot pose from TF [%s -> %s]",
      target_frame.c_str(), base_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double lower_x = std::min(min_x.value(), max_x.value());
  const double upper_x = std::max(min_x.value(), max_x.value());
  const double lower_y = std::min(min_y.value(), max_y.value());
  const double upper_y = std::max(min_y.value(), max_y.value());
  const double x = current_pose.pose.position.x;
  const double y = current_pose.pose.position.y;

  if (x >= lower_x && x <= upper_x && y >= lower_y && y <= upper_y) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsRobotInAxisAlignedBoxCondition>(
    "IsRobotInAxisAlignedBox");
}