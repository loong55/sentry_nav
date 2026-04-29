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

#include "pb2025_sentry_behavior/plugins/condition/is_robot_near_pose.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

IsRobotNearPoseCondition::IsRobotNearPoseCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
  ensureNode();
}

bool IsRobotNearPoseCondition::ensureNode()
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

BT::PortsList IsRobotNearPoseCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Target pose in global frame. Fill with format `x;y;yaw`"),
    BT::InputPort<double>("tolerance", 0.3, "Position tolerance in meters"),
    BT::InputPort<std::string>("global_frame", "map", "Global frame for the goal pose"),
    BT::InputPort<std::string>("robot_base_frame", "chassis", "Robot base frame")};
}

BT::NodeStatus IsRobotNearPoseCondition::tick()
{
  if (!ensureNode()) {
    throw BT::RuntimeError("Missing ROS node in blackboard with key [node]");
  }

  auto goal = getInput<std::string>("goal");
  auto tolerance = getInput<double>("tolerance");
  auto global_frame = getInput<std::string>("global_frame");
  auto robot_base_frame = getInput<std::string>("robot_base_frame");

  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "IsRobotNearPose: missing input [goal]: %s", goal.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!tolerance) {
    RCLCPP_ERROR(
      node_->get_logger(), "IsRobotNearPose: missing input [tolerance]: %s",
      tolerance.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto parsed_goal = poseStampedFromString(goal.value());

  geometry_msgs::msg::PoseStamped current_pose;
  const std::string target_frame = global_frame ? global_frame.value() : "map";
  const std::string base_frame = robot_base_frame ? robot_base_frame.value() : "chassis";
  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, target_frame, base_frame, 0.2)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRobotNearPose: failed to get robot pose from TF [%s -> %s]",
      target_frame.c_str(), base_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double distance = std::hypot(
    current_pose.pose.position.x - parsed_goal.pose.position.x,
    current_pose.pose.position.y - parsed_goal.pose.position.y);

  if (distance <= tolerance.value()) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "IsRobotNearPose: target reached, distance=%.3f tolerance=%.3f",
      distance, tolerance.value());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsRobotNearPoseCondition>("IsRobotNearPose");
}