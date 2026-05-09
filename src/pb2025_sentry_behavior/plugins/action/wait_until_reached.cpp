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

#include "pb2025_sentry_behavior/plugins/action/wait_until_reached.hpp"

#include <cmath>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_util/robot_utils.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

WaitUntilReachedAction::WaitUntilReachedAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), tolerance_(0.3)
{
  ensureNode();
}

bool WaitUntilReachedAction::ensureNode()
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

BT::PortsList WaitUntilReachedAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      "Optional: {supply},{fort},... from params/pose.yaml blackboard; overrides [goal]"),
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Target pose in global frame. Fill with format `x;y;yaw`"),
    BT::InputPort<double>("tolerance", 0.3, "Position tolerance in meters"),
    BT::InputPort<std::string>("global_frame", "map", "Global frame for the goal pose"),
    BT::InputPort<std::string>("robot_base_frame", "chassis", "Robot base frame")};
}

BT::NodeStatus WaitUntilReachedAction::onStart()
{
  if (!ensureNode()) {
    throw BT::RuntimeError("Missing ROS node in blackboard with key [node]");
  }

  auto tolerance = getInput<double>("tolerance");
  auto global_frame = getInput<std::string>("global_frame");
  auto robot_base_frame = getInput<std::string>("robot_base_frame");

  if (!tolerance) {
    RCLCPP_ERROR(
      node_->get_logger(), "WaitUntilReached: missing input [tolerance]: %s",
      tolerance.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto ports = config().input_ports;
  const bool goal_pose_from_xml =
    ports.find("goal_pose") != ports.end() && !ports.at("goal_pose").empty();

  if (goal_pose_from_xml) {
    auto goal_pose_in = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!goal_pose_in) {
      RCLCPP_ERROR(
        node_->get_logger(), "WaitUntilReached: invalid [goal_pose]: %s",
        goal_pose_in.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    goal_pose_ = goal_pose_in.value();
  } else {
    auto goal = getInput<std::string>("goal");
    if (!goal) {
      RCLCPP_ERROR(
        node_->get_logger(), "WaitUntilReached: missing input [goal]: %s",
        goal.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    goal_pose_ = poseStampedFromString(goal.value());
  }

  tolerance_ = tolerance.value();
  global_frame_ = global_frame ? global_frame.value() : "map";
  robot_base_frame_ = robot_base_frame ? robot_base_frame.value() : "chassis";

  if (goal_pose_.header.frame_id.empty()) {
    goal_pose_.header.frame_id = global_frame_;
  }

  RCLCPP_INFO(
    node_->get_logger(), "WaitUntilReached: waiting for [%.2f, %.2f], tolerance=%.2f",
    goal_pose_.pose.position.x, goal_pose_.pose.position.y, tolerance_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilReachedAction::onRunning()
{
  geometry_msgs::msg::PoseStamped current_pose;
  const std::string target_frame =
    goal_pose_.header.frame_id.empty() ? global_frame_ : goal_pose_.header.frame_id;
  if (!nav2_util::getCurrentPose(current_pose, *tf_buffer_, target_frame, robot_base_frame_, 0.2)) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "WaitUntilReached: failed to get robot pose from TF [%s -> %s]",
      target_frame.c_str(), robot_base_frame_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  const double distance = std::hypot(
    current_pose.pose.position.x - goal_pose_.pose.position.x,
    current_pose.pose.position.y - goal_pose_.pose.position.y);

  if (distance <= tolerance_) {
    RCLCPP_INFO(
      node_->get_logger(), "WaitUntilReached: target reached, distance=%.3f tolerance=%.3f",
      distance, tolerance_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitUntilReachedAction::onHalted()
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "WaitUntilReached: halted");
  }
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::WaitUntilReachedAction>("WaitUntilReached");
}