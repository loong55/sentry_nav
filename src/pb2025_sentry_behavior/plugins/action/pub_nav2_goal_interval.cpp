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

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal_interval.hpp"

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalIntervalAction::PubNav2GoalIntervalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalIntervalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(logger(), "Failed to read input port [goal]: %s", goal.error().c_str());
    return false;
  }

  msg = goal.value();

  if (msg.header.frame_id.empty()) {
    auto frame_id = getInput<std::string>("frame_id");
    msg.header.frame_id =
      (frame_id && !frame_id.value().empty()) ? frame_id.value() : std::string("map");
  }

  msg.header.stamp = now();

  RCLCPP_DEBUG(
    logger(), "pub_nav2_goal_interval publish goal [%.2f, %.2f, %.2f] in frame %s",
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.header.frame_id.c_str());
  return true;
}

BT::PortsList PubNav2GoalIntervalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>("frame_id", "map", "Goal frame id"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalIntervalAction, "pub_nav2_goal_interval");