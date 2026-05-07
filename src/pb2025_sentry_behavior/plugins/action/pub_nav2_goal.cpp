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

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  const auto ports = config().input_ports;
  const bool goal_pose_from_xml =
    ports.find("goal_pose") != ports.end() && !ports.at("goal_pose").empty();

  if (goal_pose_from_xml) {
    auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
    if (!goal_pose) {
      RCLCPP_ERROR(
        node_->get_logger(), "PubNav2Goal: invalid [goal_pose]: %s", goal_pose.error().c_str());
      return false;
    }
    msg = goal_pose.value();
    msg.header.stamp = now();
    if (msg.header.frame_id.empty()) {
      auto frame_in = getInput<std::string>("frame_id");
      msg.header.frame_id =
        (frame_in && !frame_in.value().empty()) ? frame_in.value() : std::string("map");
    }
    return true;
  }

  auto goal = getInput<std::string>("goal");
  if (!goal) {
    throw BT::RuntimeError("missing input [goal]: ", goal.error());
  }

  const auto parsed_goal = poseStampedFromString(goal.value());

  msg.header.stamp = now();
  auto frame_in = getInput<std::string>("frame_id");
  msg.header.frame_id =
    (frame_in && !frame_in.value().empty()) ? frame_in.value() : std::string("map");
  msg.pose = parsed_goal.pose;
  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "When set (e.g. {supply} from params/pose.yaml), overrides [goal]"),
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>("frame_id", "map", "Frame id when goal_pose has empty frame_id"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
