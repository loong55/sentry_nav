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

#include "pb2025_sentry_behavior/plugins/action/navigate_through_poses.hpp"

#include <vector>

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateThroughPoses>(name, conf, params)
{
}

bool NavigateThroughPosesAction::setGoal(nav2_msgs::action::NavigateThroughPoses::Goal & goal)
{
  auto receive_waypoint_file = getInput<std::string>("waypoint_file");
  auto receive_frame_id = getInput<std::string>("frame_id");

  if (!receive_waypoint_file) {
    RCLCPP_ERROR(logger(), "NavigateThroughPosesAction missing waypoint_file: %s", receive_waypoint_file.error().c_str());
    return false;
  }

  const std::string frame_id =
    receive_frame_id && !receive_frame_id->empty() ? receive_frame_id.value() : "map";

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  if (!loadWaypointsFromCSV(receive_waypoint_file.value(), waypoints, frame_id)) {
    RCLCPP_ERROR(
      logger(), "NavigateThroughPosesAction failed to load waypoints from: %s",
      receive_waypoint_file.value().c_str());
    return false;
  }

  total_waypoints_ = static_cast<int>(waypoints.size());
  const auto stamp = now();
  for (auto & waypoint : waypoints) {
    waypoint.header.frame_id = frame_id;
    waypoint.header.stamp = stamp;
  }

  goal.poses = waypoints;

  RCLCPP_INFO(
    logger(), "NavigateThroughPosesAction loaded %d waypoints from %s", total_waypoints_,
    resolvePackageSharePath(receive_waypoint_file.value()).c_str());
  return true;
}

BT::NodeStatus NavigateThroughPosesAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "NavigateThroughPosesAction succeeded");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "NavigateThroughPosesAction aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "NavigateThroughPosesAction canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(
        logger(), "NavigateThroughPosesAction unknown result code: %d",
        static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus NavigateThroughPosesAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
{
  const int current_waypoint = total_waypoints_ - feedback->number_of_poses_remaining;
  setOutput("current_waypoint", current_waypoint);
  RCLCPP_DEBUG(
    logger(), "NavigateThroughPosesAction progressing waypoint %d/%d", current_waypoint,
    total_waypoints_);
  return BT::NodeStatus::RUNNING;
}

void NavigateThroughPosesAction::onHalt()
{
  RCLCPP_INFO(logger(), "NavigateThroughPosesAction has been halted.");
}

BT::NodeStatus NavigateThroughPosesAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "NavigateThroughPosesAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList NavigateThroughPosesAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<std::string>("waypoint_file", "CSV waypoint file path"),
    BT::InputPort<std::string>("frame_id", "map", "Waypoint frame id"),
    BT::OutputPort<int>("current_waypoint", "Current waypoint index from feedback"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::NavigateThroughPosesAction, "NavigateThroughPosesAction");