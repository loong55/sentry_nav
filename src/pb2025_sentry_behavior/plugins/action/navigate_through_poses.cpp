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

#include "pb2025_sentry_behavior/plugins/action/navigate_through_poses.hpp"

#include <exception>
#include <vector>

#include "behaviortree_ros2/plugins.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

NavigateThroughPosesAction::NavigateThroughPosesAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  node_(params.nh.lock()),
  default_action_name_(params.default_port_value),
  server_timeout_(params.server_timeout),
  wait_for_server_timeout_(params.wait_for_server_timeout)
{
  if (!node_) {
    throw BT::RuntimeError("NavigateThroughPosesAction: ROS node expired");
  }
}

bool NavigateThroughPosesAction::ensureClient()
{
  std::string requested_action_name;
  getInput("action_name", requested_action_name);
  if (requested_action_name.empty()) {
    requested_action_name = default_action_name_;
  }
  if (requested_action_name.empty()) {
    RCLCPP_ERROR(logger(), "NavigateThroughPosesAction action_name is empty");
    return false;
  }

  if (action_client_ && requested_action_name == action_name_) {
    return true;
  }

  action_name_ = requested_action_name;
  action_client_ = rclcpp_action::create_client<ActionType>(node_, action_name_);
  if (!action_client_->wait_for_action_server(wait_for_server_timeout_)) {
    RCLCPP_ERROR(
      logger(), "NavigateThroughPosesAction action server not reachable: %s",
      action_name_.c_str());
    return false;
  }
  return true;
}

bool NavigateThroughPosesAction::buildGoal(ActionType::Goal & goal)
{
  auto receive_waypoint_file = getInput<std::string>("waypoint_file");
  auto receive_frame_id = getInput<std::string>("frame_id");

  if (!receive_waypoint_file) {
    RCLCPP_ERROR(
      logger(), "NavigateThroughPosesAction missing waypoint_file: %s",
      receive_waypoint_file.error().c_str());
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

BT::NodeStatus NavigateThroughPosesAction::handleResult(const WrappedResult & wr)
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

BT::NodeStatus NavigateThroughPosesAction::onStart()
{
  if (!ensureClient()) {
    return BT::NodeStatus::FAILURE;
  }

  ActionType::Goal goal;
  if (!buildGoal(goal)) {
    return BT::NodeStatus::FAILURE;
  }

  goal_handle_.reset();
  future_goal_handle_ = {};
  future_result_ = {};
  setOutput("current_waypoint", 0);

  future_goal_handle_ = action_client_->async_send_goal(goal);
  goal_request_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateThroughPosesAction::onRunning()
{
  if (!goal_handle_) {
    if (future_goal_handle_.valid() &&
      future_goal_handle_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      goal_handle_ = future_goal_handle_.get();
      future_goal_handle_ = {};

      if (!goal_handle_) {
        RCLCPP_ERROR(logger(), "NavigateThroughPosesAction goal rejected by server");
        return BT::NodeStatus::FAILURE;
      }

      future_result_ = action_client_->async_get_result(goal_handle_);
    }
  }

  if (!goal_handle_) {
    const auto elapsed = std::chrono::steady_clock::now() - goal_request_time_;
    if (elapsed > server_timeout_) {
      RCLCPP_ERROR(
        logger(),
        "NavigateThroughPosesAction timed out waiting for goal response after %.2fs",
        std::chrono::duration<double>(elapsed).count());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  if (future_result_.valid() &&
    future_result_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
    setOutput("current_waypoint", total_waypoints_);
    return handleResult(future_result_.get());
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateThroughPosesAction::onHalted()
{
  RCLCPP_INFO(logger(), "NavigateThroughPosesAction has been halted.");

  if (!action_client_ || !goal_handle_) {
    return;
  }

  try {
    action_client_->async_cancel_goal(goal_handle_);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger(), "NavigateThroughPosesAction cancel failed during halt: %s", error.what());
  }

  goal_handle_.reset();
  future_goal_handle_ = {};
  future_result_ = {};
}

BT::PortsList NavigateThroughPosesAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "navigate_through_poses", "Action server name"),
    BT::InputPort<std::string>("waypoint_file", "CSV waypoint file path"),
    BT::InputPort<std::string>("frame_id", "map", "Waypoint frame id"),
    BT::OutputPort<int>("current_waypoint", "Current waypoint index from feedback"),
  };
}

rclcpp::Logger NavigateThroughPosesAction::logger() const
{
  return node_->get_logger();
}

rclcpp::Time NavigateThroughPosesAction::now() const
{
  return node_->now();
}

}  // namespace pb2025_sentry_behavior

CreateRosNodePlugin(pb2025_sentry_behavior::NavigateThroughPosesAction, "NavigateThroughPosesAction");