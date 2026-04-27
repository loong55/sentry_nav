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

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
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
  action_client_ = rclcpp_action::create_client<ActionType>(node_, action_name_, callback_group_);
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
  result_ = {};
  result_.code = rclcpp_action::ResultCode::UNKNOWN;
  goal_response_received_ = false;
  goal_rejected_ = false;
  result_received_ = false;

  typename ActionClient::SendGoalOptions options;
  options.goal_response_callback = [this](GoalHandle::SharedPtr goal_handle) {
    goal_response_received_ = true;
    goal_handle_ = goal_handle;
    goal_rejected_ = (goal_handle == nullptr);
  };
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const ActionType::Feedback> feedback) {
      const int current_waypoint = total_waypoints_ - feedback->number_of_poses_remaining;
      setOutput("current_waypoint", current_waypoint);
      RCLCPP_DEBUG(
        logger(), "NavigateThroughPosesAction progressing waypoint %d/%d", current_waypoint,
        total_waypoints_);
    };
  options.result_callback = [this](const WrappedResult & result) {
    result_ = result;
    result_received_ = true;
  };

  future_goal_handle_ = action_client_->async_send_goal(goal, options);
  goal_request_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateThroughPosesAction::onRunning()
{
  callback_executor_.spin_some();

  if (!goal_response_received_ && future_goal_handle_.valid()) {
    const auto ret = callback_executor_.spin_until_future_complete(
      future_goal_handle_, std::chrono::milliseconds(0));
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_.get();
      future_goal_handle_ = {};
      goal_response_received_ = true;
      goal_rejected_ = (goal_handle_ == nullptr);
    }
  }

  if (result_received_) {
    return handleResult(result_);
  }

  if (goal_rejected_) {
    RCLCPP_ERROR(logger(), "NavigateThroughPosesAction goal rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_response_received_) {
    const auto elapsed = std::chrono::steady_clock::now() - goal_request_time_;
    if (elapsed > server_timeout_) {
      RCLCPP_ERROR(
        logger(),
        "NavigateThroughPosesAction timed out waiting for goal response after %.2fs",
        std::chrono::duration<double>(elapsed).count());
      return BT::NodeStatus::FAILURE;
    }
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
    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    callback_executor_.spin_until_future_complete(cancel_future, server_timeout_);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger(), "NavigateThroughPosesAction cancel failed during halt: %s", error.what());
  }

  goal_handle_.reset();
  future_goal_handle_ = {};
  goal_response_received_ = false;
  goal_rejected_ = false;
  result_received_ = false;
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