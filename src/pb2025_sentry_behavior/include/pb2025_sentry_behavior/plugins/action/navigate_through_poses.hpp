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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace pb2025_sentry_behavior
{
class NavigateThroughPosesAction
: public BT::StatefulActionNode
{
public:
  using ActionType = nav2_msgs::action::NavigateThroughPoses;
  using ActionClient = rclcpp_action::Client<ActionType>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using WrappedResult = GoalHandle::WrappedResult;

  NavigateThroughPosesAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  bool ensureClient();
  bool buildGoal(ActionType::Goal & goal);
  BT::NodeStatus handleResult(const WrappedResult & wr);
  rclcpp::Logger logger() const;
  rclcpp::Time now() const;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ActionClient> action_client_;
  std::string default_action_name_;
  std::string action_name_;
  std::chrono::milliseconds server_timeout_;
  std::chrono::milliseconds wait_for_server_timeout_;
  std::chrono::steady_clock::time_point goal_request_time_;
  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  std::shared_future<WrappedResult> future_result_;
  GoalHandle::SharedPtr goal_handle_;
  int total_waypoints_{0};
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_HPP_