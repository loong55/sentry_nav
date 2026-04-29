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

#include "pb2025_sentry_behavior/plugins/condition/is_game_status.hpp"

namespace pb2025_sentry_behavior
{

IsGameStatusCondition::IsGameStatusCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameStatusCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameStatusCondition::checkGameStart()
{
  int expected_game_progress = 4;
  int min_remain_time = 0;
  int max_remain_time = 420;
  bool wait_for_message = false;
  bool wait_until_expected = false;
  auto msg = getInput<pb_rm_interfaces::msg::GameStatus>("key_port");
  getInput("wait_for_message", wait_for_message);
  getInput("wait_until_expected", wait_until_expected);

  if (!msg) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock, 3000, "GameStatus message is not available");
    return wait_for_message ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }

  getInput("expected_game_progress", expected_game_progress);
  getInput("min_remain_time", min_remain_time);
  getInput("max_remain_time", max_remain_time);

  RCLCPP_DEBUG(
    logger_, "Checking: Progress(%d/%d), Remain Time(%ds) in [%d-%d]",
    static_cast<int>(msg->game_progress), expected_game_progress, msg->stage_remain_time,
    min_remain_time, max_remain_time);

  const bool is_progress_match = (msg->game_progress == expected_game_progress);
  const bool is_time_in_range =
    (msg->stage_remain_time >= min_remain_time) && (msg->stage_remain_time <= max_remain_time);

  if (is_progress_match && is_time_in_range) {
    return BT::NodeStatus::SUCCESS;
  }

  return wait_until_expected ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGameStatusCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "key_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
    BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
    BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
    BT::InputPort<bool>(
      "wait_for_message", false,
      "Return RUNNING instead of FAILURE while waiting for the first GameStatus message"),
    BT::InputPort<bool>(
      "wait_until_expected", false,
      "Return RUNNING instead of FAILURE while GameStatus has not reached the expected stage/time")
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}
