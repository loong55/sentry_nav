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

#include "pb2025_sentry_behavior/plugins/condition/need_outpost_defense.hpp"

namespace pb2025_sentry_behavior
{

NeedOutpostDefenseCondition::NeedOutpostDefenseCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus NeedOutpostDefenseCondition::tick()
{
  auto msg = getInput<pb_rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int enter_outpost_hp_below = 20;
  int exit_outpost_hp_min = 300;

  getInput("enter_outpost_hp_below", enter_outpost_hp_below);
  getInput("exit_outpost_hp_min", exit_outpost_hp_min);

  const int current_outpost_hp = static_cast<int>(msg->ally_outpost_hp);

  if (!need_outpost_defense_ && current_outpost_hp < enter_outpost_hp_below) {
    need_outpost_defense_ = true;
  } else if (need_outpost_defense_ && current_outpost_hp > exit_outpost_hp_min) {
    need_outpost_defense_ = false;
  }

  return need_outpost_defense_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList NeedOutpostDefenseCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard"),
    BT::InputPort<int>(
      "enter_outpost_hp_below", 20, "Enter defense mode when outpost hp is below this value"),
    BT::InputPort<int>(
      "exit_outpost_hp_min", 300, "Exit defense mode when outpost hp is above this value")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::NeedOutpostDefenseCondition>(
    "NeedOutpostDefense");
}