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

#include "pb2025_sentry_behavior/plugins/condition/need_supply.hpp"

namespace pb2025_sentry_behavior
{

NeedSupplyCondition::NeedSupplyCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus NeedSupplyCondition::tick()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int enter_hp_below = 150;
  int enter_ammo_below = 20;
  int exit_hp_min = 350;
  int exit_ammo_min = 200;

  getInput("enter_hp_below", enter_hp_below);
  getInput("enter_ammo_below", enter_ammo_below);
  getInput("exit_hp_min", exit_hp_min);
  getInput("exit_ammo_min", exit_ammo_min);

  const int current_hp = msg->current_hp;
  const int current_ammo = msg->projectile_allowance_17mm;

  if (!need_supply_ && (current_hp < enter_hp_below || current_ammo < enter_ammo_below)) {
    need_supply_ = true;
  } else if (
    need_supply_ && current_hp >= exit_hp_min && current_ammo >= exit_ammo_min)
  {
    need_supply_ = false;
  }

  return need_supply_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList NeedSupplyCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>("enter_hp_below", 150, "Enter supply mode when hp is below this value"),
    BT::InputPort<int>(
      "enter_ammo_below", 20, "Enter supply mode when ammo is below this value"),
    BT::InputPort<int>("exit_hp_min", 350, "Exit supply mode when hp reaches this value"),
    BT::InputPort<int>(
      "exit_ammo_min", 200, "Exit supply mode when ammo reaches this value")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::NeedSupplyCondition>("NeedSupply");
}