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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__NEED_OUTPOST_DEFENSE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__NEED_OUTPOST_DEFENSE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
class NeedOutpostDefenseCondition : public BT::ConditionNode
{
public:
  NeedOutpostDefenseCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("NeedOutpostDefenseCondition");
  bool need_outpost_defense_ = false;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__NEED_OUTPOST_DEFENSE_HPP_