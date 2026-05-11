// Copyright 2026 Huang Hao
#include "pb2025_sentry_behavior/plugins/condition/is_outpost_hp_healthy.hpp"

namespace pb2025_sentry_behavior
{

IsOutpostHPHealthyCondition::IsOutpostHPHealthyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsOutpostHPHealthyCondition::checkGameRobotHP, this), config)
{
}

BT::NodeStatus IsOutpostHPHealthyCondition::checkGameRobotHP()
{
  int outpost_hp_min;

  auto msg = getInput<pb_rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("outpost_hp_min", outpost_hp_min);

  const bool is_outpost_hp_ok = (static_cast<int>(msg->ally_outpost_hp) >= outpost_hp_min);

  return is_outpost_hp_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsOutpostHPHealthyCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard"),
    BT::InputPort<int>("outpost_hp_min", 500, "Minimum ally outpost HP"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsOutpostHPHealthyCondition>(
    "IsOutpostHPHealthy");
}
