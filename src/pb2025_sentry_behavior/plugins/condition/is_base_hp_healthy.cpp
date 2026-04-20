#include "pb2025_sentry_behavior/plugins/condition/is_base_hp_healthy.hpp"

namespace pb2025_sentry_behavior
{

IsBaseHPHealthyCondition::IsBaseHPHealthyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsBaseHPHealthyCondition::checkGameRobotHP, this), config)
{
}

BT::NodeStatus IsBaseHPHealthyCondition::checkGameRobotHP()
{
  int base_hp_min;

  auto msg = getInput<pb_rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("base_hp_min", base_hp_min);

  const bool is_base_hp_ok = (static_cast<int>(msg->ally_base_hp) >= base_hp_min);

  return is_base_hp_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsBaseHPHealthyCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard"),
    BT::InputPort<int>("base_hp_min", 3000, "Minimum ally base HP"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsBaseHPHealthyCondition>("IsBaseHPHealthy");
}
