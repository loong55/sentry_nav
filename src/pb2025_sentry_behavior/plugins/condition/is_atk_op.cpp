
#include "pb2025_sentry_behavior/plugins/condition/is_atk_op.hpp"

namespace pb2025_sentry_behavior
{

IsATKOPCondition::IsATKOPCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsATKOPCondition::checkHPStatus, this), config)
{
}

BT::NodeStatus IsATKOPCondition::checkHPStatus()
{
  int op_hp;
  auto msg = getInput<pb_rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "HP message is not available");
    return BT::NodeStatus::FAILURE;
    
  }

  getInput("op_hp", op_hp);
  RCLCPP_DEBUG(
    logger_, "Checking: red_outpost_hp(%d)",
    static_cast<int>(msg->red_outpost_hp));

  const bool op_down = (msg->red_outpost_hp <= op_hp);
  return (op_down) ? BT::NodeStatus::FAILURE: BT::NodeStatus::SUCCESS;

}

BT::PortsList IsATKOPCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "GameRobotHP port on blackboard"),
    BT::InputPort<int>("op_hp", 0, "前哨血量"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsATKOPCondition>("IsATKOP");
}