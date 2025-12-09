
#include "pb2025_sentry_behavior/plugins/condition/is_energy_enough.hpp"

namespace pb2025_sentry_behavior
{

IsEnergyEnoughCondition::IsEnergyEnoughCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsEnergyEnoughCondition::checkBuffStatus, this), config)
{
}

BT::NodeStatus IsEnergyEnoughCondition::checkBuffStatus()
{
  int remaining_energy;
  auto msg = getInput<pb_rm_interfaces::msg::Buff>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "Buff message is not available");
    return BT::NodeStatus::FAILURE;
    
  }

  getInput("remaining_energy", remaining_energy);
  RCLCPP_DEBUG(
    logger_, "Checking: remaining_energy(%d)",
    static_cast<int>(msg->remaining_energy));

  const bool less_energy = (msg->remaining_energy <= remaining_energy);
  return (less_energy) ? BT::NodeStatus::FAILURE: BT::NodeStatus::SUCCESS;

}

BT::PortsList IsEnergyEnoughCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::Buff>(
      "key_port", "{@referee_buff}", "Buff port on blackboard"),
    BT::InputPort<int>("remaining_energy", 20, "剩余能量"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsEnergyEnoughCondition>("IsEnergyEnough");
}