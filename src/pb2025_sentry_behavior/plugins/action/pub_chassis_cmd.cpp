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

#include "pb2025_sentry_behavior/plugins/action/pub_chassis_cmd.hpp"

#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

PublishChassisCmdAction::PublishChassisCmdAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishChassisCmdAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<unsigned>("reset", pb_rm_interfaces::msg::ChassisCmd::DISABLE, "Reset chassis"),
    BT::InputPort<unsigned>(
      "rotate", pb_rm_interfaces::msg::ChassisCmd::DISABLE, "Rotate chassis"),
  });
}

bool PublishChassisCmdAction::setMessage(pb_rm_interfaces::msg::ChassisCmd & msg)
{
  unsigned reset = pb_rm_interfaces::msg::ChassisCmd::DISABLE;
  unsigned rotate = pb_rm_interfaces::msg::ChassisCmd::DISABLE;

  getInput("reset", reset);
  getInput("rotate", rotate);

  msg.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  msg.reset = reset == pb_rm_interfaces::msg::ChassisCmd::ENABLE ?
    pb_rm_interfaces::msg::ChassisCmd::ENABLE : pb_rm_interfaces::msg::ChassisCmd::DISABLE;
  msg.rotate = rotate == pb_rm_interfaces::msg::ChassisCmd::ENABLE ?
    pb_rm_interfaces::msg::ChassisCmd::ENABLE : pb_rm_interfaces::msg::ChassisCmd::DISABLE;

  return true;
}

bool PublishChassisCmdAction::setHaltMessage(pb_rm_interfaces::msg::ChassisCmd & msg)
{
  msg.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  msg.reset = pb_rm_interfaces::msg::ChassisCmd::DISABLE;
  msg.rotate = pb_rm_interfaces::msg::ChassisCmd::DISABLE;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishChassisCmdAction, "PublishChassisCmd");