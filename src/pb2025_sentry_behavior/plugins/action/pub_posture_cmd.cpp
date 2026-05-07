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

#include "pb2025_sentry_behavior/plugins/action/pub_posture_cmd.hpp"

namespace pb2025_sentry_behavior
{

PublishPostureCmdAction::PublishPostureCmdAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishPostureCmdAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<unsigned>("posture", pb_rm_interfaces::msg::PostureCmd::MOVE, "Posture mode"),
    BT::InputPort<unsigned>(
      "reason", pb_rm_interfaces::msg::PostureCmd::REASON_DEFAULT, "Posture switch reason"),
    BT::InputPort<bool>("force_switch", false, "Force posture switch"),
  });
}

bool PublishPostureCmdAction::setMessage(pb_rm_interfaces::msg::PostureCmd & msg)
{
  unsigned posture = pb_rm_interfaces::msg::PostureCmd::MOVE;
  unsigned reason = pb_rm_interfaces::msg::PostureCmd::REASON_DEFAULT;
  bool force_switch = false;

  getInput("posture", posture);
  getInput("reason", reason);
  getInput("force_switch", force_switch);

  msg.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  msg.posture = static_cast<uint8_t>(posture);
  msg.reason = static_cast<uint8_t>(reason);
  msg.force_switch = force_switch;

  if (
    msg.posture < pb_rm_interfaces::msg::PostureCmd::OFF ||
    msg.posture > pb_rm_interfaces::msg::PostureCmd::SPIN)
  {
    msg.posture = pb_rm_interfaces::msg::PostureCmd::MOVE;
    msg.reason = pb_rm_interfaces::msg::PostureCmd::REASON_DEFAULT;
  }

  return true;
}

bool PublishPostureCmdAction::setHaltMessage(pb_rm_interfaces::msg::PostureCmd & msg)
{
  msg.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  msg.posture = pb_rm_interfaces::msg::PostureCmd::OFF;
  msg.reason = pb_rm_interfaces::msg::PostureCmd::REASON_DEFAULT;
  msg.force_switch = false;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishPostureCmdAction, "PublishPostureCmd");
