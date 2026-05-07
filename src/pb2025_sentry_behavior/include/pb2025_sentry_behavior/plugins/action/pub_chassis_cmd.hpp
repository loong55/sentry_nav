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

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_CHASSIS_CMD_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_CHASSIS_CMD_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "pb_rm_interfaces/msg/chassis_cmd.hpp"

namespace pb2025_sentry_behavior
{

class PublishChassisCmdAction
: public BT::RosTopicPubStatefulActionNode<pb_rm_interfaces::msg::ChassisCmd>
{
public:
  PublishChassisCmdAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(pb_rm_interfaces::msg::ChassisCmd & msg) override;

  bool setHaltMessage(pb_rm_interfaces::msg::ChassisCmd & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_CHASSIS_CMD_HPP_