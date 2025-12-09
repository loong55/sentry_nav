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

#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

using msec = std::chrono::milliseconds;

namespace BT
{

template <>
inline std::chrono::milliseconds convertFromString(StringView str)
{
  return std::chrono::milliseconds(std::stoll(std::string(str)));
}

/**
 * @brief Abstract class to wrap a ROS publisher as a StatefulActionNode
 *
 * This class allows publishing a ROS message for a specified duration, returning RUNNING
 * during the duration and SUCCESS once elapsed. Supports dynamic topic name changes and
 * halt messages.
 */
template <class TopicT>
class RosTopicPubStatefulActionNode : public BT::StatefulActionNode
{
public:
  // Type definitions
  using Publisher = typename rclcpp::Publisher<TopicT>;

  /**
   * @brief Constructor for RosTopicPubStatefulActionNode
   * @param instance_name Name of the BT node instance
   * @param conf BT node configuration
   * @param params ROS node parameters including the node handle
   */
  explicit RosTopicPubStatefulActionNode(const std::string& instance_name,
                                         const BT::NodeConfig& conf,
                                         const RosNodeParams& params);

  virtual ~RosTopicPubStatefulActionNode() = default;

  /**
   * @brief Provides the list of ports with added duration port
   * @param addition Additional ports to add
   * @return PortsList containing basic ports and duration port
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name"),
      InputPort<msec>("duration", 0, "Publish then sleep duration in milliseconds")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /**
   * @brief Called when transitioning from IDLE to RUNNING
   * @return NodeStatus SUCCESS if setup succeeded, FAILURE otherwise
   */
  virtual BT::NodeStatus onStart() override;

  /**
   * @brief Called periodically while in RUNNING state
   * @return NodeStatus RUNNING while publishing, SUCCESS when duration elapsed
   */
  virtual BT::NodeStatus onRunning() override;

  /**
   * @brief Called when the node is halted
   */
  virtual void onHalted() override;

  /**
   * @brief Pure virtual method to set the message to publish
   * @param msg Output message to be populated
   * @return true if message setup succeeded, false otherwise
   */
  virtual bool setMessage(TopicT& msg) = 0;

  /**
   * @brief Virtual method to set halt message (override if needed)
   * @param msg Output message to be populated
   * @return true if halt message setup succeeded, false otherwise
   */
  virtual bool setHaltMessage(TopicT& msg)
  {
    return false;
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::string prev_topic_name_;
  bool topic_name_may_change_ = false;

private:
  std::shared_ptr<Publisher> publisher_;
  msec duration_;
  rclcpp::Time start_time_;

  bool createPublisher(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosTopicPubStatefulActionNode<T>::RosTopicPubStatefulActionNode(
    const std::string& instance_name, const NodeConfig& conf, const RosNodeParams& params)
  : BT::StatefulActionNode(instance_name, conf), node_(params.nh)
{
  // Topic name initialization logic
  auto portIt = config().input_ports.find("topic_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if(bb_topic_name.empty() || bb_topic_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty())
      {
        throw std::logic_error("Both [topic_name] in InputPort and RosNodeParams are "
                               "empty");
      }
      else
      {
        createPublisher(params.default_port_value);
      }
    }
    else if(!isBlackboardPointer(bb_topic_name))
    {
      createPublisher(bb_topic_name);
    }
    else
    {
      topic_name_may_change_ = true;
    }
  }
  else
  {
    if(params.default_port_value.empty())
    {
      throw std::logic_error("Both [topic_name] in InputPort and RosNodeParams are "
                             "empty");
    }
    else
    {
      createPublisher(params.default_port_value);
    }
  }
}

template <class T>
inline bool
RosTopicPubStatefulActionNode<T>::createPublisher(const std::string& topic_name)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  publisher_ = node_->create_publisher<T>(topic_name, 1);
  prev_topic_name_ = topic_name;
  return true;
}

template <class T>
inline NodeStatus RosTopicPubStatefulActionNode<T>::onStart()
{
  // Check for topic name change
  if(topic_name_may_change_ || !publisher_)
  {
    std::string topic_name;
    if(!getInput("topic_name", topic_name) || topic_name.empty())
    {
      throw RuntimeError("Failed to get topic_name");
    }
    if(prev_topic_name_ != topic_name)
    {
      createPublisher(topic_name);
    }
  }

  // Get duration parameter
  if(!getInput("duration", duration_))
  {
    duration_ = msec(0);
  }

  // Create and publish message
  T msg;
  if(!setMessage(msg))
  {
    return NodeStatus::FAILURE;
  }
  publisher_->publish(msg);

  // Record start time
  start_time_ = node_->now();
  return NodeStatus::RUNNING;
}

template <class T>
inline NodeStatus RosTopicPubStatefulActionNode<T>::onRunning()
{
  auto elapsed = node_->now() - start_time_;
  auto elapsed_ms = elapsed.to_chrono<msec>();

  return (elapsed_ms < duration_) ? NodeStatus::RUNNING : NodeStatus::SUCCESS;
}

template <class T>
inline void RosTopicPubStatefulActionNode<T>::onHalted()
{
  T halt_msg;
  if(setHaltMessage(halt_msg))
  {
    publisher_->publish(halt_msg);
  }
}

}  // namespace BT