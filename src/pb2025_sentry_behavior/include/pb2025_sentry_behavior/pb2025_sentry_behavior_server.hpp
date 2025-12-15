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

#ifndef PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_SERVER_HPP_
#define PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_SERVER_HPP_

// 引入 BehaviorTree.cpp 库中的标准输出日志记录器头文件
// 这个日志记录器会将树的行为和状态输出到标准输出流（通常是控制台）
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <memory>  // 包含C++标准库中的内存管理头文件，提供了智能指针等内存管理工具
#include <string>
#include <vector>

// 引入行为树ROS2执行服务器的头文件
// 该头文件包含了BehaviorTree::TreeExecutionServer类的定义
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SentryBehaviorServer : public BT::TreeExecutionServer
{
public:
  explicit SentryBehaviorServer(const rclcpp::NodeOptions & options);//explicit 关键字用于防止隐式转换

  /**
   * @brief 接收到目标时、在创建行为树之前调用的回调函数。
   * 如果返回false，目标将被拒绝。
  */
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override;

  /**
   * @brief 在创建行为树后调用的回调函数。
   * 可用于初始化日志记录器或全局黑板等。
   *
   * @param tree 创建的行为树实例
  */
  void onTreeCreated(BT::Tree & tree) override;

  /**
   * @brief 在每个循环中、tree.tickOnce()之后调用的函数。
   * 如果返回有效的NodeStatus，行为树将停止并返回该状态。
   * 返回std::nullopt以继续执行。
   *
   * @param status 上一次tick后行为树的状态
  */
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

  /**
   * @brief 行为树执行完成后调用的回调函数，
   * 即当行为树返回SUCCESS/FAILURE或被Action Client取消时调用。
   *
   * @param status 最后一次tick后行为树的状态
   * @param was_cancelled 如果被Action Client取消则为true
   *
   * @return 如果不是std::nullopt，该字符串将作为[return_message]发送给Action Client。
  */
  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override;

private:
  template <typename T>
  void subscribe(
    const std::string & topic, const std::string & bb_key,
    // 定义一个ROS 2 Quality of Service (QoS)配置的常量引用
    // 该配置设置为10的历史深度，即保留最近10条消息
    const rclcpp::QoS & qos = rclcpp::QoS(10));

  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;  // 订阅器列表
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;  // 标准输出日志记录器
  uint32_t tick_count_;  // 行为树tick计数器
  bool use_cout_logger_;  // 是否使用标准输出日志记录器
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_SERVER_HPP_
