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

#ifndef PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_CLIENT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_CLIENT_HPP_

#include <memory>
#include <string>

#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"

namespace pb2025_sentry_behavior
{

/**
 * @brief 哨兵行为树客户端类
 * 
 * 该类继承自rclcpp::Node，用于与行为树服务器通信，
 * 发送行为树执行请求并处理执行结果和反馈。
 */
class SentryBehaviorClient : public rclcpp::Node
{
public:
  // 类型别名定义
  using BTExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;  // 行为树执行action类型
  using GoalHandleBTExecuateTree = rclcpp_action::ClientGoalHandle<BTExecuteTree>;  // action目标句柄类型

  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   */
  explicit SentryBehaviorClient(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 发送行为树执行目标
   * 
   * 从参数服务器读取目标行为树名称，
   * 构造并发送执行请求到行为树服务器
   */
  void sendGoal();

  /**
   * @brief 处理行为树执行结果回调
   * @param result 执行结果包装器，包含最终执行状态和返回消息
   */
  void resultCallback(const GoalHandleBTExecuateTree::WrappedResult & result);

  /**
   * @brief 处理行为树执行反馈回调
   * @param goal_handle 目标句柄
   * @param feedback 执行反馈信息
   */
  void feedbackCallback(
    GoalHandleBTExecuateTree::SharedPtr goal_handle,
    const std::shared_ptr<const BTExecuteTree::Feedback> feedback);

  rclcpp::TimerBase::SharedPtr timer_;  // 定时器，用于周期性发送目标
  rclcpp_action::Client<BTExecuteTree>::SharedPtr action_client_;  // action客户端
  std::string target_tree_;  // 要执行的目标行为树名称
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_CLIENT_HPP_
