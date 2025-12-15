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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_client.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace pb2025_sentry_behavior
{

//创建客户端ros2节点
//class SentryBehaviorClient : public rclcpp::Node，并初始化节点名称与节点配置
SentryBehaviorClient::SentryBehaviorClient(const rclcpp::NodeOptions & options)
: Node("sentry_behavior_client", options)
{
  // 1. 声明并获取参数 参数名为"target_tree" 默认值为"test_attacked_feedback"
  declare_parameter<std::string>("target_tree", "test_attacked_feedback");
  get_parameter("target_tree", target_tree_);//从sentry_behavior.yaml获取参数，并赋值到target_tree_

  // 2. 创建了一个名为action_client_的动作客户端，用于与名为"pb2025_sentry_behavior"的动作服务器进行通信。
  // 处理BTExecuteTree类型的动作，包含目标、结果和反馈等结构，此动作类型与服务端相匹配
  action_client_ = rclcpp_action::create_client<BTExecuteTree>(this, "pb2025_sentry_behavior");

  // 3. 等待动作服务器启动，wait_for_action_server() 会等待一小段时间（默认为几秒）来检查服务器，不可以返回false
  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(get_logger(), "Action server not available!");
    return;
  }

  // 4. 创建一个定时器，每100ms调用一次sendGoal函数
  timer_ = create_wall_timer(100ms, std::bind(&SentryBehaviorClient::sendGoal, this));
}

//给服务器发送目标点
void SentryBehaviorClient::sendGoal()
{
  using namespace std::placeholders; // for _1, _2, _3 below
  timer_->cancel(); // 立即取消定时器，避免重复发送

  auto goal_msg = BTExecuteTree::Goal();//BTExecuteTree类型中有目标，结果和反馈等结构
  goal_msg.target_tree = target_tree_;

  RCLCPP_INFO(get_logger(), "Sending goal to execute Behavior Tree: %s", target_tree_.c_str());

  // 动作客户端的发送目标选项对象
  auto options = rclcpp_action::Client<BTExecuteTree>::SendGoalOptions();
  //设置反馈回调函数，绑定成员函数和对象实例，_1和_2是占位符表示目标和反馈消息
  options.feedback_callback = std::bind(&SentryBehaviorClient::feedbackCallback, this, _1, _2);
  //设置结果回调函数，绑定成员函数和对象实例，_1是目标结果消息（包含动作执行状态和实际结果数据）
  options.result_callback = std::bind(&SentryBehaviorClient::resultCallback, this, _1);

  //异步发送动作目标，不会阻塞当前线程
  action_client_->async_send_goal(goal_msg, options);
}

//处理服务器返回的结果
void SentryBehaviorClient::resultCallback(
  const rclcpp_action::ClientGoalHandle<BTExecuteTree>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal succeeded: %s", result.result->return_message.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Goal was canceled.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal failed: %s", result.result->return_message.c_str());
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      break;
  }
  timer_->reset(); //重置定时器
}

//处理行为树服务器返回的反馈消息
void SentryBehaviorClient::feedbackCallback(
  rclcpp_action::ClientGoalHandle<BTExecuteTree>::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const BTExecuteTree::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Received feedback: %s", feedback->message.c_str());
}

}  // namespace pb2025_sentry_behavior

//pb2025_sentry_behavior命名空间下的SentryBehaviorClient类注册为一个ROS2组件节点。支持节点的动态加载和卸载
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pb2025_sentry_behavior::SentryBehaviorClient)
