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

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

// 包含C++17标准库中的filesystem头文件，用于处理文件系统相关的操作
#include <filesystem>
// 包含文件流输入输出头文件，用于文件读写操作
#include <fstream>

// 包含装甲板相关消息类型定义（自瞄包）
#include "auto_aim_interfaces/msg/armors.hpp"
// 包含目标相关消息类型定义（自瞄包）
#include "auto_aim_interfaces/msg/target.hpp"
// 包含BehaviorTree XML解析相关功能
#include "behaviortree_cpp/xml_parsing.h"
// 包含占用栅格地图相关消息类型定义
#include "nav_msgs/msg/occupancy_grid.hpp"
// 包含裁判系统相关消息类型定义
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
namespace pb2025_sentry_behavior
{
//订阅器；参数：话题名称，黑板键名，服务质量
template <typename T>
void SentryBehaviorServer::subscribe(
  //(Quality of Service)参数配置,影响消息的传输质量和可靠性
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  // 创建订阅器，订阅指定话题的消息，并将消息存储到全局黑板中
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);//根据qos,最多储存10条消息
}

//服务器订阅消息
SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)//TreeExecutionServer是一个行为树执行服务器的基类，而options是ROS2的节点配置参数。
{
  node()->declare_parameter("use_cout_logger", false);//声明一个参数，用于配置是否使用cout日志记录器
  node()->get_parameter("use_cout_logger", use_cout_logger_);//获取参数的值

  //订阅裁判系统信息
  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  subscribe<pb_rm_interfaces::msg::GameStatus>("referee/game_status", "referee_gameStatus");
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");

  auto detector_qos = rclcpp::SensorDataQoS();//创建一个高频传感器数据质量服务对象
  subscribe<auto_aim_interfaces::msg::Armors>("detector/armors", "detector_armors", detector_qos);//装甲板
  auto tracker_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Target>("tracker/target", "tracker_target", tracker_qos);//目标

  //订阅全局地图，KeepLast(1)表示只保留最新的1个消息，transient_local()表示消息只在本地保存，reliable()表示消息可靠传输
  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);
}

//行为树生命周期管理，行为树服务器目标接收函数，接收到目标返回true
bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)//payload是目标坐标点信息
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

//行为树生命周期管理，调试阶段，监控行为树的创建，use_cout_logger_默认为false，不在控制台监控树的创建
//可以通过参数文件 sentry_behavior.yaml 进行配置：use_cout_logger: true
void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

//行为树执行周期数，optional表示一个可能包含也可能不包含值的类型
std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;//返回一个空的optional值
}

//行为树执行完成，status表示行为树执行状态，was_cancelled表示行为树是否被取消
//返回结果字符串
std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  //[INFO] [1699123456.123456789] [pb2025_sentry_behavior_server]: Starting SentryBehaviorServer
  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  //多线程执行器，参数：执行器的配置选项（默认），单核cpu并发线程数，是否使用定时器，定时器间隔250ms
  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  //不是固定3个线程，而是所有回调函数根据硬件自动调整，动态线程池
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

  // Save the XML models to a file
  std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
  file << xml_models;

  rclcpp::shutdown();
}
