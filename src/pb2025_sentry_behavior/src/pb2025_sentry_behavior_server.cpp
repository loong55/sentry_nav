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

/*行为树服务端作用：
1.订阅并整合所有外部系统数据（裁判系统、自瞄系统、导航全局代价地图）至黑板中
2.通过继承TreeExecutionServer类，接收客户端发送的树名，管理行为树生命周期：创建、执行、完成；
  行为树会自行调用行为树插件节点（插件是热插拔的，用工厂注册）
3.支持Groot2可视化调试，自动生成节点模型XML文件
*/

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <utility>

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

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{

namespace
{

geometry_msgs::msg::PoseStamped load_pose_stamped_prefix(
  const rclcpp::Node::SharedPtr & node, const std::string & prefix)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = node->get_parameter(prefix + ".frame_id").as_string();
  pose.pose.position.x = node->get_parameter(prefix + ".position.x").as_double();
  pose.pose.position.y = node->get_parameter(prefix + ".position.y").as_double();
  pose.pose.position.z = node->get_parameter(prefix + ".position.z").as_double();

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  const std::string ori_prefix = prefix + ".orientation";
  if (node->has_parameter(ori_prefix + ".x")) {
    pose.pose.orientation.x = node->get_parameter(ori_prefix + ".x").as_double();
  }
  if (node->has_parameter(ori_prefix + ".y")) {
    pose.pose.orientation.y = node->get_parameter(ori_prefix + ".y").as_double();
  }
  if (node->has_parameter(ori_prefix + ".z")) {
    pose.pose.orientation.z = node->get_parameter(ori_prefix + ".z").as_double();
  }
  if (node->has_parameter(ori_prefix + ".w")) {
    pose.pose.orientation.w = node->get_parameter(ori_prefix + ".w").as_double();
  }
  return pose;
}

void load_nav_pose_parameters_into_blackboard(
  const rclcpp::Node::SharedPtr & node, const BT::Blackboard::Ptr & blackboard)
{
  static const std::pair<const char *, const char *> k_bindings[] = {
    {"supply", "supply"},
    {"fort", "fort"},
    {"highway_in", "highway_in"},
    {"highway_out", "highway_out"},
    {"outpost", "outpost"},
  };

  for (const auto & [bb_key, param_prefix] : k_bindings) {
    const std::string frame_param = std::string(param_prefix) + ".frame_id";
    if (!node->has_parameter(frame_param)) {
      continue;
    }
    try {
      blackboard->set(bb_key, load_pose_stamped_prefix(node, param_prefix));
      RCLCPP_INFO(
        node->get_logger(), "Loaded pose blackboard key [%s] from params [%s]", bb_key,
        param_prefix);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        node->get_logger(), "Skip pose [%s] (%s): %s", bb_key, param_prefix, e.what());
    }
  }
}

}  // namespace

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
  globalBlackboard()->set("node", node());

  load_nav_pose_parameters_into_blackboard(node(), globalBlackboard());

  // Params YAML + automatically_declare_parameters_from_overrides(true) may already
  // declare use_cout_logger; avoid ParameterAlreadyDeclaredException.
  if (!node()->has_parameter("use_cout_logger")) {
    node()->declare_parameter("use_cout_logger", false);
  }
  node()->get_parameter("use_cout_logger", use_cout_logger_);

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

/**********************************行为树生命周期管理************************************/

//接收目标前调用（在树创建前），行为树服务器目标接收函数，接收到目标返回true
bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)//payload是目标坐标点信息
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

//树创建后调用，调试阶段，监控行为树的创建，use_cout_logger_默认为false，不在控制台监控树的创建
//可以通过参数文件 sentry_behavior.yaml 进行配置：use_cout_logger: true
void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

//每次tick后调用，optional表示一个可能包含也可能不包含值的类型
std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;//返回一个空的optional值
}

//行为树执行完成调用，status表示行为树执行状态，was_cancelled表示行为树是否被取消
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

/**********************************行为树生命周期管理************************************/

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
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
