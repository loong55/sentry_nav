#pragma once
#include "include.h"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"

/**
 * @brief 通过话题发布Nav2目标点的行为树节点
 *
 * 该节点通过发布 geometry_msgs::msg::PoseStamped 消息到指定话题来设置导航目标。
 * 默认话题为 /goal_pose，这是 Nav2 的标准目标点输入话题。
 */
class PubNav2Goal : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped> {
public:
    // 构造函数
    PubNav2Goal(
        const std::string &name,
        const BT::NodeConfiguration &conf,
        const BT::RosNodeParams &params)
        : RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
    {
    }

    // 定义节点需要的输入端口
    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置"),
            BT::InputPort<std::string>("frame_id", "map", "坐标系ID，默认为map"),
            BT::InputPort<unsigned>("min_pub_interval_ms")

        });
    }
    // 设置要发布的消息
    bool setMessage(geometry_msgs::msg::PoseStamped &msg) override {
        // 从输入端口获取目标位置
        auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
        if (!res) {
            RCLCPP_ERROR(node_->get_logger(), "读取端口[goal_pose]时出错: %s", res.error().c_str());
            return false;
        }

        msg = res.value();

        // 设置frame_id（如果未设置则使用输入端口的值）
        if (msg.header.frame_id.empty()) {
            auto frame_res = getInput<std::string>("frame_id");
            msg.header.frame_id = (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";
        }

        // 设置时间戳为当前时间
        msg.header.stamp = rclcpp::Clock().now();

        // 输出调试信息
        RCLCPP_INFO(node_->get_logger(),
            "PubNav2Goal: 发布目标点 [%.2f, %.2f, %.2f] frame: %s",
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.header.frame_id.c_str());

        return true;
    }
};

class SendNav2Goal : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {
public:
    // 构造函数：初始化节点名称、配置和ROS2参数
    SendNav2Goal(
        const std::string &name,
        const BT::NodeConfiguration &conf,
        const BT::RosNodeParams &params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) // 调用基类构造函数
    {
        // 构造函数体，可以在这里进行额外的初始化
    }

    // 定义节点需要的输入端口
    static BT::PortsList providedPorts() {
        return {
            // 输入端口：目标位置，类型为geometry_msgs::msg::PoseStamped
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "导航目标位置")
        };
    }

    // 设置导航目标：从输入端口获取目标位置并设置到action goal中
    bool setGoal(Goal &goal) override {
        // 从输入端口"goal_pose"获取目标位置
        auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
        if (!res) {
            // 如果获取失败，抛出运行时错误
            throw BT::RuntimeError("读取端口[goal_pose]时出错:", res.error());
        }

        // 将获取到的位姿赋值给goal的pose字段
        goal.pose = *res;
        // 设置时间戳为当前时间
        goal.pose.header.stamp = rclcpp::Clock().now();

        // 格式化输出目标位置信息（调试用）
        // clang-format off
        std::cout << "Goal_pose: [ "
            << std::fixed << std::setprecision(1)  // 设置输出精度
            << goal.pose.pose.position.x << ", "   // 输出x坐标
            << goal.pose.pose.position.y << ", "   // 输出y坐标
            << goal.pose.pose.position.z << ", " << " ]\n";  // 输出z坐标
        // clang-format on

        // 返回true表示成功设置目标
        return true;
    }

    // 当节点被中断时调用
     void onHalt() override {
        RCLCPP_INFO(logger(), "SendGoalAction has been halted.");

        // 行为树库的 RosActionNode 在 onHalt 中通常会尝试取消 Goal。
        // 如果 Goal 句柄已失效（例如服务器重启或超时），这会抛出 UnknownGoalHandleError。
        // 我们必须捕获这个异常以防止节点崩溃。

        try {
            // 调用基类的 onHalt 执行标准的取消流程
            // 注意：在某些 behavior_tree_ros2 版本中，基类方法名为 onHalt
            BT::RosActionNode<nav2_msgs::action::NavigateToPose>::onHalt();
        }
        catch (const rclcpp_action::exceptions::UnknownGoalHandleError & e) {
            RCLCPP_WARN(logger(), "Ignored UnknownGoalHandleError during halt: %s", e.what());
            // 忽略错误，继续执行清理逻辑
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(logger(), "Exception caught during halt: %s", e.what());
        }

        // 如果需要额外的自定义清理逻辑，可以在这里添加
        // 例如：重置某些局部标志位
    }

    // 当收到action结果时调用
    BT::NodeStatus onResultReceived(const WrappedResult &wr) override {
        // 根据action返回的结果码进行不同的处理
        switch (wr.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                // 导航成功完成
                RCLCPP_INFO(logger(), "Success!!!");
                return BT::NodeStatus::SUCCESS; // 返回成功状态

            case rclcpp_action::ResultCode::ABORTED:
                // 导航被中止（通常是由于严重错误）
                RCLCPP_INFO(logger(), "Goal was aborted");
                return BT::NodeStatus::FAILURE; // 返回失败状态

            case rclcpp_action::ResultCode::CANCELED:
                // 导航被取消（通常是被用户或系统取消）
                RCLCPP_INFO(logger(), "Goal was canceled");
                std::cout << "Goal was canceled" << '\n';
                return BT::NodeStatus::FAILURE; // 返回失败状态

            default:
                // 未知的结果码
                RCLCPP_INFO(logger(), "Unknown result code");
                return BT::NodeStatus::FAILURE; // 返回失败状态
        }
    }

    // 当收到action反馈时调用
    BT::NodeStatus onFeedback(
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) override {
        // 这里可以处理反馈信息，例如显示剩余距离
        std::cout << "Distance remaining: " << feedback->distance_remaining << '\n';

        // 可选：可以添加更多的反馈处理逻辑
        // 例如检查进度、更新UI等

        // 返回RUNNING状态表示动作仍在进行中
        return BT::NodeStatus::RUNNING;
    }

    // 当action失败时调用（例如连接失败、超时等）
    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
        // 记录错误信息，包括错误码
        RCLCPP_ERROR(logger(), "SendGoalAction failed with error code: %d", error);

        // 返回失败状态
        return BT::NodeStatus::FAILURE;
    }
};

// 多点导航行为树节点：从 CSV 文件加载航点，通过 Nav2 的 follow_waypoints action 下发
class FollowWaypointsAction : public BT::RosActionNode<nav2_msgs::action::FollowWaypoints> {
public:
    FollowWaypointsAction(
        const std::string &name,
        const BT::NodeConfiguration &conf,
        const BT::RosNodeParams &params)
        : RosActionNode<nav2_msgs::action::FollowWaypoints>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<std::string>("waypoint_file", "航点 CSV 文件的绝对路径"),
            BT::InputPort<std::string>("frame_id", "map", "坐标系 ID，默认为 map"),
            BT::OutputPort<int>("current_waypoint", "当前正在前往的航点索引"),
        });
    }

    bool setGoal(Goal &goal) override {
        auto file_res = getInput<std::string>("waypoint_file");
        if (!file_res) {
            RCLCPP_ERROR(logger(), "读取端口 [waypoint_file] 时出错: %s", file_res.error().c_str());
            return false;
        }
        const std::string &waypoint_file = file_res.value();

        auto frame_res = getInput<std::string>("frame_id");
        const std::string &frame_id = (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";

        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (!loadWaypointsFromCSV(waypoint_file, waypoints, frame_id)) {
            RCLCPP_ERROR(logger(), "无法从文件加载航点: %s", waypoint_file.c_str());
            return false;
        }

        if (waypoints.empty()) {
            RCLCPP_ERROR(logger(), "FollowWaypointsAction: 航点列表为空，取消发送 goal");
            return false;
        }

        // Ensure header frame_id and stamp for each waypoint
        rclcpp::Time now = rclcpp::Clock().now();
        for (auto &wp : waypoints) {
            if (wp.header.frame_id.empty()) wp.header.frame_id = frame_id;
            wp.header.stamp = now;
        }

        goal.poses = waypoints;
        RCLCPP_INFO(logger(), "FollowWaypointsAction: 已加载 %lu 个航点，准备下发", static_cast<unsigned long>(waypoints.size()));
        for (size_t i = 0; i < waypoints.size(); ++i) {
            RCLCPP_INFO(logger(), "  航点[%lu]: [%.2f, %.2f]",
                        static_cast<unsigned long>(i), waypoints[i].pose.position.x, waypoints[i].pose.position.y);
        }
        return true;
    }

    void onHalt() override {
        RCLCPP_WARN(logger(), "FollowWaypointsAction: 多点导航已被中断");
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override {
        RCLCPP_INFO(logger(), "FollowWaypointsAction: 所有航点导航完成!");
        if (wr.result && !wr.result->missed_waypoints.empty()) {
            {
                std::ostringstream oss;
                oss << "  有 " << wr.result->missed_waypoints.size() << " 个航点未到达:";
                std::string msg = oss.str();
                RCLCPP_WARN(logger(), "%s", msg.c_str());
            }
            /*for (const auto &wp : wr.result->missed_waypoints) {
                std::ostringstream oss_idx;
                oss_idx << "    未到达航点索引: " << wp.index;
                std::string msg_idx = oss_idx.str();
                RCLCPP_WARN(logger(), "%s", msg_idx.c_str());
            }*/
             // 根据需求，将未到达视为失败
             return BT::NodeStatus::FAILURE;
         }

        RCLCPP_INFO(logger(), "  所有航点均已成功到达");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFeedback(
        const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) override {
        // 将反馈日志降为 DEBUG，避免频繁 INFO 污染日志
        RCLCPP_DEBUG(logger(), "FollowWaypointsAction: 正在前往航点 %u", static_cast<unsigned>(feedback->current_waypoint));
        // 将当前航点索引写回黑板
        std::cout<<"set......................................."<<std::endl;
        setOutput("current_waypoint", static_cast<int>(feedback->current_waypoint));
        std::cout<<"ste111111111111111........................"<<std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
        RCLCPP_ERROR_STREAM(logger(), "FollowWaypointsAction 失败，错误码: " << BT::toStr(error));
        return BT::NodeStatus::FAILURE;
    }
};

// 穿越导航行为树节点：从 CSV 加载航点，通过 Nav2 的 navigate_through_poses action
// 规划一条平滑路径穿越所有航点，不在中间航点停留，适合巡逻/快速穿越场景
class NavigateThroughPosesAction : public BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses> {
public:
    NavigateThroughPosesAction(
        const std::string &name,
        const BT::NodeConfiguration &conf,
        const BT::RosNodeParams &params)
        : RosActionNode<nav2_msgs::action::NavigateThroughPoses>(name, conf, params),
          total_waypoints_(0)
    {
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<std::string>("waypoint_file", "航点 CSV 文件的绝对路径"),
            BT::InputPort<std::string>("frame_id", "map", "坐标系 ID，默认为 map"),
            BT::OutputPort<int>("current_waypoint", "当前正在前往的航点索引"),
        });
    }

    bool setGoal(Goal &goal) override {
        auto file_res = getInput<std::string>("waypoint_file");
        if (!file_res) {
            RCLCPP_ERROR(logger(), "读取端口 [waypoint_file] 时出错: %s", file_res.error().c_str());
            return false;
        }
        const std::string &waypoint_file = file_res.value();

        auto frame_res = getInput<std::string>("frame_id");
        const std::string &frame_id = (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";

        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (!loadWaypointsFromCSV(waypoint_file, waypoints, frame_id)) {
            RCLCPP_ERROR(logger(), "无法从文件加载航点: %s", waypoint_file.c_str());
            return false;
        }

        if (waypoints.empty()) {
            RCLCPP_ERROR(logger(), "NavigateThroughPosesAction: 航点列表为空，取消发送 goal");
            return false;
        }

        // 设置每个航点的 frame_id 和时间戳
        rclcpp::Time now = rclcpp::Clock().now();
        for (auto &wp : waypoints) {
            if (wp.header.frame_id.empty()) wp.header.frame_id = frame_id;
            wp.header.stamp = now;
        }

        total_waypoints_ = static_cast<int>(waypoints.size());
        goal.poses = waypoints;

        RCLCPP_INFO(logger(), "NavigateThroughPosesAction: 已加载 %d 个航点，准备穿越导航", total_waypoints_);
        for (size_t i = 0; i < waypoints.size(); ++i) {
            RCLCPP_INFO(logger(), "  航点[%zu]: [%.2f, %.2f]",
                        i, waypoints[i].pose.position.x, waypoints[i].pose.position.y);
        }
        return true;
    }

    void onHalt() override {
        RCLCPP_WARN(logger(), "NavigateThroughPosesAction: 穿越导航已被中断");
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override {
        RCLCPP_INFO(logger(), "NavigateThroughPosesAction: 穿越导航完成!");
        if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_WARN(logger(), "  穿越导航未成功，结果码: %d", static_cast<int>(wr.code));
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(logger(), "  所有航点均已成功穿越");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFeedback(
        const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback) override {
        int remaining = feedback->number_of_poses_remaining;
        int current = total_waypoints_ - remaining;
        RCLCPP_DEBUG(logger(), "NavigateThroughPosesAction: 正在前往航点 %d/%d",
                     current, total_waypoints_);
        setOutput("current_waypoint", current);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
        RCLCPP_ERROR_STREAM(logger(), "NavigateThroughPosesAction 失败，错误码: " << BT::toStr(error));
        return BT::NodeStatus::FAILURE;
    }

private:
    int total_waypoints_;
};

//通过接收的消息将有用的消息写入黑板实现共享
class WriteToBlackboard : public BT::SyncActionNode {
public:
    WriteToBlackboard(const std::string &name,
                      const BT::NodeConfiguration &config,
                      const std::shared_ptr<rclcpp::Node> &global_node)
        : BT::SyncActionNode(name, config),
          global_node_(global_node) {
        global_node_ = rclcpp::Node::make_shared("WriteToBlackboard");
        sub_ = global_node_->create_subscription<rm_interfaces::msg::SerialReceiveData>(
            "/SerialReceiveData", 10,
            std::bind(&WriteToBlackboard::callback, this, std::placeholders::_1));
        is_ReadInterface_ = false;
    }


    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<float>("Hp"),
            BT::OutputPort<bool>("Zone_status"),
            BT::OutputPort<bool>("Is_defence"),
            BT::OutputPort<bool>("Is_attack"),
            BT::OutputPort<bool>("Self_status"),
            BT::OutputPort<bool>("Is_recover"),
        };
    }

    //设置参数
    float hp = 400;
    bool zone_status = false;
    bool is_defence = false;
    bool is_attack = false;
	bool self_status = false;
	bool is_recover = false;

    BT::NodeStatus tick() override {
        // 处理回调
        rclcpp::spin_some(global_node_); //只处理当前队列中的回s后就返回
        if (!is_ReadInterface_)
            return BT::NodeStatus::FAILURE;

        //写入黑板
        setOutput("Hp", hp);
        setOutput("Zone_status", zone_status);
        setOutput("Is_defence", is_defence);
        setOutput("Is_attack", is_attack);
        setOutput("Self_status", self_status);
        setOutput("Is_recover", is_recover);

        return BT::NodeStatus::SUCCESS;
    }

    void callback(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg) {
        hp = msg->judge_system_data.hp;
        zone_status = msg->judge_system_data.zone_status;
        is_defence = msg->judge_system_data.is_defence;
        is_attack = msg->judge_system_data.is_attack;
        self_status = msg->judge_system_data.self_status;
        is_recover = msg->judge_system_data.is_recover;

        is_ReadInterface_ = true;
        RCLCPP_INFO(global_node_->get_logger(), "Callback hp = %f", hp);
    }

private:
    std::shared_ptr<rclcpp::Node> global_node_;
    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr sub_;
    bool is_ReadInterface_;
};

// ======================== PubNav2Goal 巡逻方案节点 ========================

// 从 CSV 加载航点列表到黑板
class LoadWaypoints : public BT::SyncActionNode {
public:
    LoadWaypoints(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("waypoint_file", "航点 CSV 文件的绝对路径"),
            BT::InputPort<std::string>("frame_id", "map", "坐标系 ID"),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "航点列表"),
            BT::OutputPort<std::vector<double>>("wait_times", "每个航点的等待时间 (秒)"),
            BT::OutputPort<int>("total_waypoints", "航点总数"),
        };
    }

    BT::NodeStatus tick() override {
        auto file_res = getInput<std::string>("waypoint_file");
        if (!file_res) {
            RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"),
                         "读取端口 [waypoint_file] 时出错: %s", file_res.error().c_str());
            return BT::NodeStatus::FAILURE;
        }

        auto frame_res = getInput<std::string>("frame_id");
        const std::string frame_id = (frame_res && !frame_res.value().empty()) ? frame_res.value() : "map";

        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::vector<double> wait_times;
        if (!loadWaypointsFromCSV(file_res.value(), waypoints, frame_id, &wait_times)) {
            RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"),
                         "无法从文件加载航点: %s", file_res.value().c_str());
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("LoadWaypoints"),
                    "已加载 %lu 个航点", static_cast<unsigned long>(waypoints.size()));

        setOutput("waypoints", waypoints);
        setOutput("wait_times", wait_times);
        setOutput("total_waypoints", static_cast<int>(waypoints.size()));
        return BT::NodeStatus::SUCCESS;
    }
};

// 根据 wp_idx 从航点列表中取出当前目标点
class GetCurrentWaypoint : public BT::SyncActionNode {
public:
    GetCurrentWaypoint(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "航点列表"),
            BT::InputPort<std::vector<double>>("wait_times", "每个航点的等待时间列表"),
            BT::InputPort<int>("wp_idx", "当前航点索引"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_goal", "当前目标点"),
            BT::OutputPort<double>("current_wait_sec", "当前航点的等待时间"),
        };
    }

    BT::NodeStatus tick() override {
        auto wp_res = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
        auto wt_res = getInput<std::vector<double>>("wait_times");
        auto idx_res = getInput<int>("wp_idx");

        if (!wp_res || !idx_res) {
            RCLCPP_ERROR(rclcpp::get_logger("GetCurrentWaypoint"), "读取端口失败");
            return BT::NodeStatus::FAILURE;
        }

        const auto &waypoints = wp_res.value();
        int idx = idx_res.value();

        if (waypoints.empty() || idx < 0 || idx >= static_cast<int>(waypoints.size())) {
            RCLCPP_ERROR(rclcpp::get_logger("GetCurrentWaypoint"),
                         "航点索引越界: idx=%d, size=%lu", idx,
                         static_cast<unsigned long>(waypoints.size()));
            return BT::NodeStatus::FAILURE;
        }

        auto goal = waypoints[idx];
        goal.header.stamp = rclcpp::Clock().now();

        double wait_sec = 0.0;
        if (wt_res && idx < static_cast<int>(wt_res.value().size())) {
            wait_sec = wt_res.value()[idx];
        }

        RCLCPP_INFO(rclcpp::get_logger("GetCurrentWaypoint"),
                    "当前目标航点[%d]: [%.2f, %.2f], 等待 %.1fs", idx,
                    goal.pose.position.x, goal.pose.position.y, wait_sec);

        setOutput("current_goal", goal);
        setOutput("current_wait_sec", wait_sec);
        return BT::NodeStatus::SUCCESS;
    }
};

// 通过 TF2 查询机器人位姿，判断是否到达目标点
class WaitUntilReached : public BT::StatefulActionNode {
public:
    WaitUntilReached(const std::string &name,
                     const BT::NodeConfiguration &config,
                     const std::shared_ptr<rclcpp::Node> &node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose", "目标位姿"),
            BT::InputPort<double>("tolerance", 0.5, "到达判定距离阈值 (m)"),
        };
    }

    BT::NodeStatus onStart() override {
        auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
        auto tol_res = getInput<double>("tolerance");

        if (!goal_res) {
            RCLCPP_ERROR(node_->get_logger(), "WaitUntilReached: 读取 goal_pose 失败");
            return BT::NodeStatus::FAILURE;
        }

        goal_ = goal_res.value();
        tolerance_ = (tol_res) ? tol_res.value() : 0.5;

        RCLCPP_INFO(node_->get_logger(),
                    "WaitUntilReached: 等待到达 [%.2f, %.2f], 阈值=%.2f",
                    goal_.pose.position.x, goal_.pose.position.y, tolerance_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "WaitUntilReached: TF 查询失败: %s", ex.what());
            return BT::NodeStatus::RUNNING;
        }

        double dx = transform.transform.translation.x - goal_.pose.position.x;
        double dy = transform.transform.translation.y - goal_.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < tolerance_) {
            RCLCPP_INFO(node_->get_logger(),
                        "WaitUntilReached: 已到达目标点 (距离=%.2f)", distance);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(), "WaitUntilReached: 被中断");
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PoseStamped goal_;
    double tolerance_ = 0.5;
};

// 到达航点后等待指定时间
class WaitDuration : public BT::StatefulActionNode {
public:
    WaitDuration(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("duration_sec", 5.0, "等待时长 (秒)，0 则跳过"),
        };
    }

    BT::NodeStatus onStart() override {
        auto dur_res = getInput<double>("duration_sec");
        duration_ = (dur_res) ? dur_res.value() : 0.0;

        if (duration_ <= 0.0) {
            return BT::NodeStatus::SUCCESS;
        }

        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(rclcpp::get_logger("WaitDuration"),
                    "在航点等待 %.1f 秒", duration_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed >= duration_) {
            RCLCPP_INFO(rclcpp::get_logger("WaitDuration"), "等待完成");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(rclcpp::get_logger("WaitDuration"), "等待被中断");
    }

private:
    double duration_ = 0.0;
    std::chrono::steady_clock::time_point start_time_;
};

// 切换到下一个航点（循环）
class NextWaypoint : public BT::SyncActionNode {
public:
    NextWaypoint(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::BidirectionalPort<int>("wp_idx", "当前航点索引"),
            BT::InputPort<int>("total_waypoints", "航点总数"),
        };
    }

    BT::NodeStatus tick() override {
        auto idx_res = getInput<int>("wp_idx");
        auto total_res = getInput<int>("total_waypoints");

        if (!idx_res || !total_res) {
            RCLCPP_ERROR(rclcpp::get_logger("NextWaypoint"), "读取端口失败");
            return BT::NodeStatus::FAILURE;
        }

        int idx = idx_res.value();
        int total = total_res.value();
        int next = (idx + 1) % total;

        RCLCPP_INFO(rclcpp::get_logger("NextWaypoint"),
                    "航点切换: %d -> %d (共 %d 个)", idx, next, total);

        setOutput("wp_idx", next);
        return BT::NodeStatus::SUCCESS;
    }
};