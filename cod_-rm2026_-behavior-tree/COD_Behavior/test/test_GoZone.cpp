//
// Created by ysl on 2026/1/20.
//
//
// Created by ysl on 2026/1/20.
////
// Created by ysl on 2026/1/18.
//
#include "iostream"
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <rclcpp/rclcpp.hpp>

class sendmsg : public rclcpp::Node {
public:
    // 构造函数接受 CODSerial 对象的指针
    sendmsg() : Node("sendmsg") {
        // 创建发布器，发布到 SerialReceiveData 话题，队列大小为 10
        publisher_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>("SerialReceiveData", 10);

        // 创建定时器，每 500ms 调用一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&sendmsg::timer_callback, this));
    }

private:
    void timer_callback() {
        // 创建要发布的消息
        auto msg = rm_interfaces::msg::SerialReceiveData();

        msg.judge_system_data.hp = 400.0;
        msg.judge_system_data.zone_status = false;
        msg.judge_system_data.is_defence = false;
        msg.judge_system_data.is_attack = false;

        RCLCPP_INFO(this->get_logger(), "使用hp = 400.0      zone_status = false      is_defence = false         is_attack = false...........................\n");

        // 发布消息
        publisher_->publish(msg);
    }

    // 成员变量
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// 主函数
int main(int argc, char *argv[]) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);


    // 创建 sendmsg 节点，传入 CODSerial 对象
    auto node = std::make_shared<sendmsg>();

    // 运行 ROS2 节点
    while (rclcpp::ok()) {
        // 处理ROS2事件，包括定时器回调
        rclcpp::spin_some(node);

        // 这里可以添加其他自定义任务
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // 运行 ROS2 节点
    //rclcpp::spin(node);

    // 关闭 ROS2
    rclcpp::shutdown();

    return 0;
}
