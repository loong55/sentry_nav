#include "uart_transporter.hpp"
#include <Eigen/Eigen>
#include "colortxt/color_txt.hpp"
#include "iostream"
#include "iomanip"
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <rclcpp/rclcpp.hpp>

// 添加必要的头文件
#include <memory>
#include <optional>  // 用于可选的返回值
// sendmsg 类
class sendmsg : public rclcpp::Node {
public:
    sendmsg()
        : Node("sendmsg") {
        // 创建发布器，发布到 SerialReceiveData 话题，队列大小为 10
        publisher_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>("SerialReceiveData", 10);
        // 创建定时器，每 50ms 调用一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&sendmsg::timer_callback, this));
    }

private:
    //接收串口消息
    bool receiveData(UartTransporter &uart) {
        uint8_t package[26];
        uint8_t header=0xff;
        if (!uart.open()) {
            std::cout << "Failed to open serial port" << std::endl;
            return false;
        }
        // 读取数据
        if (uart.read(package, sizeof(package)) == sizeof(package)) {
            if (header == package[0]) {
                is_recover = package[14];
                self_status = package[15];
                zone_status = package[16];
                is_defence = package[17];
                std::memcpy(&hp,&package[18],sizeof(float));
                return true;
            }else {
                std::cout << "Failed to open serial port\n";
                std::cout << "invaild package header\n";
                return false;
            }
        }
        return false;
    }

    void timer_callback() {

        // 创建要发布的消息
        auto msg = rm_interfaces::msg::SerialReceiveData();

        if (receiveData(uart_)) {
            // 根据实际需求将浮点数映射到消息字段
            msg.judge_system_data.hp = hp;
            RCLCPP_INFO(this->get_logger(), "发布hp: %f", msg.judge_system_data.hp);

            msg.judge_system_data.zone_status = zone_status;
            RCLCPP_INFO(this->get_logger(), "发布zone_status: %d", msg.judge_system_data.zone_status);
            msg.judge_system_data.self_status = self_status;

            msg.judge_system_data.is_defence = is_defence;
            RCLCPP_INFO(this->get_logger(), "发布is_defence: %d", msg.judge_system_data.is_defence);

            msg.judge_system_data.is_attack = is_attack;
            RCLCPP_INFO(this->get_logger(), "发布is_attack: %d", msg.judge_system_data.is_attack);
            msg.judge_system_data.is_recover = is_recover;

        } else {
              //如果没有新数据，使用默认值
              // msg.judge_system_data.hp = 400.0;
              // msg.judge_system_data.zone_status = false;
              // msg.judge_system_data.is_defence = false;
              // msg.judge_system_data.is_attack = false;
              // RCLCPP_INFO(this->get_logger(), "使用默认值...........................");
        }

        // 发布消息
        publisher_->publish(msg);
    }

    // 成员变量
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    UartTransporter uart_= UartTransporter("/dev/ttyACM0",115200);
    float hp = 400.0;
    bool self_status = false;
    bool is_defence = false;
    bool is_attack = false;
    bool is_recover = false;
    bool zone_status = false;
};

// 主函数
int main(int argc, char *argv[]) {
    // 初始化 ROS2
    rclcpp::init(argc, argv);


    // 创建 sendmsg 节点，传入 CODSerial 对象
    auto node = std::make_shared<sendmsg>();

    while (rclcpp::ok()) {
        // 处理ROS2事件，包括定时器回调
        rclcpp::spin_some(node);

        // 这里可以添加其他自定义任务
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 关闭 ROS2
    rclcpp::shutdown();

    return 0;
}