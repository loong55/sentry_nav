#include "uart_transporter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/serial_receive_data.hpp>
#include <../include/message.h>

#include <vector>
#include <thread>
#include <cstring>

// ================= 串口解析后的数据 =================
struct SerialData
{
  uint8_t  hp = 0;
  uint8_t  zone_status = 0;
};



// ================= 串口解析类 =================
class CODSerial
{
public:
  bool receive(UartTransporter& uart, SerialData& out)
  {
    uint8_t buf[64];
    int n = uart.read(buf, sizeof(buf));
    if (n <= 0) return false;

    rx_cache_.insert(rx_cache_.end(), buf, buf + n);
    return parseFrame(out);
  }

private:
  bool parseFrame(SerialData& out)
  {
    constexpr uint8_t HEAD = 0xFF;
    constexpr size_t LEN = 20;

    while (rx_cache_.size() >= LEN)
    {
      if (rx_cache_[0] != HEAD)
      {
        rx_cache_.erase(rx_cache_.begin());
        continue;
      }

      // ===== 按协议解析 =====
      out.hp = rx_cache_[2];
      out.zone_status = rx_cache_[3];

      rx_cache_.erase(rx_cache_.begin(), rx_cache_.begin() + LEN);
      return true;
    }
    return false;
  }

private:
  std::vector<uint8_t> rx_cache_;
};

// ================= ROS2 Node =================
class CODSerialNode : public rclcpp::Node
{
public:
  CODSerialNode()
  : Node("cod_serial_node"),
    uart_("/dev/ttyACM0", 115200)
  {
    publisher_ = this->create_publisher<
        rm_interfaces::msg::SerialReceiveData>(
        "/SerialReceiveData", 10);

    if (!uart_.open())
    {
    	throw std::runtime_error("Failed to open UART");
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&CODSerialNode::loop, this));

    RCLCPP_INFO(this->get_logger(), "COD Serial Node started");
  }

  ~CODSerialNode()
  {
    uart_.close();
  }

private:
  void loop()
  {
    SerialData data;
    if (serial_.receive(uart_, data))
    {
      rm_interfaces::msg::SerialReceiveData msg;

      // ===== 映射到自定义消息 =====
      msg.judge_system_data.hp = data.hp;
      msg.judge_system_data.zone_status = data.zone_status;

      publisher_->publish(msg);
    }
  }

private:
  CODSerial serial_;
  UartTransporter uart_;

  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ================= main =================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
 try
  {
    rclcpp::spin(std::make_shared<CODSerialNode>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("cod_serial"),
      "Node crashed: %s",
      e.what()
    );
  }  rclcpp::shutdown();
  return 0;
}
