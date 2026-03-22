// Copyright 2025 SMBU-PolarBear-Robotics-Team
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

#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms) 串口断开后重新连接的时间间隔
#define USB_PROTECT_SLEEP_TIME 1000  // (ms) 串口保护时间间隔（防止频繁打开关闭）

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{

namespace
{
const char * postureToString(uint8_t posture)
{
  switch (posture) {
    case pb_rm_interfaces::msg::PostureCmd::ATTACK:
      return "ATTACK(进攻)";
    case pb_rm_interfaces::msg::PostureCmd::DEFENSE:
      return "DEFENSE(防御)";
    case pb_rm_interfaces::msg::PostureCmd::MOVE:
      return "MOVE(移动)";
    default:
      return "UNKNOWN(未知)";
  }
}

const char * hpDeductionReasonToString(uint8_t reason)
{
  switch (reason) {
    case pb_rm_interfaces::msg::RobotStatus::ARMOR_HIT:
      return "ARMOR_HIT(弹丸命中装甲)";
    case pb_rm_interfaces::msg::RobotStatus::SYSTEM_OFFLINE:
      return "SYSTEM_OFFLINE(裁判系统离线)";
    case pb_rm_interfaces::msg::RobotStatus::OVER_SHOOT_SPEED:
      return "OVER_SHOOT_SPEED(射速超限)";
    case pb_rm_interfaces::msg::RobotStatus::OVER_HEAT:
      return "OVER_HEAT(热量超限)";
    case pb_rm_interfaces::msg::RobotStatus::OVER_POWER:
      return "OVER_POWER(功率超限)";
    case pb_rm_interfaces::msg::RobotStatus::ARMOR_COLLISION:
      return "ARMOR_COLLISION(装甲碰撞)";
    default:
      return "UNKNOWN_REASON(未知原因)";
  }
}
}  // namespace

// 构造函数实现
StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),
  owned_ctx_{new IoContext(2)}, //创建一个IoContext对象（用于异步IO操作），参数2表示使用2个线程
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)} //创建一个SerialDriver对象，用于串口通信，传入之前创建的IoContext
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");

  getParams(); // 获取参数，包括串口名称、波特率等
  createPublisher(); // 创建发布者，包括IMU数据、机器人状态信息等
  createSubscription(); // 创建订阅者，包括控制命令、目标点等

  robot_models_.chassis = {
    {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
  robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
  robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
  robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
  robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};

  serial_port_protect_thread_ = std::thread(&StandardRobotPpRos2Node::serialPortProtect, this); // 创建一个线程用于串口保护
  receive_thread_ = std::thread(&StandardRobotPpRos2Node::receiveData, this); // 创建一个线程用于接收数据
  send_thread_ = std::thread(&StandardRobotPpRos2Node::sendData, this); // 创建一个线程用于发送数据
}

//析构函数实现
StandardRobotPpRos2Node::~StandardRobotPpRos2Node()
{
  if (send_thread_.joinable()) { //检查线程是否还在运行且可以被join，返回 true：线程正在运行，返回 false：线程已结束或未启动
    send_thread_.join(); //阻塞当前线程，等待发送线程完全结束；join的作用，等待线程结束并回收资源，防止删除此时正在访问的数据
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_port_protect_thread_.joinable()) {
    serial_port_protect_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) { //检查指针是否有效（非nullptr）
    owned_ctx_->waitForExit();//等待IO上下文的所有异步操作完成，waitForExit() 阻塞直到这些线程完全停止
  }
}

//从下位机（串口）接收数据（receiveData方法，将串口字节流转化为自定义结构体）→  
//解析处理（publishImuData 方法，将转化后的结构体，转换为 ROS 2 消息格式） → 
//发布到ROS 2话题
void StandardRobotPpRos2Node::createPublisher()
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);
  robot_state_info_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);
  event_data_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::EventData>("referee/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  ground_robot_position_pub_ = this->create_publisher<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  buff_pub_ = this->create_publisher<pb_rm_interfaces::msg::Buff>("referee/buff", 10);
}

//动态创建调试发布者，用于实时发布下位机的调试变量，name为下位机传入的调试变量名
void StandardRobotPpRos2Node::createNewDebugPublisher(const std::string & name)
{
  RCLCPP_INFO(get_logger(), "Create new debug publisher: %s", name.c_str());
  std::string topic_name = "serial/debug/" + name;
  auto debug_pub = this->create_publisher<example_interfaces::msg::Float64>(topic_name, 10);
  debug_pub_map_.insert(std::make_pair(name, debug_pub));//用键值对形式，储存发布者指针
}

//动态创建调试订阅者，主要看底盘速度，这个订阅话题由NAV2的路径规划发布
void StandardRobotPpRos2Node::createSubscription()
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_nav2_result", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));

  cmd_gimbal_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "cmd_gimbal_joint", 10,
    std::bind(&StandardRobotPpRos2Node::cmdGimbalJointCallback, this, std::placeholders::_1));

  cmd_shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    "cmd_spin", 10,
    std::bind(&StandardRobotPpRos2Node::cmdShootCallback, this, std::placeholders::_1));
  cmd_posture_sub_ = this->create_subscription<pb_rm_interfaces::msg::PostureCmd>(
    "cmd_posture", 10,
    std::bind(&StandardRobotPpRos2Node::cmdPostureCallback, this, std::placeholders::_1));
  cmd_tracking_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "tracker/target", 10,
    std::bind(&StandardRobotPpRos2Node::visionTargetCallback, this, std::placeholders::_1));
}

//从ROS2参数服务器读取串口配置参数，初始化串口驱动
/*三种参数来源
1. 启动命令行
  $ ros2 run ... --ros-args -p device_name:=/dev/ttyUSB0

2. 配置文件（YAML）standard_robot_pp_ros2.yaml
  device_name: /dev/ttyUSB0
  baud_rate: 115200

3. launch文件
  <param name="device_name" value="/dev/ttyUSB0"/>
*/
void StandardRobotPpRos2Node::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl; //流控制，流控制用于防止数据传输过快导致接收端缓冲区溢出。
  using Parity = drivers::serial_driver::Parity; //奇偶校验
  using StopBits = drivers::serial_driver::StopBits; //停止位

  uint32_t baud_rate{}; //波特率
  auto fc = FlowControl::NONE; //不使用流控制
  auto pt = Parity::NONE; //不使用奇偶校验
  auto sb = StopBits::ONE; //1个停止位

  //向ROS2框架声明这个节点有一个参数device_name，从参数服务器读取参数值，如果参数不存在，使用默认值 ""
  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE; //不使用流控制
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE; //硬件流控制
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE; //软件流控制
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE; //不使用奇偶校验
    } else if (pt_string == "odd") {
      pt = Parity::ODD; //奇校验
    } else if (pt_string == "even") {
      pt = Parity::EVEN; //偶校验
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE; //1位停止位
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE; //1.5位停止位
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO; //2位停止位
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  //std::make_unique 工厂函数，用于创建 std::unique_ptr 智能指针，类似new操作符，但更安全
  /*
  // 1. 使用 new 创建原始指针
      drivers::serial_driver::SerialPortConfig* raw_ptr = 
      new drivers::serial_driver::SerialPortConfig(baud_rate, fc, pt, sb);

  // 2. 将原始指针的所有权转移给智能指针
      device_config_.reset(raw_ptr); 
  */
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

  record_rosbag_ = declare_parameter("record_rosbag", false); //是否记录rosbag
  set_detector_color_ = declare_parameter("set_detector_color", false); //是否设置检测器颜色
  debug_ = declare_parameter("debug", false); //是否调试
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  // @TODO: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口
  serial_driver_->init_port(device_name_, *device_config_);
  // 尝试打开串口
  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Serial port opened!");
      is_usb_ok_ = true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    is_usb_ok_ = false;
  }

  is_usb_ok_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }

        serial_driver_->port()->open();

        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;
        }
      } catch (const std::exception & ex) {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    }

    // thread sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!"); //线程启动时，打印日志信息

  constexpr size_t kMaxPayloadLen =
    sizeof(ReceiveDebugData) - sizeof(HeaderFrame) - sizeof(uint16_t);

  std::vector<uint8_t> sof(1); //储存单字节帧头
  std::vector<uint8_t> pending_bytes;

  auto read_exact = [this, &pending_bytes](std::vector<uint8_t> & buffer, size_t total_len) -> bool {
      buffer.clear();
      buffer.reserve(total_len);

      if (!pending_bytes.empty()) {
        const size_t reuse_len = std::min(total_len, pending_bytes.size());
        buffer.insert(buffer.end(), pending_bytes.begin(), pending_bytes.begin() + reuse_len);
        pending_bytes.erase(pending_bytes.begin(), pending_bytes.begin() + reuse_len);
      }

      while (buffer.size() < total_len) {
        std::vector<uint8_t> temp(total_len - buffer.size());
        int received_len = serial_driver_->port()->receive(temp);
        if (received_len <= 0) {
          return false;
        }

        const size_t copy_len =
          std::min(static_cast<size_t>(received_len), temp.size());
        buffer.insert(buffer.end(), temp.begin(), temp.begin() + copy_len);
      }

      if (buffer.size() > total_len) {
        pending_bytes.insert(pending_bytes.end(), buffer.begin() + total_len, buffer.end());
        buffer.resize(total_len);
      }

      return true;
    };

  int sof_count = 0; //帧头计数
  int retry_count = 0; // 串口异常重试计数

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      if (!read_exact(sof, 1)) {
        RCLCPP_WARN(get_logger(), "Read SOF failed");
        continue;
      }

      if (sof[0] != SOF_RECEIVE) { // SOF_RECEIVE = 0x5A
        sof_count++; // 帧头计数
        RCLCPP_INFO(get_logger(), "Not sof, cnt=%d", sof_count);
        continue; //如果不是帧头，跳出循环，进入下一次 while (rclcpp::ok())循环继续读取
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;

      // sof[0] == SOF_RECEIVE 后读取剩余 header_frame 内容
      std::vector<uint8_t> header_frame_buf;
      if (!read_exact(header_frame_buf, 3)) {
        RCLCPP_WARN(get_logger(), "Read header frame failed");
        continue;
      }
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加 sof
      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf); // 将串口字节流转化为 HeaderFrame 结构体

      // HeaderFrame CRC8 check
      // CRC8 校验只针对帧头部分，下位机通过帧头前3个字节计算 CRC8 校验码，放在第4个字节，上位机接收到帧头后，对前3个字节进行 CRC8 校验
      bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
      if (!crc8_ok) {
        RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
        continue; //帧头crc8校验失败，跳出循环，进入下一次 while (rclcpp::ok())循环继续读取
      }

      if (header_frame.len == 0 || header_frame.len > kMaxPayloadLen) {
        RCLCPP_ERROR(
          get_logger(),
          "Invalid payload length: %u (max=%zu), drop and resync",
          header_frame.len, kMaxPayloadLen);
        continue;
      }

      // crc8_ok 校验正确后读取数据段
      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf;
      if (!read_exact(data_buf, static_cast<size_t>(header_frame.len) + 2U)) {
        RCLCPP_WARN(get_logger(), "Read data segment failed, len=%u", header_frame.len);
        continue;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包;之前的data_buf初始化数据长度数据段+CRC16，这里会自动扩容
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      // 是否开启调试模式，在 getParams() 方法中初始化 debug_ 参数（默认false，可从YAML配置文件或launch文件设置）
      // 根据上位机和下位机的id共同决定，是否处理调试数据包
      // 开启调试模式后，上位机可以用话题或者rqt_graph查看下位机发送的调试数据
      // 上位机关闭调试模式后，如果下位机发送调试数据包，则上位机会忽略该数据包
      if (!debug_ && header_frame.id == ID_DEBUG) {
        continue;
      }

      // 整包crc16数据校验
      bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
      if (!crc16_ok) {
        std::ostringstream hex_stream;
        hex_stream << std::hex << std::setfill('0');
        for (size_t index = 0; index < data_buf.size(); ++index) {
          hex_stream << std::setw(2) << static_cast<int>(data_buf[index]);
          if (index + 1 < data_buf.size()) {
            hex_stream << ' ';
          }
        }
        RCLCPP_ERROR(
          get_logger(),
          "Data segment CRC16 error! frame_len=%zu payload_len=%u frame_hex=[%s]",
          data_buf.size(), header_frame.len, hex_stream.str().c_str());

        auto next_sof_it = std::find(data_buf.begin() + 1, data_buf.end(), SOF_RECEIVE);
        if (next_sof_it != data_buf.end()) {
          pending_bytes.assign(next_sof_it, data_buf.end());
          RCLCPP_WARN(
            get_logger(),
            "CRC16 resync: recovered %zu bytes starting from next SOF",
            pending_bytes.size());
        }
        continue;
      }

      // // crc16_ok 校验正确给出提示
      // RCLCPP_INFO_THROTTLE(
      //   get_logger(), *this->get_clock(), 1000,
      //   "CRC16 verify passed: frame_len=%zu payload_len=%u id=0x%02x",
      //   data_buf.size(), header_frame.len, header_frame.id);

      // crc16_ok 校验正确后根据 header_frame.id 解析数据
      switch (header_frame.id) {
        case ID_DEBUG: {
          ReceiveDebugData debug_data = fromVector<ReceiveDebugData>(data_buf);
          publishDebugData(debug_data);
        } break;
        case ID_IMU: {
          ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);
          publishImuData(imu_data);
        } break;
        case ID_ROBOT_STATE_INFO: {
          ReceiveRobotInfoData robot_info_data = fromVector<ReceiveRobotInfoData>(data_buf);
          publishRobotInfo(robot_info_data);
        } break;
        case ID_EVENT_DATA: {
          ReceiveEventData event_data = fromVector<ReceiveEventData>(data_buf);
          publishEventData(event_data);
        } break;
        case ID_PID_DEBUG: {
          RCLCPP_WARN(get_logger(), "Not implemented yet!");
        } break;
        case ID_ALL_ROBOT_HP: {
          ReceiveAllRobotHpData all_robot_hp_data = fromVector<ReceiveAllRobotHpData>(data_buf);
          publishAllRobotHp(all_robot_hp_data);
        } break;
        case ID_GAME_STATUS: {
          ReceiveGameStatusData game_status_data = fromVector<ReceiveGameStatusData>(data_buf);

          // //比赛状态解码调试信息
          // RCLCPP_INFO_THROTTLE(
          //   get_logger(), *this->get_clock(), 1000,
          //   "[ReceiveGameStatusData DECODE OK] game_progress=%u stage_remain_time=%u",
          //   game_status_data.data.game_progress,
          //   game_status_data.data.stage_remain_time);
          
            publishGameStatus(game_status_data);
        } break;
        case ID_ROBOT_MOTION: {
          ReceiveRobotMotionData robot_motion_data = fromVector<ReceiveRobotMotionData>(data_buf);
          publishRobotMotion(robot_motion_data);
        } break;
        case ID_GROUND_ROBOT_POSITION: {
          ReceiveGroundRobotPosition ground_robot_position_data =
            fromVector<ReceiveGroundRobotPosition>(data_buf);
          publishGroundRobotPosition(ground_robot_position_data);
        } break;
        case ID_RFID_STATUS: {
          ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(data_buf);

          // //中心点rfid解码
          // RCLCPP_INFO_THROTTLE(
          //   get_logger(), *this->get_clock(), 1000,
          //   "[ID_RFID_STATUS DECODE OK] center_gain_point=%s",
          //   rfid_status_data.data.center_gain_point ? "true" : "false");
          publishRfidStatus(rfid_status_data);
        } break;
        case ID_ROBOT_STATUS: {
          ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(data_buf);

          // // 机器人状态解码调试信息
          // const bool hp_deduced =
          //   (last_hp_ >= 0.0f) && (static_cast<float>(robot_status_data.data.current_hp) < last_hp_);
          // RCLCPP_INFO_THROTTLE(
          //   get_logger(), *this->get_clock(), 1000,
          //   "[0x0B DECODE OK] hp=%u angle=%.3f is_hp_deduced=%s",
          //   robot_status_data.data.current_hp,
          //   robot_status_data.data.robot_pos_angle,
          //   hp_deduced ? "true" : "false");

          publishRobotStatus(robot_status_data);
        } break;
        case ID_JOINT_STATE: {
          ReceiveJointState joint_state_data = fromVector<ReceiveJointState>(data_buf);
          publishJointState(joint_state_data);
        } break;
        case ID_BUFF: {
          ReceiveBuff buff = fromVector<ReceiveBuff>(data_buf);
          publishBuff(buff);
        } break;
        default: {
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
        } break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

//发布调试数据
void StandardRobotPpRos2Node::publishDebugData(ReceiveDebugData & received_debug_data)
{
  static rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr debug_pub;

  // 遍历所有调试数据包
  for (auto & package : received_debug_data.packages) {
    // Create a vector to hold the non-zero data
    std::vector<uint8_t> non_zero_data;
    for (unsigned char name : package.name) { // 遍历所有数据包中的名称
      if (name != 0) {
        non_zero_data.push_back(name);
      } else {
        break; // 遇到0字节（字符串结尾）就停止
      }
    }
    // Convert the non-zero data to a string
    std::string name(non_zero_data.begin(), non_zero_data.end());

    if (name.empty()) {
      continue;
    }

    if (debug_pub_map_.find(name) == debug_pub_map_.end()) {
      createNewDebugPublisher(name);
    }
    debug_pub = debug_pub_map_.at(name);

    example_interfaces::msg::Float64 msg;
    msg.data = package.data;
    debug_pub->publish(msg);
  }
}


//发布IMU数据
void StandardRobotPpRos2Node::publishImuData(ReceiveImuData & imu_data)
{
  sensor_msgs::msg::JointState joint_msg;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = joint_msg.header.stamp = now();
  imu_msg.header.frame_id = "gimbal_pitch_odom";

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
  imu_msg.orientation = tf2::toMsg(q);
  imu_msg.angular_velocity.x = imu_data.data.roll_vel;
  imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
  imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
  imu_pub_->publish(imu_msg);

  joint_msg.name = {
    "gimbal_pitch_joint",
    "gimbal_yaw_joint",
    "gimbal_pitch_odom_joint",
    "gimbal_yaw_odom_joint",
  };
  joint_msg.position = {
    imu_data.data.pitch,
    imu_data.data.yaw,
    last_gimbal_pitch_odom_joint_,
    last_gimbal_yaw_odom_joint_,
  };
  joint_state_pub_->publish(joint_msg);
}

// 发布机器人状态信息
void StandardRobotPpRos2Node::publishRobotInfo(ReceiveRobotInfoData & robot_info)
{
  pb_rm_interfaces::msg::RobotStateInfo msg;

  // 设置时间戳：秒和纳秒
  msg.header.stamp.sec = robot_info.time_stamp / 1000; // 毫秒转换为秒
  msg.header.stamp.nanosec = (robot_info.time_stamp % 1000) * 1e6; // 毫秒转换为纳秒
  msg.header.frame_id = "odom";

  // 从预定义的robot_models_映射中查找对应的型号名称
  msg.models.chassis = robot_models_.chassis.at(robot_info.data.type.chassis);
  msg.models.gimbal = robot_models_.gimbal.at(robot_info.data.type.gimbal);
  msg.models.shoot = robot_models_.shoot.at(robot_info.data.type.shoot);
  msg.models.arm = robot_models_.arm.at(robot_info.data.type.arm);
  msg.models.custom_controller =
    robot_models_.custom_controller.at(robot_info.data.type.custom_controller);

  robot_state_info_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishEventData(ReceiveEventData & event_data)
{
  pb_rm_interfaces::msg::EventData msg;

  // 直接复制下位机传来的各个事件标志位
  msg.non_overlapping_supply_zone = event_data.data.non_overlapping_supply_zone;
  msg.overlapping_supply_zone = event_data.data.overlapping_supply_zone;
  msg.supply_zone = event_data.data.supply_zone;

  msg.small_energy = event_data.data.small_energy;
  msg.big_energy = event_data.data.big_energy;

  msg.central_highland = event_data.data.central_highland;
  msg.trapezoidal_highland = event_data.data.trapezoidal_highland;

  msg.center_gain_zone = event_data.data.center_gain_zone;
  msg.fortress_gain_zone = event_data.data.fortress_gain_zone;
  msg.outpost_gain_zone = event_data.data.outpost_gain_zone;
  msg.base_gain_zone = event_data.data.base_gain_zone;

  event_data_pub_->publish(msg);
}

//发布基地和前哨站血量
void StandardRobotPpRos2Node::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  pb_rm_interfaces::msg::GameRobotHP msg;

  msg.ally_outpost_hp = 1;
  msg.ally_base_hp = 1;

  msg.ally_outpost_hp = all_robot_hp.data.ally_outpost_hp;
  msg.ally_base_hp = all_robot_hp.data.ally_base_hp;

  all_robot_hp_pub_->publish(msg);
}

// 发布比赛状态
void StandardRobotPpRos2Node::publishGameStatus(ReceiveGameStatusData & game_status)
{
  pb_rm_interfaces::msg::GameStatus msg;
  msg.game_progress = game_status.data.game_progress; // 比赛阶段
  msg.stage_remain_time = game_status.data.stage_remain_time;  // 阶段剩余时间
  game_status_pub_->publish(msg);

  // 比赛状态解码后写入消息接口，调试信息
  RCLCPP_INFO_THROTTLE(
    get_logger(), *this->get_clock(), 1000,
    "[ReceiveGameStatusData PUB OK] topic=/referee/game_status game_progress=%u stage_remain_time=%u",
    msg.game_progress,
    msg.stage_remain_time);

  // 如果启用了rosbag录制，根据比赛状态自动控制rosbag录制
  if (record_rosbag_ && game_status.data.game_progress != previous_game_progress_) {
    previous_game_progress_ = game_status.data.game_progress; // 更新上一个比赛状态
    RCLCPP_INFO(get_logger(), "Game progress: %d", game_status.data.game_progress);

    std::string service_name;
    switch (game_status.data.game_progress) {
      case pb_rm_interfaces::msg::GameStatus::COUNT_DOWN:
        service_name = "start_recording"; // 开始录制
        break;
      case pb_rm_interfaces::msg::GameStatus::GAME_OVER:
        service_name = "stop_recording"; // 停止录制
        break;
      default:
        return;
    }

    if (!callTriggerService(service_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to call service: %s", service_name.c_str());
    }
  }
}

// 发布机器人运动信息 
void StandardRobotPpRos2Node::publishRobotMotion(ReceiveRobotMotionData & robot_motion)
{
  geometry_msgs::msg::Twist msg;  // Twist消息表示线速度和角速度

  msg.linear.x = robot_motion.data.speed_vector.vx; 
  msg.linear.y = robot_motion.data.speed_vector.vy;
  msg.angular.z = robot_motion.data.speed_vector.wz;

  robot_motion_pub_->publish(msg);
}

// 发布地面机器人位置
void StandardRobotPpRos2Node::publishGroundRobotPosition(
  ReceiveGroundRobotPosition & ground_robot_position)
{
  pb_rm_interfaces::msg::GroundRobotPosition msg;

  msg.hero_position.x = ground_robot_position.data.hero_x;
  msg.hero_position.y = ground_robot_position.data.hero_y;

  msg.engineer_position.x = ground_robot_position.data.engineer_x;
  msg.engineer_position.y = ground_robot_position.data.engineer_y;

  msg.standard_3_position.x = ground_robot_position.data.standard_3_x;
  msg.standard_3_position.y = ground_robot_position.data.standard_3_y;

  msg.standard_4_position.x = ground_robot_position.data.standard_4_x;
  msg.standard_4_position.y = ground_robot_position.data.standard_4_y;

  ground_robot_position_pub_->publish(msg);
}

// 发布RFID状态
void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  pb_rm_interfaces::msg::RfidStatus msg;

  msg.base_gain_point = rfid_status.data.base_gain_point;
  msg.central_highland_gain_point = rfid_status.data.central_highland_gain_point;
  msg.enemy_central_highland_gain_point = rfid_status.data.enemy_central_highland_gain_point;
  msg.friendly_trapezoidal_highland_gain_point =
    rfid_status.data.friendly_trapezoidal_highland_gain_point;
  msg.enemy_trapezoidal_highland_gain_point =
    rfid_status.data.enemy_trapezoidal_highland_gain_point;
  msg.friendly_fly_ramp_front_gain_point = rfid_status.data.friendly_fly_ramp_front_gain_point;
  msg.friendly_fly_ramp_back_gain_point = rfid_status.data.friendly_fly_ramp_back_gain_point;
  msg.enemy_fly_ramp_front_gain_point = rfid_status.data.enemy_fly_ramp_front_gain_point;
  msg.enemy_fly_ramp_back_gain_point = rfid_status.data.enemy_fly_ramp_back_gain_point;
  msg.friendly_central_highland_lower_gain_point =
    rfid_status.data.friendly_central_highland_lower_gain_point;
  msg.friendly_central_highland_upper_gain_point =
    rfid_status.data.friendly_central_highland_upper_gain_point;
  msg.enemy_central_highland_lower_gain_point =
    rfid_status.data.enemy_central_highland_lower_gain_point;
  msg.enemy_central_highland_upper_gain_point =
    rfid_status.data.enemy_central_highland_upper_gain_point;
  msg.friendly_highway_lower_gain_point = rfid_status.data.friendly_highway_lower_gain_point;
  msg.friendly_highway_upper_gain_point = rfid_status.data.friendly_highway_upper_gain_point;
  msg.enemy_highway_lower_gain_point = rfid_status.data.enemy_highway_lower_gain_point;
  msg.enemy_highway_upper_gain_point = rfid_status.data.enemy_highway_upper_gain_point;
  msg.friendly_fortress_gain_point = rfid_status.data.friendly_fortress_gain_point;
  msg.friendly_outpost_gain_point = rfid_status.data.friendly_outpost_gain_point;
  msg.friendly_supply_zone_non_exchange = rfid_status.data.friendly_supply_zone_non_exchange;
  msg.friendly_supply_zone_exchange = rfid_status.data.friendly_supply_zone_exchange;
  msg.friendly_big_resource_island = rfid_status.data.friendly_big_resource_island;
  msg.enemy_big_resource_island = rfid_status.data.enemy_big_resource_island;
  msg.center_gain_point = rfid_status.data.center_gain_point;

  rfid_status_pub_->publish(msg);

  // RCLCPP_INFO_THROTTLE(
  //   get_logger(), *this->get_clock(), 1000,
  //   "[ID_RFID_STATUS PUB OK] topic=/referee/rfid_status center_gain_point=%s",
  //   msg.center_gain_point ? "true" : "false");
}

// 发布机器人状态
void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  pb_rm_interfaces::msg::RobotStatus msg;

  latest_robot_id_ = robot_status.data.robot_id;

  //基本信息
  msg.robot_id = robot_status.data.robot_id; //机器人id
  msg.robot_level = robot_status.data.robot_level; //机器人等级
  msg.current_hp = robot_status.data.current_hp; //当前血量
  msg.maximum_hp = robot_status.data.maximum_hp; //最大血量

  //枪管热量相关
  msg.shooter_barrel_cooling_value = robot_status.data.shooter_barrel_cooling_value; //枪管冷却值
  msg.shooter_barrel_heat_limit = robot_status.data.shooter_barrel_heat_limit; //枪管热量上限
  msg.shooter_17mm_1_barrel_heat = robot_status.data.shooter_17mm_1_barrel_heat; //当前热量

  //机器人位置和朝向
  msg.robot_pos.position.x = robot_status.data.robot_pos_x;
  msg.robot_pos.position.y = robot_status.data.robot_pos_y;
  // 使用四元数表示方向（绕Z轴旋转）
  msg.robot_pos.orientation =
    tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_status.data.robot_pos_angle));

  // 伤害信息  
  msg.armor_id = robot_status.data.armor_id; //被击中的装甲id
  msg.hp_deduction_reason = robot_status.data.hp_deduction_reason; //血量减少原因

  //弹药和金币
  msg.projectile_allowance_17mm = robot_status.data.projectile_allowance_17mm; //17mm弹药数量
  msg.remaining_gold_coin = robot_status.data.remaining_gold_coin; //剩余的金币数量
  msg.out_of_combat_status = robot_status.data.out_of_combat_status; //哨兵是否处于脱战状态，1为是，0为否
  msg.fire_rem_17mm = robot_status.data.fire_rem_17mm; //队伍 17mm 允许发弹量的剩余可兑换数
  msg.current_posture = robot_status.data.current_posture; //当前姿态（进攻1 防御2 移动3）
  msg.energy_mechanism_activable = robot_status.data.energy_mechanism_activable; //己方能量机关是否能够进入正在激活状态
  msg.shoot_state = robot_status.data.shoot_state; //自瞄状态

  // 检测是否被扣血（当前血量 < 上次血量），首帧(last_hp_ < 0)不判定
  msg.is_hp_deduced =
    (last_hp_ >= 0.0f) && (static_cast<float>(msg.current_hp) < last_hp_);

  if (msg.is_hp_deduced) {
    RCLCPP_WARN(
      get_logger(),
      "[0x0B HP DROP EVENT] hp %u -> %u, delta=%d, reason=%u(%s), armor_id=%u",
      static_cast<unsigned int>(last_hp_),
      msg.current_hp,
      static_cast<int>(last_hp_) - static_cast<int>(msg.current_hp),
      msg.hp_deduction_reason,
      hpDeductionReasonToString(msg.hp_deduction_reason),
      msg.armor_id);
  }

  last_hp_ = robot_status.data.current_hp; //保存本次血量

  robot_status_pub_->publish(msg);

  //机器人状态解码后，写入消息接口的调试信息
  RCLCPP_INFO_THROTTLE(
    get_logger(), *this->get_clock(), 1000,
    "[0x0B PUB OK] topic=/referee/robot_status hp=%u angle=%.3f is_hp_deduced=%s",
    msg.current_hp,
    robot_status.data.robot_pos_angle,
    msg.is_hp_deduced ? "true" : "false");

  // 自动设置目标检测器的颜色
  if (set_detector_color_) {
    uint8_t detect_color;
    if (getDetectColor(robot_status.data.robot_id, detect_color)) {
      if (!initial_set_param_ || detect_color != previous_receive_color_) {
        previous_receive_color_ = detect_color;
        setParam(rclcpp::Parameter("detect_color", detect_color));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
}

// 保存云台的俯仰和偏航角度，供publishImuData使用
void StandardRobotPpRos2Node::publishJointState(ReceiveJointState & packet)
{
  last_gimbal_pitch_odom_joint_ = packet.data.pitch;
  last_gimbal_yaw_odom_joint_ = packet.data.yaw;
}

// 发布增益状态
void StandardRobotPpRos2Node::publishBuff(ReceiveBuff & buff)
{
  pb_rm_interfaces::msg::Buff msg;
  msg.recovery_buff = buff.data.recovery_buff;              // 恢复增益
  msg.cooling_buff = buff.data.cooling_buff;                // 冷却增益
  msg.defence_buff = buff.data.defence_buff;                // 防御增益
  msg.vulnerability_buff = buff.data.vulnerability_buff;    // 易伤增益
  msg.attack_buff = buff.data.attack_buff;                  // 攻击增益
  msg.remaining_energy = buff.data.remaining_energy;        // 剩余能量值
  buff_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  // 初始化发送数据包
  send_robot_cmd_data_.frame_header.sof = SOF_SEND;        // 帧头起始字节
  send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;     // 数据包ID
  send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;  // 数据长度

  // 初始化速度为0，姿态为移动3，射击和摩擦轮状态为关闭
  send_robot_cmd_data_.data.speed_vector.vx = 0;
  send_robot_cmd_data_.data.speed_vector.vy = 0;
  send_robot_cmd_data_.data.speed_vector.wz = 0;
  send_robot_cmd_data_.data.shoot.fire = 0;
  send_robot_cmd_data_.data.shoot.fric_on = 0;
  send_robot_cmd_data_.data.posture.posture = 3;

  // 计算帧头CRC8校验
  crc8::append_CRC8_check_sum(
    reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));

  int retry_count = 0;
  uint8_t last_logged_posture = 0;

  while (rclcpp::ok()) {  // 持续运行直到ROS2关闭
    if (!is_usb_ok_) {
      // USB（串口）连接不正常，等待并重试
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    //RCLCPP_ERROR(get_logger(), "Error ");
    
    try {
      send_robot_cmd_data_.time_stamp =
        static_cast<uint32_t>(this->get_clock()->now().nanoseconds() / 1000000ULL);

      const uint8_t current_posture = send_robot_cmd_data_.data.posture.posture;
      if (current_posture != last_logged_posture) {
        RCLCPP_WARN(
          get_logger(), "Publishing posture: %s (%u)", postureToString(current_posture),
          current_posture);
        last_logged_posture = current_posture;
      }

      // 添加数据段CRC16校验（覆盖所有数据）
      crc16::append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(SendRobotCmdData));

      // 将C++结构体转换为字节向量
      std::vector<uint8_t> send_data = toVector(send_robot_cmd_data_);

      // 通过串口发送数据
      serial_driver_->port()->send(send_data);

    //  RCLCPP_ERROR(get_logger(), "Sending data: vx=%.2f, vy=%.2f, wz=%.2f", 
    //         send_robot_cmd_data_.data.speed_vector.vx,
    //         send_robot_cmd_data_.data.speed_vector.vy,
    //         send_robot_cmd_data_.data.speed_vector.wz);

    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }
    
    // 发送频率约200Hz (5ms间隔)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

//底盘速度回调
void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 接收NAV2的路径规划输出（cmd_vel_nav2_result话题）
  send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
  send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
  send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;

  std::cout<<msg->linear.x<< " "<<msg->linear.y<<std::endl;  // 控制台输出
}

//云台关节回调
void StandardRobotPpRos2Node::cmdGimbalJointCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() != msg->position.size()) {
    RCLCPP_ERROR(
      get_logger(), "JointState message name and position arrays are of different sizes");
    return;
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "gimbal_pitch_joint") {
      send_robot_cmd_data_.data.gimbal.pitch = msg->position[i];
    } else if (msg->name[i] == "gimbal_yaw_joint") {
      send_robot_cmd_data_.data.gimbal.yaw = msg->position[i];
    }
  }
}

//视觉目标回调
void StandardRobotPpRos2Node::visionTargetCallback(
  const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  //send_robot_cmd_data_.data.tracking.tracking = msg->tracking;
}

//射击回调
void StandardRobotPpRos2Node::cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
  send_robot_cmd_data_.data.shoot.fire = msg->data;
}

//机器人姿态回调
void StandardRobotPpRos2Node::cmdPostureCallback(
  const pb_rm_interfaces::msg::PostureCmd::SharedPtr msg)
{
  if (msg->posture < pb_rm_interfaces::msg::PostureCmd::ATTACK ||
    msg->posture > pb_rm_interfaces::msg::PostureCmd::MOVE)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Invalid posture command: %u (valid range: 1~3)", msg->posture);
    return;
  }
  send_robot_cmd_data_.data.posture.posture = msg->posture;
  // RCLCPP_INFO(
  //   get_logger(), "Posture command received: %s (%u)", postureToString(msg->posture),
  //   msg->posture);
}
// void StandardRobotPpRos2Node::cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
// {
//   send_robot_cmd_data_.data.shoot.fric_on = true;
//   send_robot_cmd_data_.data.shoot.fire = msg->data;
// }

//参数设置函数（视觉探测相关）
void StandardRobotPpRos2Node::setParam(const rclcpp::Parameter & param)
{
  if (!initial_set_param_) {
    auto node_graph = this->get_node_graph_interface();
    auto node_names = node_graph->get_node_names();
    std::vector<std::string> possible_detectors = {
      "armor_detector_openvino", "armor_detector_opencv"};

    for (const auto & name : possible_detectors) {
      for (const auto & node_name : node_names) {
        if (node_name.find(name) != std::string::npos) {
          detector_node_name_ = node_name;
          break;
        }
      }
      if (!detector_node_name_.empty()) {
        break;
      }
    }

    if (detector_node_name_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
      return;
    }

    detector_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
    if (!detector_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
      return;
    }
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

// 根据机器人ID获取检测颜色
bool StandardRobotPpRos2Node::getDetectColor(uint8_t robot_id, uint8_t & color)
{
  // 机器人ID判断规则：
  // - 红队：1-11号
  // - 蓝队：101-111号
  if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
    return false;
  }
  color = (robot_id >= 100) ? 0 : 1; // 0:红色, 1:蓝色
  return true;
}

// 调用Trigger服务（用于rosbag录制控制）
bool StandardRobotPpRos2Node::callTriggerService(const std::string & service_name)
{
  // 创建服务客户端
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
 
  // 等待服务可用（最多等待5秒）
  auto start_time = std::chrono::steady_clock::now();
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service: %s", service_name.c_str());
      return false;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time > std::chrono::seconds(5)) {
      RCLCPP_ERROR(
        get_logger(), "Service %s not available after 5 seconds, giving up.", service_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }

  // 异步调用服务
  auto result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      get_logger(), "Service %s call succeeded: %s", service_name.c_str(),
      result.get()->success ? "true" : "false");
    return result.get()->success;
  }

  RCLCPP_ERROR(get_logger(), "Service %s call failed", service_name.c_str());
  return false;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)
