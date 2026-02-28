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

#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_

#include <algorithm>   // std::copy
#include <cstdint>     // uint8_t, uint16_t 等固定宽度整数类型
#include <vector>      // std::vector

namespace standard_robot_pp_ros2
{
const uint8_t SOF_RECEIVE = 0x5A; // 起始标志（Start Of Frame），接收数据
const uint8_t SOF_SEND = 0x5A;    // 起始标志（Start Of Frame），发送数据

// Receive 接收数据包ID映射表
const uint8_t ID_DEBUG = 0x01;       // 调试信息    
const uint8_t ID_IMU = 0x02;        // IMU数据
const uint8_t ID_ROBOT_STATE_INFO = 0x03; // 机器人信息
const uint8_t ID_EVENT_DATA = 0x04; // 事件数据
const uint8_t ID_PID_DEBUG = 0x05;  // PID调试数据
const uint8_t ID_ALL_ROBOT_HP = 0x06; // 所有机器人血量
const uint8_t ID_GAME_STATUS = 0x07;  // 游戏状态
const uint8_t ID_ROBOT_MOTION = 0x08; // 机器人运动状态
const uint8_t ID_GROUND_ROBOT_POSITION = 0x09;  // 地面机器人位置
const uint8_t ID_RFID_STATUS = 0x0A;  // RFID状态
const uint8_t ID_ROBOT_STATUS = 0x0B; // 机器人状态
const uint8_t ID_JOINT_STATE = 0x0C;  // 关节状态
const uint8_t ID_BUFF = 0x0D;        // Buff状态
// Send 发送数据包ID映射表
const uint8_t ID_ROBOT_CMD = 0x01; // 机器人控制指令

const uint8_t DEBUG_PACKAGE_NUM = 10; // 调试数据包中包含的数据项数量
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10; // 调试数据包中每个数据项名称的长度

// 数据包头结构体
struct HeaderFrame
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包，包含帧头，时间戳，10个调试数据项，每个调试数据项包含名称、类型和数据值
struct ReceiveDebugData
{
  HeaderFrame frame_header; // 数据帧头，4个字节
  uint32_t time_stamp;  // 时间戳，4个字节
  struct
  {
    uint8_t name[DEBUG_PACKAGE_NAME_LEN];
    uint8_t type;
    float data;
  } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];

  uint16_t checksum;
} __attribute__((packed));

// IMU 数据包
struct ReceiveImuData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    float yaw;    // rad
    float pitch;  // rad
    float roll;   // rad

    float yaw_vel;    // rad/s
    float pitch_vel;  // rad/s
    float roll_vel;   // rad/s

    // float x_accel;  // m/s^2
    // float y_accel;  // m/s^2
    // float z_accel;  // m/s^2
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人信息数据包
struct ReceiveRobotInfoData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    /// @brief 机器人部位类型 2 bytes
    struct
    {
      uint16_t chassis : 3; //chassis 指定只占用3个比特位，即000~111,0~7,8种类型
      uint16_t gimbal : 3;
      uint16_t shoot : 3;
      uint16_t arm : 3;
      uint16_t custom_controller : 3;
      uint16_t reserve : 1;
    } __attribute__((packed)) type;

    /// @brief 机器人部位状态 1 byte
    /// @note 0: 错误，1: 正常
    struct
    {
      uint8_t chassis : 1; //底盘状态
      uint8_t gimbal : 1; //云台状态
      uint8_t shoot : 1; //发射机构状态
      uint8_t arm : 1; //机械臂状态
      uint8_t custom_controller : 1;  //自定义控制器状态
      uint8_t reserve : 3; //保留3位（未使用）
    } __attribute__((packed)) state;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 事件数据包
struct ReceiveEventData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t non_overlapping_supply_zone : 1; //占领非重叠补给区？
    uint8_t overlapping_supply_zone : 1; //占领重叠补给区？
    uint8_t supply_zone : 1; //占领补给区？

    uint8_t small_energy : 1;  // 小能量机关触发？
    uint8_t big_energy : 1;  // 大能量机关触发？

    uint8_t central_highland : 2; // 中央高地占领状态
    uint8_t reserved1 : 1;  // 保留1位

    uint8_t trapezoidal_highland : 2; //梯形高地占领状态

    uint8_t center_gain_zone : 2;// 中心增益区占领状态

    uint8_t reserved2 : 4; // 保留4位
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// PID调参数据包
struct ReceivePidDebugData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct
  {
    float fdb; //反馈值
    float ref; //目标值
    float pid_out; //PID输出值
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 全场机器人hp信息数据包
struct ReceiveAllRobotHpData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {

    uint16_t red_outpost_hp; // 红方前哨站血量
    uint16_t red_base_hp; // 红方基地血量


    uint16_t blue_outpost_hp; // 蓝方前哨站血量
    uint16_t blue_base_hp; // 蓝方基地血量
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 比赛信息数据包
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t game_progress; // 比赛进程
    uint16_t stage_remain_time; // 阶段剩余时间
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人运动数据包
struct ReceiveRobotMotionData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx; // x轴速度
      float vy; // y轴速度
      float wz; // 角速度
    } __attribute__((packed)) speed_vector;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));  

// 地面机器人位置数据包
struct ReceiveGroundRobotPosition
{
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct
  {
    float hero_x; // 英雄机器人x坐标
    float hero_y; // 英雄机器人y坐标

    float engineer_x; // 工程机器人x坐标
    float engineer_y; // 工程机器人y坐标

    float standard_3_x; // 标准机器人3x坐标
    float standard_3_y; // 标准机器人3y坐标

    float standard_4_x; // 标准机器人4x坐标
    float standard_4_y; // 标准机器人4y坐标

    float reserved1; // 保留
    float reserved2; // 保留
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// RFID 状态数据包
struct ReceiveRfidStatus
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint32_t base_gain_point : 1; // 基地得分点
    uint32_t central_highland_gain_point : 1; // 中央高地得分点
    uint32_t enemy_central_highland_gain_point : 1; // 敌方中央高地得分点
    uint32_t friendly_trapezoidal_highland_gain_point : 1; // 友方梯形高地得分点
    uint32_t enemy_trapezoidal_highland_gain_point : 1; // 敌方梯形高地得分点
    uint32_t friendly_fly_ramp_front_gain_point : 1; // 友方飞坡前得分点
    uint32_t friendly_fly_ramp_back_gain_point : 1; // 友方飞坡后得分点
    uint32_t enemy_fly_ramp_front_gain_point : 1;   // 敌方飞坡前得分点
    uint32_t enemy_fly_ramp_back_gain_point : 1; // 敌方飞坡后得分点
    uint32_t friendly_central_highland_lower_gain_point : 1; // 友方中央高地低得分点
    uint32_t friendly_central_highland_upper_gain_point : 1; // 友方中央高地高得分点
    uint32_t enemy_central_highland_lower_gain_point : 1; // 敌方中央高地低得分点
    uint32_t enemy_central_highland_upper_gain_point : 1; // 敌方中央高地高得分点
    uint32_t friendly_highway_lower_gain_point : 1; // 友方公路低得分点
    uint32_t friendly_highway_upper_gain_point : 1; // 友方公路高得分点
    uint32_t enemy_highway_lower_gain_point : 1;   // 敌方公路低得分点
    uint32_t enemy_highway_upper_gain_point : 1; // 敌方公路高得分点
    uint32_t friendly_fortress_gain_point : 1; // 友方堡垒得分点
    uint32_t friendly_outpost_gain_point : 1; // 友方前哨得分点
    uint32_t friendly_supply_zone_non_exchange : 1; // 友方补给区非交换区
    uint32_t friendly_supply_zone_exchange : 1; // 友方补给区交换区
    uint32_t friendly_big_resource_island : 1; // 友方大资源岛
    uint32_t enemy_big_resource_island : 1; // 敌方大资源岛
    uint32_t center_gain_point : 1; // 中央高地得分点
    uint32_t reserved : 8; // 保留8位
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 机器人状态数据包
struct ReceiveRobotStatus
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    uint8_t robot_id; // 机器人id
    uint8_t robot_level; // 机器人等级
    uint16_t current_hp; // 当前血量
    uint16_t maximum_hp; // 最大血量
    uint16_t shooter_barrel_cooling_value; // 射手冷却值
    uint16_t shooter_barrel_heat_limit; // 射手热量上限

    uint16_t shooter_17mm_1_barrel_heat; // 17mm 1号枪管热量

    float robot_pos_x; // 机器人位置x
    float robot_pos_y; // 机器人位置y
    float robot_pos_angle; // 机器人角度

    uint8_t armor_id : 4; // 装甲id
    uint8_t hp_deduction_reason : 4; // 伤害扣除原因

    uint16_t projectile_allowance_17mm; // 17mm弹丸余量
    uint16_t remaining_gold_coin; // 剩余金币
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed));

// 云台状态数据包
struct ReceiveJointState
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    float pitch; // 云台俯仰角度
    float yaw; // 云台水平角度
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 机器人增益和底盘能量数据包
struct ReceiveBuff
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t recovery_buff; // 恢复增益
    uint8_t cooling_buff; // 冷却增益
    uint8_t defence_buff; // 防御增益
    uint8_t vulnerability_buff; // 易伤增益
    uint16_t attack_buff; // 攻击增益
    uint8_t remaining_energy; // 剩余能量
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));
/********************************************************/
/* Send data                                            */
/********************************************************/

struct SendRobotCmdData
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx; // x轴速度
      float vy; // y轴速度
      float wz; // z轴角速度
    } __attribute__((packed)) speed_vector;

    struct
    {
      float roll; // 横滚角
      float pitch; // 俯仰角
      float yaw;  // 偏航角
      float leg_lenth; // 腿长
    } __attribute__((packed)) chassis;

    struct
    {
      float pitch; // 云台俯仰角度
      float yaw; // 云台水平角度
    } __attribute__((packed)) gimbal;

    struct
    {
      uint8_t fire; // 开火命令
      uint8_t fric_on; // 摩擦轮启动
    } __attribute__((packed)) shoot;

    struct
    {
      bool tracking; // 跟踪命令
    } __attribute__((packed)) tracking;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));

//数据流向：ROS2话题 → SendRobotCmdData结构体 → 串口 → 下位机

/********************************************************/
/* template                                             */
/********************************************************/
//1.从字节流转换为结构体,即从下位机传上来的data数据转换为packet结构体

//inline 将函数体直接嵌入到函数调用处，执行的时候不进行跳入跳出的操作，从而提高效率
//同时，写在头文件中，避免多次定义，被定义多次，但只保留一个定义

//reinterpret_cast<uint8_t *>(&data) 重新解释转换，将data的地址强制转换为uint8_t*类型

//copy的三个参数： 源数据起始地址，源数据结束地址（不含），目标位置的起始地址
template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  //&packet 获取packet的首地址，reinterpret_cast<uint8_t *>(&packet) 将packet的首地址强制转换为uint8_t*类型
  //std::copy 把 data里的一个个字节，直接“灌入”到 packet 的内存空间里。
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

//2.从结构体（data）转换为字节流packet
template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
