//
// Created by ysl on 2025/12/24.
//

#ifndef COD_MESSAGE_H
#define COD_MESSAGE_H

#include <stdint.h>

/* ================= 0x0001 ================= */
typedef struct __attribute__((packed))
{
    uint8_t  game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

/* ================= 0x0002 ================= */
typedef struct __attribute__((packed))
{
    uint8_t winner;
} game_result_t;

/* ================= 0x0003 ================= */
typedef struct __attribute__((packed))
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_reserved;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;

    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_reserved;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

/* ================= 0x0101 ================= */
typedef struct __attribute__((packed))
{
    uint32_t zone_status;
} event_data_t;

/* ================= 0x0104 ================= */
typedef struct __attribute__((packed))
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

/* ================= 0x0201 ================= */
typedef struct __attribute__((packed))
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;

    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

/* ================= 0x0203 ================= */
typedef struct __attribute__((packed))
{
    float x;
    float y;
    float angle;
} robot_pos_t;

/* ================= 0x0206 ================= */
typedef struct __attribute__((packed))
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/* ================= 0x0207 ================= */
typedef struct __attribute__((packed))
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float   initial_speed;
} shoot_data_t;

/* ================= 0x0208 ================= */
typedef struct __attribute__((packed))
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* ================= 0x020B ================= */
typedef struct __attribute__((packed))
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
} ground_robot_position_t;

/* ================= 0x020D ================= */
typedef struct __attribute__((packed))
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

#endif // COD_MESSAGE_H
