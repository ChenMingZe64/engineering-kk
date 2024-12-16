/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板
// #define BALANCE_BOARD //启用平衡底盘,则默认双板且当前板位底盘,目前不支持!请勿使用!

// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

typedef enum
{
    GIMBAL_OFF = 0, // 电流零输入
    GIMBAL_FREE_MODE,
} gimbal_mode_e;

// 底盘模式设置
typedef enum
{
    CHASSIS_OFF = 0,   // 电流零输入
    CHASSIS_ROTATE,    // 小陀螺模式
    CHASSIS_FREE_MODE, // 不跟随，允许全向平移
} chassis_mode_e;

typedef enum
{
    GRAB_OFF = 0, // 电流零输入
    GRAB_FAST,
    GRAB_SLOW,
} grab_mode_e;

typedef enum
{
    LIFT_OFF = 0,
    LIFT_SLOW,
    LIFT_FAST,
} lift_mode_e;

typedef enum
{
    RESCUE_OFF = 0,
    RESCUE_ON,
} rescue_mode_e;

typedef enum
{
    FLIP_OFF = 0,
    FLIP_ON,
} flip_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx; // 前进方向速度
    float vy; // 横移方向速度
    float wz; // 旋转速度
    chassis_mode_e chassis_mode;
    flip_mode_e flip_mode;
    rescue_mode_e rescue_mode;
    uint8_t rescue_direction;
} Chassis_Ctrl_Cmd_s;

typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;
typedef struct
{
    grab_mode_e grab_mode;
    uint8_t clock_wise_flag;
    uint8_t pitch_inward_flag;
    uint8_t forward_flag;
    uint8_t sucker_flag;
} Grab_Ctrl_Cmd_s;

typedef struct
{
    lift_mode_e lift_mode;
    uint8_t up_flag;
} Lift_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{

} Chassis_Upload_Data_s;

typedef struct
{
    /* data */
} Gimbal_Upload_Data_s;

typedef struct
{
    float dist_lift;
} Lift_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H