/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

static Publisher_t *chassis_pub;  // 用于发布底盘的数据
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令

static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
static DJIMotorInstance *rescue_l, *rescue_r;
static DJIMotorInstance *flip_f, *flip_b;

/* 用于自旋变速策略的时间变量,后续考虑查表加速 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy, chassis_wz; // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;         // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5,
                .Ki = 60,
                .Kd = 0,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.02,
                .Kd = 0,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4,
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3,
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2,
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    Motor_Init_Config_s rescue_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 150000,
            },
            .current_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 150000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    rescue_config.can_init_config.tx_id = 5;
    rescue_l = DJIMotorInit(&rescue_config);
    rescue_config.can_init_config.tx_id = 6;
    rescue_r = DJIMotorInit(&rescue_config);

    Motor_Init_Config_s flip_conf = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 150000,
            },
            .current_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 150000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    flip_conf.can_init_config.tx_id = 7;
    flip_f = DJIMotorInit(&flip_conf);
    flip_conf.can_init_config.tx_id = 8;
    flip_b = DJIMotorInit(&flip_conf);

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy - 0.5 * chassis_wz * LF_CENTER;
    vt_rf = chassis_vx - chassis_vy - 0.5 * chassis_wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy - 0.5 * chassis_wz * LB_CENTER;
    vt_rb = -chassis_vx - chassis_vy - 0.5 * chassis_wz * RB_CENTER;
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */

/* 机器人底盘控制核心任务 */

void ChassisTask()
{
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_FREE_MODE:
        chassis_vx = chassis_cmd_recv.vx;
        chassis_vy = chassis_cmd_recv.vy;
        break;
    case CHASSIS_ROTATE:
        chassis_vx = chassis_vy = 0;
        chassis_wz = chassis_cmd_recv.wz;
        break;
    }

    // 设置翻矿电机旋转与否
    static float flip_speed = 0;
    if (chassis_cmd_recv.flip_mode == FLIP_ON)
        flip_speed = 10000;
    else
        flip_speed = 0;
    DJIMotorSetRef(flip_f, flip_speed);
    DJIMotorSetRef(flip_b, flip_speed);

    // 设置救援电机是否旋转和旋转方向
    static float rescue_speed = 0;
    if (chassis_cmd_recv.rescue_mode == RESCUE_ON && chassis_cmd_recv.rescue_direction != 0)
        rescue_speed = 10000 * chassis_cmd_recv.rescue_direction;
    else
        rescue_speed = 0;
    DJIMotorSetRef(rescue_l, rescue_speed);
    DJIMotorSetRef(rescue_r, rescue_speed);

    MecanumCalculate();
    
    // if (chassis_cmd_recv.chassis_mode == CHASSIS_OFF)
    // { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    //     DJIMotorStop(motor_lf);
    //     DJIMotorStop(motor_rf); 
    //     DJIMotorStop(motor_lb);
    //     DJIMotorStop(motor_rb);
    //     DJIMotorStop(rescue_l);
    //     DJIMotorStop(rescue_r);
    //     DJIMotorStop(flip_f);
    //     DJIMotorStop(flip_b);
    // }
    // else
    // { // 正常工作
    //     DJIMotorEnable(motor_lf);
    //     DJIMotorEnable(motor_rf);
    //     DJIMotorEnable(motor_lb);
    //     DJIMotorEnable(motor_rb);
    //     DJIMotorEnable(rescue_l);
    //     DJIMotorEnable(rescue_r);
    //     DJIMotorEnable(flip_f);
    //     DJIMotorEnable(flip_b);
    // }

    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
    DJIMotorEnable(rescue_l);
    DJIMotorEnable(rescue_r);
    DJIMotorEnable(flip_f);
    DJIMotorEnable(flip_b);

    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}