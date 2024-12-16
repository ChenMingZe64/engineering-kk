#include "grab.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "LK9025.h"
#include "bsp_gpio.h"

static Subscriber_t *grab_sub;
static Grab_Ctrl_Cmd_s grab_cmd_recv;
static DJIMotorInstance *grab_pitch, *grab_forward;
static LKMotorInstance *grab_roll;
static GPIOInstance *sucker_1, *sucker_2;

// tbd 确定限位值?
#define PITCH_OUTWARD_LIMIT 0.5
#define PITCH_INWARDE_LIMIT -0.5
#define STRECH_FRONT_LIMIT 0.5
#define STRECH_BACK_LIMIT -0.5
#define ROLL_CLOCKWISE_LIMIT 0.5
#define ROLL_COUNTER_CLOCKWISE_LIMIT -0.5

void GrabInit()
{
    Motor_Init_Config_s conf_pitch = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 15000,
                .DeadBand = 0.1,
            },
            .speed_PID = {
                .Kp = 2,
                .Ki = 100,
                .Kd = 0,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    grab_pitch = DJIMotorInit(&conf_pitch); 

    Motor_Init_Config_s conf_forward = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 15000,
                .DeadBand = 0.1,
            },
            .speed_PID = {
                .Kp = 2,
                .Ki = 100,
                .Kd = 0,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    grab_forward = DJIMotorInit(&conf_forward);

    Motor_Init_Config_s conf_roll = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 15000,
                .DeadBand = 0.1,
            },
            .speed_PID = {
                .Kp = 2,
                .Ki = 100,
                .Kd = 0,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK9025,
    };
    grab_roll = LKMotorInit(&conf_roll);

    // 吸盘继电器
    GPIO_Init_Config_s conf_sucker_1 = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_0,
        .pin_state = GPIO_PIN_RESET,
    };
    sucker_1 = GPIORegister(&conf_sucker_1);
    GPIO_Init_Config_s conf_sucker_2 = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_1,
        .pin_state = GPIO_PIN_RESET,
    };
    sucker_2 = GPIORegister(&conf_sucker_2);

    grab_sub = SubRegister("grab_cmd", sizeof(Grab_Ctrl_Cmd_s));
}

/* 机器人夹爪控制核心任务 */
void GrabTask()
{
    SubGetMessage(grab_sub, &grab_cmd_recv);

    if (grab_cmd_recv.sucker_flag) // 吸盘开关
    {
        GPIOSet(sucker_1);
        GPIOSet(sucker_2);
    }
    else
    {
        GPIOReset(sucker_1);
        GPIOReset(sucker_2);
    }

    // 切换夹爪运行速度
    static float speed_coef;
    if (grab_cmd_recv.grab_mode == GRAB_FAST)
        speed_coef = 1;
    else if (grab_cmd_recv.grab_mode == GRAB_SLOW)
        speed_coef = 0.4;

    // 夹爪Roll控制
    if (grab_cmd_recv.clock_wise_flag != 0)
        LKMotorSetRef(grab_roll, grab_cmd_recv.clock_wise_flag * speed_coef * 10); // tbd 调试夹爪roll转速
    else
    {
        // LKmotorOuterLoop(grab_roll,ANGLE_LOOP); // tbd 增加外层闭环切换
        LKMotorSetRef(grab_roll, grab_roll->measure.total_angle);
    }

    // 夹爪pitch控制
    if (grab_cmd_recv.pitch_inward_flag != 0)
        DJIMotorSetRef(grab_pitch, grab_cmd_recv.pitch_inward_flag * speed_coef * 10); // tbd 调试夹爪pitch转速
    else
    {
        DJIMotorOuterLoop(grab_pitch, ANGLE_LOOP);
        DJIMotorSetRef(grab_pitch, grab_pitch->measure.total_angle);
    }

    // 夹爪前伸
    if (grab_cmd_recv.forward_flag != 0)
        DJIMotorSetRef(grab_forward, grab_cmd_recv.forward_flag * speed_coef * 10); // tbd 调试夹爪前伸转速
    else
    {
        DJIMotorOuterLoop(grab_forward, ANGLE_LOOP);
        DJIMotorSetRef(grab_forward, grab_forward->measure.total_angle);
    }

    // 处理紧急情况
    if (grab_cmd_recv.grab_mode == GRAB_OFF)
    {
        DJIMotorStop(grab_pitch);
        DJIMotorStop(grab_forward);
        LKMotorStop(grab_roll);
        GPIOReset(sucker_1);
        GPIOReset(sucker_2);
    }
    else
    {
        DJIMotorEnable(grab_pitch);
        DJIMotorEnable(grab_forward);
        LKMotorEnable(grab_roll);
    }
}
