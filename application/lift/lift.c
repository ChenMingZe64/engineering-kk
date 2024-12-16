#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "arm_math.h"
#include "tfminiplus.h"

static DJIMotorInstance *lift_l, *lift_r;
static TFMiniInstance *lift_ranger;

static Publisher_t *lift_pub;
static Subscriber_t *lift_sub;
static Lift_Ctrl_Cmd_s lift_cmd_recv;
static Lift_Upload_Data_s lift_data_send;

#define LIFT_DOWN_LIMIT 0.03f
#define LIFT_UP_LIMIT 0.60f

void LiftInit()
{
    Motor_Init_Config_s lift_conf = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5,
                .Ki = 30,
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
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M3508,
    };
    lift_conf.can_init_config.tx_id = 3;
    lift_l = DJIMotorInit(&lift_conf);

    lift_conf.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    lift_conf.can_init_config.tx_id = 4,
    lift_r = DJIMotorInit(&lift_conf);

    lift_ranger = TFMiniRegister(&hi2c2); // 待定

    lift_pub = PubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    lift_sub = SubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
}

void LiftTask()
{
    SubGetMessage(lift_sub, &lift_cmd_recv);

    static uint8_t dist_freq = 0;
    if (dist_freq++ % 10 == 0) // 以20Hz的频率测距
        lift_data_send.dist_lift = TFMiniGetDistance(lift_ranger);

    if (lift_cmd_recv.lift_mode == LIFT_OFF)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(lift_l);
        DJIMotorStop(lift_r);
    }
    else
    { // 正常工作
        DJIMotorEnable(lift_l);
        DJIMotorEnable(lift_r);
    }

    static float lift_speed = 0;
    if (lift_cmd_recv.up_flag != 0) // 电机转速控制
    {
        if (lift_cmd_recv.lift_mode == LIFT_FAST)
            lift_speed = 1000;
        else if (lift_cmd_recv.lift_mode == LIFT_SLOW)
            lift_speed = 500;
        DJIMotorSetRef(lift_l, lift_speed * lift_cmd_recv.up_flag);
        DJIMotorSetRef(lift_r, lift_speed * lift_cmd_recv.up_flag);
    }
    else // 保持位置,切换到位置环
    {
        DJIMotorOuterLoop(lift_l, ANGLE_LOOP);
        DJIMotorOuterLoop(lift_r, ANGLE_LOOP);
        DJIMotorSetRef(lift_l, lift_l->measure.total_angle);
        DJIMotorSetRef(lift_r, lift_l->measure.total_angle);
    }

    PubPushMessage(lift_pub, (void *)&lift_data_send);
}
