#include "gimbal.h"
#include "robot_def.h"
#include "servo_motor.h"
#include "LK9025.h"
#include "message_center.h"
#include "general_def.h"

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static ServoInstance *gimbal_pitch;
static LKMotorInstance *gimbal_yaw;

void GimbalInit()
{

    // 云台电机初始化
    Motor_Init_Config_s yaw_config = {
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
        .motor_type = LK9025,
    };
    gimbal_yaw = LKMotorInit(&yaw_config);

    // tbd: 云台舵机初始化
    Servo_Init_Config_s pitch_config = {
        .Channel = TIM_CHANNEL_1,
        .htim = &htim4,
        .Servo_Angle_Type = Free_Angle_mode,
        .Servo_type = Servo180,
    };
    gimbal_pitch = ServoInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_OFF:
        LKMotorStop(gimbal_yaw);
        // ServoStop() tbd
        break;
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        LKMotorSetRef(gimbal_yaw, gimbal_cmd_recv.yaw);
        // ServoSetAngle(gimbal_pitch, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}