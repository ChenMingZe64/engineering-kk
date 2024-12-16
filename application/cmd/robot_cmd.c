#include "robot_def.h"
#include "robot_cmd.h"
#include "remote_control.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "referee_task.h"
#include "remote_control.h"

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;

static Publisher_t *lift_cmd_pub;          // 抬升控制消息发布者
static Subscriber_t *lift_feed_sub;        // 抬升反馈信息订阅者
static Lift_Ctrl_Cmd_s lift_cmd_send;      // 传递给抬升的控制信息
static Lift_Upload_Data_s lift_fetch_data; // 从抬升获取的反馈信息

static Publisher_t *gimbal_cmd_pub;       // 云台控制消息发布者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息

static Grab_Ctrl_Cmd_s grab_cmd_send; // 发送给夹爪的数据
static Publisher_t *grab_cmd_pub;     // 夹爪控制消息发布者

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
static Referee_Interactive_info_t UI_data;

static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    //Referee_Interactive_init(&huart6, &UI_data);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    grab_cmd_pub = PubRegister("grab_cmd", sizeof(Grab_Ctrl_Cmd_s));
    lift_cmd_pub = PubRegister("lift_cmd", sizeof(Lift_Ctrl_Cmd_s));
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    lift_feed_sub = SubRegister("lift_feed", sizeof(Lift_Upload_Data_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/* 控制输入为遥控器(调试时)的模式和控制量设置 */
static void RemoteControlSet()
{
    // 左边开关在[下]位置, 左侧摇杆控制云台转动,右侧控制底盘运动,拨轮控制救援
    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        // 清零控制量,保持hold不动
        chassis_cmd_send.flip_mode = FLIP_OFF;
        grab_cmd_send.clock_wise_flag = 0;
        grab_cmd_send.forward_flag = 0;
        grab_cmd_send.pitch_inward_flag = 0;
        lift_cmd_send.up_flag = 0;

        // tbd 添加限位
        gimbal_cmd_send.pitch += rc_data[TEMP].rc.rocker_l1 * 0.05;
        gimbal_cmd_send.yaw += rc_data[TEMP].rc.rocker_l_ * 0.05;

        chassis_cmd_send.vx = rc_data[TEMP].rc.rocker_r1 * 0.05;
        chassis_cmd_send.vy = rc_data[TEMP].rc.rocker_r_ * 0.05;

        if (rc_data[TEMP].rc.dial != 0)
        {
            if (rc_data[TEMP].rc.dial > 0)
                chassis_cmd_send.rescue_direction = 1;
            else
                chassis_cmd_send.rescue_mode = -1;
        }
        else
            chassis_cmd_send.rescue_mode = 0;
    } // 左侧开关在[中]位置, 左侧摇杆控制夹爪pitch和roll,右侧控制抬升和前伸,拨轮控制翻矿
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.vx = chassis_cmd_send.vy = 0; // 清零速度,保持云台位置不动
        chassis_cmd_send.rescue_direction = 0;         // 保持救援不动

        if (rc_data[TEMP].rc.rocker_l1 != 0)
        {
            if (rc_data[TEMP].rc.rocker_l1 > 0)
                grab_cmd_send.pitch_inward_flag = 1;
            else
                grab_cmd_send.pitch_inward_flag = -1;
        }
        else
            grab_cmd_send.pitch_inward_flag = 0;

        if (rc_data[TEMP].rc.rocker_l_ != 0)
        {
            if (rc_data[TEMP].rc.rocker_l_ > 0)
                grab_cmd_send.clock_wise_flag = 1;
            else
                grab_cmd_send.clock_wise_flag = -1;
        }
        else
            grab_cmd_send.clock_wise_flag = 0;

        if (rc_data[TEMP].rc.rocker_r_ != 0)
        {
            if (rc_data[TEMP].rc.rocker_r_ > 0)
                grab_cmd_send.forward_flag = 1;
            else
                grab_cmd_send.forward_flag = -1;
        }
        else
            grab_cmd_send.forward_flag = 0;

        if (rc_data[TEMP].rc.rocker_r1 != 0)
        {
            if (rc_data[TEMP].rc.rocker_r1 > 0)
                lift_cmd_send.up_flag = 1;
            else
                lift_cmd_send.lift_mode = -1;
        }
        else
            lift_cmd_send.lift_mode = 0;

        if (rc_data[TEMP].rc.dial != 0)
            chassis_cmd_send.flip_mode = FLIP_ON;
        else
            chassis_cmd_send.flip_mode = FLIP_OFF;
    }

    // 右侧开关为[下]:吸盘关闭,右侧开关为[中]:吸盘打开
    if (switch_is_down(rc_data[TEMP].rc.switch_right))
        grab_cmd_send.sucker_flag = 0;
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
        grab_cmd_send.sucker_flag = 1;
}

/* 输入为键鼠时模式和控制量设置 */
static void MouseKeySet()
{
    // 待添加键鼠控制
    // ...
}


/* 先暂停异常状态检测 */
static void EmergencyHandler()
{
    // if (!RemoteControlIsOnline())
    // { // 遥控器断开,机器人停止
    //     lift_cmd_send.lift_mode = LIFT_OFF;
    //     grab_cmd_send.grab_mode = GRAB_OFF;
    //     chassis_cmd_send.chassis_mode = CHASSIS_OFF;
    //     chassis_cmd_send.flip_mode = FLIP_OFF;
    //     chassis_cmd_send.rescue_mode = RESCUE_OFF;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_OFF;
    //     robot_state = ROBOT_STOP;
    // }
    // else if (RemoteControlIsOnline() && switch_is_up(rc_data[TEMP].rc.switch_left))
    // { // 左侧开关在[上]位置恢复为启动或慢速模式
    //     chassis_cmd_send.chassis_mode = CHASSIS_FREE_MODE;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
    //     lift_cmd_send.lift_mode = LIFT_SLOW;
    //     grab_cmd_send.grab_mode = GRAB_SLOW;
    //     chassis_cmd_send.flip_mode = FLIP_ON;
    //     chassis_cmd_send.rescue_mode = RESCUE_ON;
    //     robot_state = ROBOT_READY;
    // }
}



void RobotCMDTask()
{
    SubGetMessage(lift_feed_sub, &lift_fetch_data);
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);

    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    EmergencyHandler();

    PubPushMessage(lift_cmd_pub, &lift_cmd_send);
    PubPushMessage(grab_cmd_pub, &grab_cmd_send);
    PubPushMessage(chassis_cmd_pub, &chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_send);
}
