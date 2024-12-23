#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"

#include "grab.h"
#include "robot_cmd.h"
#include "lift.h"
#include "chassis.h"
#include "gimbal.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#warning check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!
#endif // !ROBOT_DEF_PARAM_WARNING

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();

    BSPInit();

    RobotCMDInit();
    //GimbalInit();
    //GrabInit();
    //LiftInit();
    ChassisInit();

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    RobotCMDTask();
    // GrabTask();
    // LiftTask();
    ChassisTask();
    // GimbalTask();
}
