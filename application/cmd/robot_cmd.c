// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "buzzer.h"
#include "referee_UI.h"
#include "referee_task.h"

#include "bsp_dwt.h"


static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Subscriber_t *chassis_imu_sub;

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Chassis_IMU_Data_s chassis_imu_data;

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回


static Robot_Status_e robot_state; // 机器人整体工作状态
static DataLebel_t DataLebel;



void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_imu_sub = SubRegister("chassis_imu", sizeof(Chassis_IMU_Data_s));

}



static void ChassisRC()
{
    chassis_cmd_send.vx = 30.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =-30.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ZERO_FORCE;

    }
}





/**
 * @brief 停止
 */
static void AnythingStop()
{
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
}

/**
 * @brief 控制量及模式设置
 *
 */
static void ControlDataDeal()
{
    if(chassis_imu_data.pit <= CHASSIS_MAX_ANG && chassis_imu_data.pit >= -CHASSIS_MAX_ANG)
    {
        if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
        {
            ChassisRC();
        }
        else if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
        {
            AnythingStop();
        }
    }
    else 
    {
        AnythingStop();
    }

}


/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    SubGetMessage(chassis_imu_sub, (void *)&chassis_imu_data);
    ControlDataDeal();
}
