#ifndef LK_motor_H
#define LK_motor_H


#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "memory.h"

#define LK_MOTOR_MX_CNT 8 // 1-32


#define LK_CAN_TXID_BASE 0x140
#define LK_CAN_RXID_BASE 0x180
#define CURRENT_SMOOTH_LPF 0.1f  //2804 0.01?
#define SPEED_SMOOTH_LPF 0.7f
#define REDUCTION_RATIO 1

#define LK_ECD2ANGLE (360.0f / 65536.0f) //18Bit?
#define LK_RAW2SPD      (1.0f)              //1dps/LSB
#define LK_MF_RAW2CUR   (33.0f / 4096.0f) 
#define LK_MG_RAW2CUR   (66.0f / 4096.0f) 
#define LK_TEMP_RAW2DEG (1.0f)          //  1Degree/LSB
#define LK_CURRENT_TORQUE_16T 0.32f  // 16T
#define LK_CURRENT_TORQUE_35T 0.81f  // 35T


enum LK_Motor_CMD {
    LK_READ_STATE_1 = 0x9A,
    LK_CLEAR_ERROR  = 0x9B,
    LK_READ_STATE_2 = 0X9C,
    LK_READ_STATE_3 = 0X9D,//NOT MS
    LK_MOTOR_START        = 0X88,
    LK_MOTOR_CLEARSTOP         = 0x80,
    LK_MOTOR_STOP   =0x81,
    LK_MOTOR_CLOSE_Torque  = 0xA1,
    LK_MOTOR_CLOSE_Speed   = 0xA2,
    LK_MOTOR_CLOSE_ACCRotate = 0xA3,
    LK_MOTOR_CLOSE_SpeedACCRotate = 0xA4,
};

typedef enum  {
    NORMAL = 0x00,
    LOWVOLTAGEPROTECT ,
    HIGHVOLTAGEPROTECT,
    DRIVEROVERTREMPPROTECT,
    MOTOROVERTEMP,
    MOTOROVERCURRENT,
    MOTORSHORTED,
    MOTORSTALL,
    MOTORSIGNALLOST,
}LK_Motor_Error;
//9025 V2
typedef struct 
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single; // 单圈角度 DEG
    float speed_rads;         // speed rad/s
    float real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float ACCangle;   // 总角度
    int32_t ACCrotation; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

typedef struct
{
    LKMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
    float *other_speed_feedback_ptr;
    float *other_current_feedback_ptr;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志
    LK_Motor_Error STATE; 

    CANInstance *motor_can_ins;

    DaemonInstance *daemon;

} LKMotorInstance;

/**
 * @brief 初始化LK电机
 *
 * @param config 电机配置
 * @return LKMotorInstance* 返回实例指针
 */
LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void LKMotorSetRef(LKMotorInstance *motor, float ref);

/**
 * @brief 为所有LK电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void LKMotorControl();

/**
 * @brief 停止LK电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void LKMotorStop(LKMotorInstance *motor);

/**
 * @brief 启动LK电机
 *
 * @param motor
 */
void LKMotorEnable(LKMotorInstance *motor);

uint8_t LKMotorIsOnline(LKMotorInstance *motor);

#endif // LK_motor_H
