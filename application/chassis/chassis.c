#include "chassis.h"
#include "chassisalgo.h"
#include "fdcan.h"

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体*/
/**IPC**/
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

// static DJIMotorInstance *foot_motor_l, *foot_motor_r; //后期打算换3508
static LKMotorInstance *foot_motor_l, *foot_motor_r;
static DMMotorInstance *hiplf, *hiplb, *hiprf, *hiprb;


/**
*@breif 左脚初始化函数
*@note  hfdcan1---1(foot)23(hip rf+rb)
 */
void LEGInit_L() {
  //     Motor_Init_Config_s DJIMotorInitConfig = {
  //       .can_init_config.can_handle = &hfdcan1,
  //       .controller_param_init_config =
  //           {
  //               .speed_PID =
  //                   {
  //                       .Kp = 10, // 4.5
  //                       .Ki = 0,  // 0
  //                       .Kd = 0,  // 0
  //                       .IntegralLimit = 3000,
  //                       .Improve = PID_Trapezoid_Intergral |
  //                       PID_Integral_Limit |
  //                                  PID_Derivative_On_Measurement,
  //                       .MaxOut = 12000,
  //                   },
  //               .current_PID =
  //                   {
  //                       .Kp = 0.5, // 0.4
  //                       .Ki = 0,   // 0
  //                       .Kd = 0,
  //                       .IntegralLimit = 3000,
  //                       .Improve = PID_Trapezoid_Intergral |
  //                       PID_Integral_Limit |
  //                                  PID_Derivative_On_Measurement,
  //                       .MaxOut = 15000,
  //                   },
  //           },
  //       .controller_setting_init_config =
  //           {
  //               .angle_feedback_source = MOTOR_FEED,
  //               .speed_feedback_source = MOTOR_FEED,
  //               .outer_loop_type = SPEED_LOOP,
  //               .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
  //           },
  //       .motor_type = M3508,
  //   };
  //   foot_motor_l = DJIMotorInit(&DJIMotorInitConfig);

  Motor_Init_Config_s LKMotorInitConfig = {
      .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 1,
      },
      .controller_param_init_config =
          {
          },
      .controller_setting_init_config = {},
      .motor_type = LK9025,
  };
  foot_motor_l = LKMotorInit(&LKMotorInitConfig);

  Motor_Init_Config_s DMMotorInitConfig = {
      .can_init_config =
          {
              .can_handle = &hfdcan1,
              .tx_id = 2,
          },
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .Kp = 10, // 4.5
                      .Ki = 0,  // 0
                      .Kd = 0,  // 0
                      .IntegralLimit = 100,
                      .Improve =
                          PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 100,
                  },
              .angle_PID =
                  {
                      .Kp = 10, // 4.5
                      .Ki = 0,  // 0
                      .Kd = 0,  // 0
                      .Improve =
                          PID_Integral_Limit | PID_Derivative_On_Measurement,
                  },

          },
      .controller_setting_init_config =
          {

          },
      .motor_type = 8009,
  };

//   hiplf = DMMotorInit(&DMMotorInitConfig);

//   DMMotorInitConfig.can_init_config.tx_id = 3;
//   hiplb = DMMotorInit(&DMMotorInitConfig);
}


/**
*@breif 右脚初始化函数
*@note  hfdcan2---4(foot)56(hip rf+rb)
 */
void LEGInit_R() {
  //     Motor_Init_Config_s DJIMotorInitConfig = {
  //       .can_init_config.can_handle = &hfdcan1,
  //       .controller_param_init_config =
  //           {
  //               .speed_PID =
  //                   {
  //                       .Kp = 10, // 4.5
  //                       .Ki = 0,  // 0
  //                       .Kd = 0,  // 0
  //                       .IntegralLimit = 3000,
  //                       .Improve = PID_Trapezoid_Intergral |
  //                       PID_Integral_Limit |
  //                                  PID_Derivative_On_Measurement,
  //                       .MaxOut = 12000,
  //                   },
  //               .current_PID =
  //                   {
  //                       .Kp = 0.5, // 0.4
  //                       .Ki = 0,   // 0
  //                       .Kd = 0,
  //                       .IntegralLimit = 3000,
  //                       .Improve = PID_Trapezoid_Intergral |
  //                       PID_Integral_Limit |
  //                                  PID_Derivative_On_Measurement,
  //                       .MaxOut = 15000,
  //                   },
  //           },
  //       .controller_setting_init_config =
  //           {
  //               .angle_feedback_source = MOTOR_FEED,
  //               .speed_feedback_source = MOTOR_FEED,
  //               .outer_loop_type = SPEED_LOOP,
  //               .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
  //           },
  //       .motor_type = M3508,
  //   };
  //   foot_motor_l = DJIMotorInit(&DJIMotorInitConfig);

  Motor_Init_Config_s LKMotorInitConfig = {
      .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 4,
      },
      .controller_param_init_config =
          {

          },
      .controller_setting_init_config = {},
      .motor_type = LK9025,
  };
//   foot_motor_r = LKMotorInit(&LKMotorInitConfig);

  Motor_Init_Config_s DMMotorInitConfig = {
      .can_init_config =
          {
              .can_handle = &hfdcan2,
              .tx_id = 5,
          },
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .Kp = 10, // 4.5
                      .Ki = 0,  // 0
                      .Kd = 0,  // 0
                      .IntegralLimit = 100,
                      .Improve =
                          PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .MaxOut = 100,
                  },
              .angle_PID =
                  {
                      .Kp = 10, // 4.5
                      .Ki = 0,  // 0
                      .Kd = 0,  // 0
                      .Improve =
                          PID_Integral_Limit | PID_Derivative_On_Measurement,
                  },

          },
      .controller_setting_init_config =
          {

          },
      .motor_type = 8009,
  };

//   hiprf = DMMotorInit(&DMMotorInitConfig);

//   DMMotorInitConfig.can_init_config.tx_id = 6;
//   hiprb = DMMotorInit(&DMMotorInitConfig);
}

static void Failsafe()
{

}

void ChassisInit()
{
    LEGInit_L();
    LEGInit_R();
}

static void ChassisStateSet(Leg_Param_t *leg_param , Chassis_Status_t *set_chassis_status)
{
    switch(set_chassis_status->Chassis_Mode)
    {
        case Chassis_Failsafe:
        {

        }
        break;
        case Chassis_Normal:
        {

        }
        break;
        case Chassis_Debug:
        {

        }
        break;
        default: 
        Failsafe();
        break;
    }
}

/* 机器人底盘控制核心任务 */
void ChassisTask() {
  SubGetMessage(chassis_sub, &chassis_cmd_recv);
  PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
