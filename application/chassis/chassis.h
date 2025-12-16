#ifndef CHASSIS_H
#define CHASSIS_H

#pragma once

/*********MOTOR******/
#include "dji_motor.h"
#include "DM_motor.h"
#include "LK_motor.h"
#include "motor_def.h"


/*****MECHANIC&&MATH*******/
#include "robot_def.h"
#include "arm_math.h"
#include "general_def.h"


/*********Funtions Pack************/
// #include "super_cap.h"
#include "chassisalgo.h"
#include "bsp_dwt.h"
#include "message_center.h"

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // CHASSIS_H