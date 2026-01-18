#ifndef chassisalgo_h
#define chassisalgo_h


#ifdef __cplusplus
extern "C" {
#endif


#include "arm_math.h"

#include "robot_def.h"
#include "dji_motor.h"
#include "DM_motor.h"
#include "LK_motor.h"
#include "motor_def.h"
#include "controller.h"

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32



#define LEG_MIN_Len  0.10f //machanical leg length min
#define LEG_MAX_Len  0.30f

typedef struct {
  float phi1,phi2,phi3,phi4,phi5;
  float phi1_w, phi2_w, phi3_w, phi4_w, phi0_w;
  float pointB[2];
  float pointD[2];
  float pointC[2];
  float Leg_Length;
}VMC_t;



typedef struct {
  VMC_t legx;
  PIDInstance LegLength_PID;
  PIDInstance LegAlingn_PID;
  LKMotorInstance* Wheel[2];
  DJIMotorInstance* Wheel_DJI[2];
  DMMotorInstance* HIP[2];
  float TWheel;
  float THip; 
} Leg_Param_t;





static Leg_Param_t legr,legl;

#ifdef __cplusplus
}
#endif

#endif