#ifndef chassisalgo_h
#define chassisalgo_h

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
// #include "matrix.h" //TODO 用armmatrix实现一遍基础运算
// #include "LQR.h"
// #include "VMC.h"
// #include "MPC.h"


#define LEG_MIN_Len  0.10f //machanical leg length min
#define LEG_MAX_Len  0.30f

typedef struct {
  float *phi1,*phi2,*phi3,*phi4,*phi5;
  float phi1_w, phi2_w, phi3_w, phi4_w, phi0_w;
  float Leg_Length;
} Leg_Param_t;

typedef enum {
  Chassis_Failsafe = 0,
  Chassis_Normal,
  Chassis_Debug,
} Chassis_Mode_e;

typedef struct {
  Chassis_Mode_e Chassis_Mode;
} Chassis_Status_t;



static Leg_Param_t legr,legl;

#ifdef __cplusplus
}
#endif

#endif