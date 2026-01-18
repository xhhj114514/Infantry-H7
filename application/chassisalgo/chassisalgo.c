#include "chassisalgo.h"
#include "controller.h"
#include "robot_def.h"


static mat *chassisX;
static float X_Data[6*1] = {0.0};// x xhat theta w alpha d_alpha
static mat *Q;//状态权重矩阵
static float Q_Data[6*6];
static mat *R;//输入权重矩阵
static float R_Data[2];
static mat *U;//OUTPUT TorWHEEL &&TorHIP
static float U_Data[1*2];





/********POLY PARAMS NEED TO  USE CORDIC**********/
static float Poly_Coef[12][4]=                              {	{0.0,0.0,0.0,0.0},//a11
																{0.0,0.0,0.0,0.0},//a12
																{0.0,0.0,0.0,0.0},//a13
																{0.0,0.0,0.0,0.0},//a14
																{0.0,0.0,0.0,0.0},//a15
																{0.0,0.0,0.0,0.0},//a16

																{0.0,0.0,0.0,0.0},//a21
																{0.0,0.0,0.0,0.0},//a22
																{0.0,0.0,0.0,0.0},//a23
																{0.0,0.0,0.0,0.0},//a24
															   {0.0,0.0,0.0,0.0},//a25
															   {0.0,0.0,0.0,0.0} //a26
                                                            };

static float K[2][6];

static float PolyFit(float a[4],float x)
{
    return a[0] + a[1]*x + a[2]*x*x + a[3]*x*x*x;
}



Leg_Param_t Legl;
Leg_Param_t Legr;

PID_Init_Config_s PIDCommonConfig;

// VMC->LQR->Torque


void Algo_Init()
{
    Matrix_Init(chassisX,6, 1,X_Data);
    Matrix_Init(Q,6, 6,Q_Data);
    Matrix_Init(R,2, 1,R_Data);
    Matrix_Init(U,1, 2,U_Data);
    legl.LegLength_PID.Measure = 0.0;
    legl.LegLength_PID.Ref = 0.0;
    PIDInit(&legl.LegLength_PID, &PIDCommonConfig);
    PIDInit(&legl.LegAlingn_PID, &PIDCommonConfig);
    //change settings
    PIDInit(&legr.LegLength_PID, &PIDCommonConfig);
    PIDInit(&legr.LegAlingn_PID, &PIDCommonConfig);
}



void Fit_K(VMC_t* vmc)
{
    for(int i=0;i<6;i++)
    {
        K[0][i] = PolyFit(Poly_Coef[i],vmc->Leg_Length);
        K[1][i] = PolyFit(Poly_Coef[i+6],vmc->Leg_Length);
    }
}

void LQR_Cal(Leg_Param_t* leg,Chassis_Data_Algo_t* chassis)
{
    leg->TWheel = ( K[0][0]*chassis->x +
                    K[0][1]*chassis->x_hat +
                    K[0][2]*chassis->theta +
                    K[0][3]*chassis->w +
                    K[0][4]*chassis->alpha +
                    K[0][5]*chassis->d_alpha);//Wheel
    leg->THip = ( K[1][0]*chassis->x +
                  K[1][1]*chassis->x_hat +
                  K[1][2]*chassis->theta +
                  K[1][3]*chassis->w +
                  K[1][4]*chassis->alpha +
                  K[1][5]*chassis->d_alpha);//Hip
}

void VMC_Cal(VMC_t* vmc)
{

}


