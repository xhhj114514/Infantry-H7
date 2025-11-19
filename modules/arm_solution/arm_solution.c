#include "arm_solution.h"
#include "matrix.h"
#include "math.h"


/**
 * @brief 逆运动学解算
 * @note  输入一个位姿向量(前三个元素为位置，后三个元素为ZYZ顺次欧拉角)，得到六轴机械臂的六轴转角
 * @param plan_data->arm_pos : 位姿向量
 * @param plan_data->arm_angle ：六轴转角
 */
int InverseK(Planning_Data_s *plan_data, uint8_t change_flag)
{
    static float angle_last[3];
    static float angle_now[3];
    static int round;
    static float xik[6];
    int turn_flag = 0;
    xik[0] = plan_data->arm_pos[0];
    xik[1] = plan_data->arm_pos[1];
    xik[2] = plan_data->arm_pos[2];
    xik[3] = plan_data->arm_pos[3];
    xik[4] = plan_data->arm_pos[4];
    xik[5] = plan_data->arm_pos[5];
    // 角度值转换为弧度制
    plan_data->arm_pos[3]=plan_data->arm_pos[3]*PI/180.0;
    plan_data->arm_pos[4]=plan_data->arm_pos[4]*PI/180.0;
    plan_data->arm_pos[5]=plan_data->arm_pos[5]*PI/180.0;
    
    // work frame
    float Xwf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xwf=[0; 0; 0; 0; 0; 0];
    
    // tool frame
    float Xtf[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Xtf=[0; 0; 0; 0; 0; 0];
    
    // work frame transformation matrix
    float Twf[16];
    pos2tranXYZ(Xwf, Twf, sizeof(Xwf)/sizeof(float), sizeof(Twf)/sizeof(float)); // Twf=pos2tran(Xwf);
    
    // tool frame transformation matrix
    float Ttf[16];
    pos2tranXYZ(Xtf, Ttf, sizeof(Xtf)/sizeof(float), sizeof(Ttf)/sizeof(float)); // Ttf=pos2tran(Xtf);
    
    // total transformation matrix
    float Twt[16];
    pos2tranXYZ(xik, Twt, 6, sizeof(Twf)/sizeof(float)); // Twt=pos2tran(plan_data->arm_pos);
    
    plan_data->arm_pos[0]=xik[0];

    plan_data->arm_pos[1] = xik[1];
    plan_data->arm_pos[2] = xik[2];
    plan_data->arm_pos[3] = xik[3];
    plan_data->arm_pos[4] = xik[4];
    plan_data->arm_pos[5] = xik[5];



    // find T06
    float inTwf[16], inTtf[16], Tw6[16], T06[16];
    invtran(Twf, inTwf, sizeof(Twf)/sizeof(float), sizeof(inTwf)/sizeof(float)); // inTwf=invtran(Twf);
    invtran(Ttf, inTtf, sizeof(Ttf)/sizeof(float), sizeof(inTtf)/sizeof(float)); // inTtf=invtran(Ttf);
    MatrixMultiply(Twt, inTtf, 4, 4, 4, Tw6, sizeof(Twt)/sizeof(float), sizeof(inTtf)/sizeof(float), sizeof(Tw6)/sizeof(float)); // Tw5=Twt*inTtf;
    MatrixMultiply(inTwf, Tw6, 4, 4, 4, T06, sizeof(inTwf)/sizeof(float), sizeof(Tw6)/sizeof(float), sizeof(T06)/sizeof(float)); // T05=inTwf*Tw5;
    
    float Px;
    float Py;
    float Pz;

    Px  = T06[0*4 + 3];
    Py  = T06[1*4 + 3];
    Pz  = T06[2*4 + 3];
    
    float a1,a2,a3,a4,a5,a6;
    float A,B,C,D,l1,l2;
    l1 = 300.0;  // a(2) = 300
    l2 = 349.5;  // d(4) = 340
    // l1 = 338.588;
    // l2 = 337.34033;
    //求解前三轴
    a1= atan2(Py,Px);
    a3= asin((Px*Px+Py*Py+Pz*Pz-l1*l1-l2*l2)/(2*l1*l2));
    A = l1 + l2*sin(a3);
    B = l2*cos(a3);
    C = l1 + l2*sin(a3);
    D = l2*cos(a3);
    a2= -atan2((B*sqrt(Px*Px+Py*Py)+Pz*C)/(A*C+B*D), (A*sqrt(Px*Px+Py*Py)-Pz*D)/(A*C+B*D));
    a3= PI/2-a3;

    //求解后三轴
    float T01[16] = {cos(a1),0,-sin(a1),0,sin(a1),0,cos(a1),0,0,-1,0,0,0,0,0,1};
    float T12[16] = {cos(a2),-sin(a2),0,300*cos(a2),sin(a2),cos(a2),0,300*sin(a2),0,0,1,0,0,0,0,1};
    float T23[16] = {cos(a3),0,-sin(a3),0,sin(a3),0,cos(a3),0,0,-1,0,0,0,0,0,1};
    float T34[16] = {0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1};
    float T02[16];
    float T03[16];
    float T04[16];
    MatrixMultiply(T01, T12, 4, 4, 4, T02, sizeof(T01)/sizeof(float), sizeof(T12)/sizeof(float), sizeof(T02)/sizeof(float)); 
    MatrixMultiply(T02, T23, 4, 4, 4, T03, sizeof(T02)/sizeof(float), sizeof(T23)/sizeof(float), sizeof(T03)/sizeof(float));
    MatrixMultiply(T03, T34, 4, 4, 4, T04, sizeof(T03)/sizeof(float), sizeof(T34)/sizeof(float), sizeof(T04)/sizeof(float));
    float inT03[16];
    float T36[16];
    invtran(T03, inT03, sizeof(T03)/sizeof(float), sizeof(inT03)/sizeof(float)); // inTtf=invtran(Ttf);
    MatrixMultiply(inT03, T06, 4, 4, 4, T36, sizeof(inT03)/sizeof(float), sizeof(T06)/sizeof(float), sizeof(T36)/sizeof(float));
    
    //选解
    float z1[3] = {T04[0*4 + 2],T04[1*4 + 2],T04[2*4 + 2]};
    float z2[3] = {T06[0*4 + 2],T06[1*4 + 2],T06[2*4 + 2]};
    float z12[3] = {z1[1]*z2[2]-z1[2]*z2[1],
                    z1[2]*z2[0]-z1[0]*z2[2],
                    z1[0]*z2[1]-z1[1]*z2[0]};
    if(z12[1] >= 0)
    {
        turn_flag = 1;
        a4 = atan2(T36[1*4 + 2],-T36[2*4 + 2])+PI;
        a5 = -acos(T36[0*4 + 2]);
        a6 = atan2(-T36[0*4+1],T36[0*4+0])+PI;
        if(a4 > PI)       a4 -= PI*2;
        else if(a4 < -PI) a4 += PI*2;
        if(a6 > PI)       a6 -= PI*2;
        else if(a6 < -PI) a6 += PI*2;
    }
    else
    {
        turn_flag = 0;
        a4 = atan2(T36[1*4 + 2],-T36[2*4 + 2]);
        a5 = acos(T36[0*4 + 2]);
        a6 = atan2(-T36[0*4+1],T36[0*4+0]);
    }

    if(isnan(a1)||isnan(a2)||isnan(a3)||isnan(a4)||isnan(a5)||isnan(a6))
    {
        a1 = 0;
        a2 = 0;
        a3 = 0;
        a4 = 0;
        a5 = 0;
        a6 = 0;
        plan_data->arm_pos[3]=plan_data->arm_pos[3]*180.0/PI;
        plan_data->arm_pos[4]=plan_data->arm_pos[4]*180.0/PI;
        plan_data->arm_pos[5]=plan_data->arm_pos[5]*180.0/PI;
        return 2;
    }
    else
    {
        if(change_flag == 0)        //重置
        {
            angle_last[0] = 0;
            angle_last[1] = 0;
            angle_last[2] = 0;
            angle_now[0] = a4;
            angle_now[1] = a5;
            angle_now[2] = a6;
            round = 0;
        }
        else if(change_flag == 1)   //适用于连续变化
        {
            float a4_ = a4;
            float a5_ = a5;
            float a6_ = a6;
            a4 = a4 + PI*round;
            a6 = a6 - PI*round;
            if(round == 1 || round == -1)  a5 = -a5;

            angle_last[0] = angle_now[0];
            angle_last[1] = angle_now[1];
            angle_last[2] = angle_now[2];
            angle_now[0] = a4;
            angle_now[1] = a5;
            angle_now[2] = a6;
            if(angle_now[0]-angle_last[0]>=PI/2)
            {
                round-=1;
                if(round < -1)  round = -1;
                a4 = a4_ + PI*round;
                a6 = a6_ - PI*round;
                if(round == 1 || round == -1)  a5 = -a5_;
            }
            if(angle_now[0]-angle_last[0]<-PI/2)   
            {
                round+=1;
                if(round > 1)   round = 1;
                a4 = a4_ + PI*round;
                a6 = a6_ - PI*round;
                if(round == 1 || round == -1)  a5 = -a5_;
            }
            angle_now[0] = a4;
            angle_now[1] = a5;
            angle_now[2] = a6;
        }
        
        plan_data->arm_angle[0] = a1*180/PI;
        plan_data->arm_angle[1] = a2*180/PI;
        plan_data->arm_angle[2] = a3*180/PI;
        plan_data->arm_angle[3] = a4*180/PI;
        plan_data->arm_angle[4] = a5*180/PI;
        plan_data->arm_angle[5] = a6*180/PI;
        plan_data->arm_pos[3]=plan_data->arm_pos[3]*180.0/PI;
        plan_data->arm_pos[4]=plan_data->arm_pos[4]*180.0/PI;
        plan_data->arm_pos[5]=plan_data->arm_pos[5]*180.0/PI;

        return turn_flag;
    }
}
/**
 * @brief 正运动学解算
 * @param Xfk : 位置向量 + 旋转矩阵3x3
 * @param Jfk ：六轴转角
 */
void ForwardK(float* Jfk, float* Xfk)
{
  // forward kinematics
  // input: Jfk - joints value for the calculation of the forward kinematics
  // output: Xfk - pos value for the calculation of the forward kinematics
  // Denavit-Hartenberg matrix
  float a1 = Jfk[0]*PI/180;
  float a2 = Jfk[1]*PI/180;
  float a3 = Jfk[2]*PI/180;
  float a4 = Jfk[3]*PI/180;
  float a5 = Jfk[4]*PI/180;
  float a6 = Jfk[5]*PI/180;
  float T01[16] = { cos(a1),       0, -sin(a1),    0,
                    sin(a1),       0,  cos(a1),    0,
                          0,      -1,        0,    0,
                          0,       0,        0,    1};
  float T12[16] = { cos(a2),-sin(a2),        0,    338.588*cos(a2),
                    sin(a2), cos(a2),        0,    338.588*sin(a2),
                          0,       0,        1,    0,
                          0,       0,        0,    1};
  float T23[16] = { cos(a3),       0, -sin(a3),    337.34033*cos(a3),
                    sin(a3),       0,  cos(a3),    337.34033*sin(a3),
                          0,      -1,        0,    0,
                          0,       0,        0,    1};
  float T34[16] = {       0,       0,        1,    0,
                          1,       0,        0,    0,
                          0,       1,        0,    0,
                          0,       0,        0,    1};
  float T45[16] = {-sin(a4),       0,  cos(a4),    0,
                    cos(a4),       0,  sin(a4),    0,
                          0,       1,        0,    0,
                          0,       0,        0,    1};
  float T56[16] = { cos(a5),       0, -sin(a5),    0,
                    sin(a5),       0,  cos(a5),    0,
                          0,      -1,        0,    0,
                          0,       0,        0,    1};
  float T67[16] = { cos(a6),-sin(a6),        0,    0,
                    sin(a6), cos(a6),        0,    0,
                          0,       0,        1,    0,
                          0,       0,        0,    1};

  float T02[16],T03[16],T04[16],T05[16],T06[16],T07[16];
  MatrixMultiply(T01, T12, 4, 4, 4, T02, sizeof(T01)/sizeof(float), sizeof(T12)/sizeof(float), sizeof(T02)/sizeof(float)); 
  MatrixMultiply(T02, T23, 4, 4, 4, T03, sizeof(T02)/sizeof(float), sizeof(T23)/sizeof(float), sizeof(T03)/sizeof(float));
  MatrixMultiply(T03, T34, 4, 4, 4, T04, sizeof(T03)/sizeof(float), sizeof(T34)/sizeof(float), sizeof(T04)/sizeof(float));
  MatrixMultiply(T04, T45, 4, 4, 4, T05, sizeof(T04)/sizeof(float), sizeof(T45)/sizeof(float), sizeof(T05)/sizeof(float));
  MatrixMultiply(T05, T56, 4, 4, 4, T06, sizeof(T05)/sizeof(float), sizeof(T56)/sizeof(float), sizeof(T06)/sizeof(float));
  MatrixMultiply(T06, T67, 4, 4, 4, T07, sizeof(T06)/sizeof(float), sizeof(T67)/sizeof(float), sizeof(T07)/sizeof(float));
  
  // calculate pos from transformation matrix
  // tran2posXYZ(T07, Xfk); // Xfk=tran2pos(Twt);
  // Xfk(4:6)=Xfk(4:6)/pi*180;
  Xfk[0] = T07[0*4 + 3];
  Xfk[1] = T07[1*4 + 3];
  Xfk[2] = T07[2*4 + 3];
  Xfk[3] = T07[0*4 + 0];
  Xfk[4] = T07[0*4 + 1];
  Xfk[5] = T07[0*4 + 2];
  Xfk[6] = T07[1*4 + 0];
  Xfk[7] = T07[1*4 + 1];
  Xfk[8] = T07[1*4 + 2];
  Xfk[9] = T07[2*4 + 0];
  Xfk[10]= T07[2*4 + 1];
  Xfk[11]= T07[2*4 + 2];
}


/**
 * @brief 逆运动学解算_基于旋转矩阵
 * @note  输入一个位姿向量(前三个元素为位置，后三个元素为ZYZ顺次欧拉角)，得到六轴机械臂的六轴转角
 * @param plan_data->arm_pos : 位姿向量：前三个为位置，后九个为旋转向量
 * @param plan_data->arm_angle ：六轴转角
 */
// int InverseK_matrix(float* plan_data->arm_pos, float* plan_data->arm_angle ,uint8_t change_flag)
// {
//   static float angle_last[3];
//   static float angle_now[3];
//   static int round;
//   int turn_flag = 0;
//   // inverse kinematics
//   // input: plan_data->arm_pos - pos value for the calculation of the inverse kinematics
//   // output: Jfk - joints value for the calculation of the inversed kinematics
  
//   // from deg to rad
//   // find T06
//   float T06[16];
//   T06[0*4 + 0] = plan_data->arm_pos[3];
//   T06[0*4 + 1] = plan_data->arm_pos[4];
//   T06[0*4 + 2] = plan_data->arm_pos[5];
//   T06[0*4 + 3] = plan_data->arm_pos[0];
//   T06[1*4 + 0] = plan_data->arm_pos[6];
//   T06[1*4 + 1] = plan_data->arm_pos[7];
//   T06[1*4 + 2] = plan_data->arm_pos[8];
//   T06[1*4 + 3] = plan_data->arm_pos[1];
//   T06[2*4 + 0] = plan_data->arm_pos[9];
//   T06[2*4 + 1] = plan_data->arm_pos[10];
//   T06[2*4 + 2] = plan_data->arm_pos[11];
//   T06[2*4 + 3] = plan_data->arm_pos[2];
//   T06[3*4 + 0] = 0;
//   T06[3*4 + 1] = 0;
//   T06[3*4 + 2] = 0;
//   T06[3*4 + 3] = 1;
  

//   float Px;
//   float Py;
//   float Pz;

//   Px  = T06[0*4 + 3];
//   Py  = T06[1*4 + 3];
//   Pz  = T06[2*4 + 3];
//   // joints variable
//   // plan_data->arm_angle=zeros(5,1);
//   // first joint
//   //出现多解时取正号，后续可优化
  
//   float a1,a2,a3,a4,a5,a6;
//   float A,B,C,D,l1,l2;
//   l1 = 338.588;
//   l2 = 337.34033;
//   //求解前三轴
//   a1= atan2(Py,Px);
//   a3= asin((Px*Px+Py*Py+Pz*Pz-l1*l1-l2*l2)/(2*l1*l2));
//   A = l1 + l2*sin(a3);
//   B = l2*cos(a3);
//   C = l1 + l2*sin(a3);
//   D = l2*cos(a3);
//   a2= -atan2((B*sqrt(Px*Px+Py*Py)+Pz*C)/(A*C+B*D), (A*sqrt(Px*Px+Py*Py)-Pz*D)/(A*C+B*D));
//   a3= PI/2-a3;
//   //求解后三轴
//   float T01[16] = {cos(a1),0,-sin(a1),0,sin(a1),0,cos(a1),0,0,-1,0,0,0,0,0,1};
//   float T12[16] = {cos(a2),-sin(a2),0,338.588*cos(a2),sin(a2),cos(a2),0,338.588*sin(a2),0,0,1,0,0,0,0,1};
//   float T23[16] = {cos(a3),0,-sin(a3),337.34033*cos(a3),sin(a3),0,cos(a3),337.34033*sin(a3),0,-1,0,0,0,0,0,1};
//   float T34[16] = {0,0,1,0,1,0,0,0,0,1,0,0,0,0,0,1};
//   float T02[16];
//   float T03[16];
//   float T04[16];
//   MatrixMultiply(T01, T12, 4, 4, 4, T02, sizeof(T01)/sizeof(float), sizeof(T12)/sizeof(float), sizeof(T02)/sizeof(float)); 
//   MatrixMultiply(T02, T23, 4, 4, 4, T03, sizeof(T02)/sizeof(float), sizeof(T23)/sizeof(float), sizeof(T03)/sizeof(float));
//   MatrixMultiply(T03, T34, 4, 4, 4, T04, sizeof(T03)/sizeof(float), sizeof(T34)/sizeof(float), sizeof(T04)/sizeof(float));
//   float inT03[16];
//   float T36[16];
//   invtran(T03, inT03, sizeof(T03)/sizeof(float), sizeof(inT03)/sizeof(float)); // inTtf=invtran(Ttf);
//   MatrixMultiply(inT03, T06, 4, 4, 4, T36, sizeof(inT03)/sizeof(float), sizeof(T06)/sizeof(float), sizeof(T36)/sizeof(float));
//   //选解
//   float z1[3] = {T04[0*4 + 2],T04[1*4 + 2],T04[2*4 + 2]};
//   float z2[3] = {T06[0*4 + 2],T06[1*4 + 2],T06[2*4 + 2]};
//   //z1和z2做叉积
//   float z12[3] = {z1[1]*z2[2]-z1[2]*z2[1],
//                   z1[2]*z2[0]-z1[0]*z2[2],
//                   z1[0]*z2[1]-z1[1]*z2[0]};
//   if(z12[1] >= 0)
//   {
//     turn_flag = 1;
//     //解2
//     a4 = atan2(T36[1*4 + 2],-T36[2*4 + 2])+PI;
//     a5 = -acos(T36[0*4 + 2]);
//     a6 = atan2(-T36[0*4+1],T36[0*4+0])+PI;
//     if(a4 > PI)       a4 -= PI*2;
//     else if(a4 < -PI) a4 += PI*2;
//     if(a6 > PI)       a6 -= PI*2;
//     else if(a6 < -PI) a6 += PI*2;
//   }
//   else
//   {
//     turn_flag = 0;
//     //解1
//     a4 = atan2(T36[1*4 + 2],-T36[2*4 + 2]);
//     a5 = acos(T36[0*4 + 2]);
//     a6 = atan2(-T36[0*4+1],T36[0*4+0]);
//   }

//   if(isnan(a1)||isnan(a2)||isnan(a3)||isnan(a4)||isnan(a5)||isnan(a6))
//   {
//     a1 = 0;
//     a2 = 0;
//     a3 = 0;
//     a4 = 0;
//     a5 = 0;
//     a6 = 0;

//     return 2;
//   }
//   else
//   {
//     if(change_flag == 0)        //重置
//     {
//         angle_last[0] = 0;
//         angle_last[1] = 0;
//         angle_last[2] = 0;
//         angle_now[0] = a4;
//         angle_now[1] = a5;
//         angle_now[2] = a6;
//         round = 0;
//     }
//     else if(change_flag == 1)   //适用于连续变化
//     {
//       float a4_ = a4;
//       float a5_ = a5;
//       float a6_ = a6;
//       a4 = a4 + PI*round;
//       a6 = a6 - PI*round;
//       if(round == 1 || round == -1)  a5 = -a5;

//       angle_last[0] = angle_now[0];
//       angle_last[1] = angle_now[1];
//       angle_last[2] = angle_now[2];
//       angle_now[0] = a4;
//       angle_now[1] = a5;
//       angle_now[2] = a6;
//       if(angle_now[0]-angle_last[0]>=PI/2)
//       {
//           round-=1;
//           if(round < -1)  round = -1;
//           a4 = a4_ + PI*round;
//           a6 = a6_ - PI*round;
//           if(round == 1 || round == -1)  a5 = -a5_;
//       }
//       if(angle_now[0]-angle_last[0]<-PI/2)   
//       {
//           round+=1;
//           if(round > 1)   round = 1;
//           a4 = a4_ + PI*round;
//           a6 = a6_ - PI*round;
//           if(round == 1 || round == -1)  a5 = -a5_;
//       }
//       angle_now[0] = a4;
//       angle_now[1] = a5;
//       angle_now[2] = a6;
//     }
    
//     // if(a4 > PI)       a4 -= PI*2;
//     // else if(a4 < -PI) a4 += PI*2;
//     // if(a6 > PI)       a6 -= PI*2;
//     // else if(a6 < -PI) a6 += PI*2;
//     plan_data->arm_angle[0] = a1*180/PI;
//     plan_data->arm_angle[1] = a2*180/PI;
//     plan_data->arm_angle[2] = a3*180/PI;
//     plan_data->arm_angle[3] = a4*180/PI;
//     plan_data->arm_angle[4] = a5*180/PI;
//     plan_data->arm_angle[5] = a6*180/PI;

//     return turn_flag;
//   }
// }

/**
 * @brief 正运动学解算(相机)
 * @param Xfk : 相机的位置向量 + 旋转矩阵3x3
 * @param Jfk ：六轴转角
 */
void ForwardKCamera(float* Jfk, float* Xfk)
{
  // forward kinematics
  // input: Jfk - joints value for the calculation of the forward kinematics
  // output: Xfk - pos value for the calculation of the forward kinematics
  // Denavit-Hartenberg matrix
  float a1 = Jfk[0]*PI/180;
  float a2 = Jfk[1]*PI/180;
  float a3 = Jfk[2]*PI/180;

  float T01[16] = { cos(a1),       0, -sin(a1),    0,
                    sin(a1),       0,  cos(a1),    0,
                          0,      -1,        0,    0,
                          0,       0,        0,    1};
  float T12[16] = { cos(a2),-sin(a2),        0,    338.588*cos(a2),
                    sin(a2), cos(a2),        0,    338.588*sin(a2),
                          0,       0,        1,    0,
                          0,       0,        0,    1};
  float T23[16] = { cos(a3),       0, -sin(a3),    99.39*cos(a3),
                    sin(a3),       0,  cos(a3),    99.39*sin(a3),
                          0,      -1,        0,    0,
                          0,       0,        0,    1};
  float T34[16] = {       1,       0,        0,    0,
                          0,       1,        0,    0,
                          0,       0,        1,    -70.9,
                          0,       0,        0,    1};
  float T45[16] = {       0,       0,        1,    0,
                          1,       0,        0,    51.5,
                          0,       1,        0,    0,
                          0,       0,        0,    1};

  float T02[16],T03[16],T04[16],T05[16];
  MatrixMultiply(T01, T12, 4, 4, 4, T02, sizeof(T01)/sizeof(float), sizeof(T12)/sizeof(float), sizeof(T02)/sizeof(float)); 
  MatrixMultiply(T02, T23, 4, 4, 4, T03, sizeof(T02)/sizeof(float), sizeof(T23)/sizeof(float), sizeof(T03)/sizeof(float));
  MatrixMultiply(T03, T34, 4, 4, 4, T04, sizeof(T03)/sizeof(float), sizeof(T34)/sizeof(float), sizeof(T04)/sizeof(float));
  MatrixMultiply(T04, T45, 4, 4, 4, T05, sizeof(T04)/sizeof(float), sizeof(T45)/sizeof(float), sizeof(T05)/sizeof(float));
  
  // calculate pos from transformation matrix
  // tran2posXYZ(T07, Xfk); // Xfk=tran2pos(Twt);
  // Xfk(4:6)=Xfk(4:6)/pi*180;
  Xfk[0] = T05[0*4 + 3];
  Xfk[1] = T05[1*4 + 3];
  Xfk[2] = T05[2*4 + 3];
  Xfk[3] = T05[0*4 + 0];
  Xfk[4] = T05[0*4 + 1];
  Xfk[5] = T05[0*4 + 2];
  Xfk[6] = T05[1*4 + 0];
  Xfk[7] = T05[1*4 + 1];
  Xfk[8] = T05[1*4 + 2];
  Xfk[9] = T05[2*4 + 0];
  Xfk[10]= T05[2*4 + 1];
  Xfk[11]= T05[2*4 + 2];

}

/**
 * @brief 正运动学解算(框)
 * @param Xfk1 ：相机的位置向量 + 旋转矩阵3x3
 * @param Xfk2 : 框相对于相机的位置向量 + 旋转矩阵3x3
 * @param Xfk_out : 框相对于底座的位置向量 + 旋转矩阵3x3

 */
void ForwardKDest(float* Xfk1, float* Xfk2,float* Xfk_out)
{
    float T1[16];
    T1[0*4 + 0] = Xfk1[3];
    T1[0*4 + 1] = Xfk1[4];
    T1[0*4 + 2] = Xfk1[5];
    T1[0*4 + 3] = Xfk1[0];
    T1[1*4 + 0] = Xfk1[6];
    T1[1*4 + 1] = Xfk1[7];
    T1[1*4 + 2] = Xfk1[8];
    T1[1*4 + 3] = Xfk1[1];
    T1[2*4 + 0] = Xfk1[9];
    T1[2*4 + 1] = Xfk1[10];
    T1[2*4 + 2] = Xfk1[11];
    T1[2*4 + 3] = Xfk1[2];
    T1[3*4 + 0] = 0;
    T1[3*4 + 1] = 0;
    T1[3*4 + 2] = 0;
    T1[3*4 + 3] = 1;

    float T2[16];
    T2[0*4 + 0] = Xfk2[3];
    T2[0*4 + 1] = Xfk2[4];
    T2[0*4 + 2] = Xfk2[5];
    T2[0*4 + 3] = Xfk2[0];
    T2[1*4 + 0] = Xfk2[6];
    T2[1*4 + 1] = Xfk2[7];
    T2[1*4 + 2] = Xfk2[8];
    T2[1*4 + 3] = Xfk2[1];
    T2[2*4 + 0] = Xfk2[9];
    T2[2*4 + 1] = Xfk2[10];
    T2[2*4 + 2] = Xfk2[11];
    T2[2*4 + 3] = Xfk2[2];
    T2[3*4 + 0] = 0;
    T2[3*4 + 1] = 0;
    T2[3*4 + 2] = 0;
    T2[3*4 + 3] = 1;

    float Tout[16];
    MatrixMultiply(T1, T2, 4, 4, 4, Tout, sizeof(T1)/sizeof(float), sizeof(T2)/sizeof(float), sizeof(Tout)/sizeof(float));

    Xfk_out[0] = Tout[0*4 + 3];
    Xfk_out[1] = Tout[1*4 + 3];
    Xfk_out[2] = Tout[2*4 + 3];
    Xfk_out[3] = Tout[0*4 + 0];
    Xfk_out[4] = Tout[0*4 + 1];
    Xfk_out[5] = Tout[0*4 + 2];
    Xfk_out[6] = Tout[1*4 + 0];
    Xfk_out[7] = Tout[1*4 + 1];
    Xfk_out[8] = Tout[1*4 + 2];
    Xfk_out[9] = Tout[2*4 + 0];
    Xfk_out[10]= Tout[2*4 + 1];
    Xfk_out[11]= Tout[2*4 + 2];

}
