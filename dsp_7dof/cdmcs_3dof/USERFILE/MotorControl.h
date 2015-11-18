#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

extern const float FS;

/*------------**
** 结构体定义 **
**------------*/

typedef struct PID
{
    float y;		// 输出
    float u;		// 输入
    float u_pre;	// 上次输入
    float Kp;
    float Ki;
    float Kd;
    float sum;
} PID;

typedef struct SecondOrderIIR
{
    float a2;
    float a3;
    float b1;
    float b2;
    float b3;
    float y_pre;
    float y_prepre;
    float u_pre;
    float u_prepre;
} IIR2;

typedef struct MotorVelocityPara
{
    float accMax;		// 电机最大加速度(rpm * 3KHz)
    float velMax;		// 电机最大速度(rpm)
    float stopAcc;		// 电机制动加速度(rpm * 3KHz)
    float cmdVelPre;	// 上次的速度指令(rpm)
} MVP;

typedef struct MotorPara
{
    int32 posCnt;		// 位置计数器			RO
    int16 index;		// 电机编号			RO
    int16 feedbackInv;	// 反馈是否反向		WR
    int16 currentInv;	// 输出电流反向		WR
	float unfiltedVel;	// 滤波前速度			RO
	float filtedVel;	// 滤波后速度			RO
    float cableTension;	// 绳索张力			RO
    float current;		// 张力控制电流		RO
    PID pid;			// PID参数			
    MVP mvp;			// 速度参数			
	IIR2 iIIR;			// 速度输入滤波器
	IIR2 oIIR;			// 电流输出滤波器
} MP;

/*----------**
** 函数声明 **
**----------*/
void InitMVP(MVP *mvp, float accMax, float velMax, float stopAcc);
float ConstrainCmdVel(MVP *mvp, float cmdVel);
void InitPID(PID *sptr, float Kp, float Ki, float Kd);
void InitIIR2(IIR2 *iir, float a2, float a3, float b1, float b2, float b3);
float FilterIIR(IIR2 *pIIR, float u);
void SetMotorSpeed(MP* motor, float velCmd);
void SetMotorTorque(MP* motor, float tt);
void InitMotor(MP *motor, const PID *ppid, const MVP* pmvp,
               int16 index, int16 feedbackInv);
void UpdateAllState(void);
void ResetMotorPosCnt(void);

extern int16 JointEncCnt[8];	// 关节角反馈
extern int16 TensionCnt[8];		// 张力反馈
extern IIR2* pQzIIR;
extern IIR2* pQyIIR;
extern IIR2* pQxIIR;
extern  MP Motor[4];

#endif

// No more
