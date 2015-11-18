#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

extern const float FS;

/*------------**
** �ṹ�嶨�� **
**------------*/

typedef struct PID
{
    float y;		// ���
    float u;		// ����
    float u_pre;	// �ϴ�����
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
    float accMax;		// ��������ٶ�(rpm * 3KHz)
    float velMax;		// �������ٶ�(rpm)
    float stopAcc;		// ����ƶ����ٶ�(rpm * 3KHz)
    float cmdVelPre;	// �ϴε��ٶ�ָ��(rpm)
} MVP;

typedef struct MotorPara
{
    int32 posCnt;		// λ�ü�����			RO
    int16 index;		// ������			RO
    int16 feedbackInv;	// �����Ƿ���		WR
    int16 currentInv;	// �����������		WR
	float unfiltedVel;	// �˲�ǰ�ٶ�			RO
	float filtedVel;	// �˲����ٶ�			RO
    float cableTension;	// ��������			RO
    float current;		// �������Ƶ���		RO
    PID pid;			// PID����			
    MVP mvp;			// �ٶȲ���			
	IIR2 iIIR;			// �ٶ������˲���
	IIR2 oIIR;			// ��������˲���
} MP;

/*----------**
** �������� **
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

extern int16 JointEncCnt[8];	// �ؽڽǷ���
extern int16 TensionCnt[8];		// ��������
extern IIR2* pQzIIR;
extern IIR2* pQyIIR;
extern IIR2* pQxIIR;
extern  MP Motor[4];

#endif

// No more
