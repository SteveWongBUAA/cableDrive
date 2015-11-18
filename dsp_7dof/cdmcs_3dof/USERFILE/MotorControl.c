#include "MotorControl.h"
#include "EPwmSetup.h"
#include "FPGA.h"

// 速度环积分分离点, 误差大于该值时，采用PD，小于该值时采用PID。
const float VLOOP_CONTROL_SP = 300.0;	
const float C2R = 175.78;	// 3000 * 60 / 1024		// cps to rpm (Count per second to revolution per minute)
const float FS = 3000.0;		// 采样频率3K
/* 电机的电流换算为绳索张力的系数 (Kg/A) */
const float CURRENT_TENSION_FACTOR = 1 / (10.98 / 10.0 * 203.92 / 9.8);

int16 MotorEncCnt[7];	// 电机反馈
int16 JointEncCnt[8];	// 关节角反馈
int16 TensionCnt[8];	// 张力反馈

IIR2 QzIIR;
IIR2 QyIIR;
IIR2 QxIIR;

//IIR2 OutIIR;

IIR2* pQzIIR = &QzIIR;
IIR2* pQyIIR = &QyIIR;
IIR2* pQxIIR = &QxIIR;

MP Motor[4];

// 初始化电机速度参数
void InitMVP(MVP *mvp, float accMax, float velMax, float stopAcc)
{
	mvp->accMax = accMax;
	mvp->velMax = velMax;
	mvp->stopAcc = stopAcc;
	mvp->cmdVelPre = 0.0;
}

// 速度限制
float ConstrainCmdVel(MVP *mvp, float cmdVel)
{
	// 加速度约束
	if((cmdVel - mvp->cmdVelPre) > mvp->accMax)
		cmdVel = mvp->cmdVelPre + mvp->accMax;
	else if((cmdVel - mvp->cmdVelPre) < -mvp->accMax)
		cmdVel = mvp->cmdVelPre - mvp->accMax;
	// 速度约束
	if(cmdVel > mvp->velMax)
		cmdVel = mvp->velMax;
	else if(cmdVel < -mvp->velMax)
		cmdVel = -mvp->velMax;

	mvp->cmdVelPre = cmdVel;
	return cmdVel;
}

// 初始化PID设置
void InitPID(PID *sptr, float Kp, float Ki, float Kd)
{
	sptr->y = 0.0;
	sptr->u = 0.0;
	sptr->u_pre = 0.0;
	sptr->Kp = Kp;
	sptr->Ki = Ki;
	sptr->Kd = Kd; 
	sptr->sum = 0.0;
}

// 初始化2阶IIR设置
void InitIIR2(IIR2 *iir, float a2, float a3, float b1, float b2, float b3)
{
	iir->a2 = a2;
	iir->a3 = a3;
	iir->b1 = b1;
	iir->b2 = b2;
	iir->b3 = b3;
	iir->y_pre = 0.0;
	iir->y_prepre = 0.0;
	iir->u_pre = 0.0;
	iir->u_prepre = 0.0;
}

// 2阶IIR滤波算法
// y[n] = b(1)*x(n) + b(2)*x(n-1) + b(3)*x(n-2) - a(2)*y(n-1) - a(3)*y(n-2)
float FilterIIR(IIR2 *pIIR, float u)
{
	float y;
	y = pIIR->b1 * u + pIIR->b2 * pIIR->u_pre + pIIR->b3 * pIIR->u_prepre - 
		pIIR->a2 * pIIR->y_pre - pIIR->a3 * pIIR->y_prepre;
	pIIR->u_prepre = pIIR->u_pre;
	pIIR->u_pre = u;
	pIIR->y_prepre = pIIR->y_pre;
	pIIR->y_pre = y;
	return y;
}

// 初始化电机
void InitMotor(MP *motor, const PID *ppid, const MVP* pmvp,
				int16 index, int16 feedbackInv)
{
	InitPID(&(motor->pid), ppid->Kp, ppid->Ki, ppid->Kd);
	InitMVP(&(motor->mvp), pmvp->accMax, pmvp->velMax, pmvp->stopAcc);
    //InitIIR2(piIIR, -1.852147, 0.862349, 0.002551, 0.005101, 0.002551); // Fc = 50Hz, Fs = 3000Hz
    //InitIIR2(piIIR, -1.705552, 0.743655, 0.009526, 0.019052, 0.009526); // Fc = 100Hz, Fs = 3000Hz
    InitIIR2(&(motor->iIIR), -1.418983, 0.553270, 0.033572, 0.067144, 0.033572); // Fc = 200Hz, Fs = 3000Hz

    //InitIIR2(poIIR, -1.418983, 0.553270, 0.033572, 0.067144, 0.033572); // Fc = 200Hz, Fs = 3000Hz
    //InitIIR2(poIIR, -0.620204, 0.240408, 0.155051, 0.310102, 0.155051); // Fc = 500Hz, Fs = 3000Hz
    InitIIR2(&(motor->oIIR), 0.122741, 0.174237, 0.324245, 0.648489, 0.324245); // Fc = 800Hz, Fs = 3000Hz
	motor->index = index;
	motor->feedbackInv = feedbackInv;
	motor->posCnt = 0;
	motor->unfiltedVel = 0.0;
	motor->filtedVel = 0.0;
	motor->current = 0.0;
	//motor->outputInv = outputInv;
}

// 设置电机速度
void SetMotorSpeed(MP* motor, float velCmd)
{
	float y, duty_cycle, motor_vel_cmd;
	if(motor->index < 1 || motor->index > 10)
		return ;

	// 速度环PID控制
	motor->unfiltedVel = (motor->feedbackInv) * MotorEncCnt[motor->index - 1] * C2R; // 当前电机速度(rpm) 

	// 输入滤波
	motor->filtedVel = FilterIIR(&(motor->iIIR), motor->unfiltedVel);
	
	// 速度约束
	motor_vel_cmd = ConstrainCmdVel(&(motor->mvp), velCmd);
	
	// 误差作为输入值
	motor->pid.u = motor_vel_cmd - motor->filtedVel;		
	if (motor->pid.u < VLOOP_CONTROL_SP && motor->pid.u > -VLOOP_CONTROL_SP)
	{
		motor->pid.sum += motor->pid.u;
		y = motor->pid.Kp * motor->pid.u + motor->pid.Ki * motor->pid.sum
			 + motor->pid.Kd * (motor->pid.u - motor->pid.u_pre);
	}
	else
	{
		y = motor->pid.Kp * motor->pid.u + motor->pid.Kd * (motor->pid.u - motor->pid.u_pre);
	}

	motor->pid.y = FilterIIR(&(motor->oIIR), y);	// 输出滤波
	//motor->current = motor->pid.y;
/*
	if(sptr->y > 2.45)
		sptr->y = 2.45;
	else if(sptr->y < -2.45)
		sptr->y = -2.45;
*/	
	duty_cycle = motor->pid.y * 0.1 + 0.5112;	// 计算占空比

	//if(duty_cycle > 1)
	//	duty_cycle = 0.8;
	//else if(duty_cycle < 0)
	//	duty_cycle = 0.2;

	SetEPwm(motor->index, duty_cycle);

	// 更新输入和输出
	motor->pid.u_pre = motor->pid.u;
}

void UpdateAllState(void)
{
	int i;
	//ReadAllSensor(JointEncCnt, TensionCnt, MotorEncCnt);
	for(i = 0; i < 4; i++)
	{
		Motor[i].posCnt += Motor[i].feedbackInv > 0 ? MotorEncCnt[i] : -MotorEncCnt[i];
	}
}

void ResetMotorPosCnt(void)
{
	int i, j;
	while(1)
	{
		j = 0;
		//ReadIncEncoder(MotorEncCnt);
		for(i = 0; i < 4; i++)
		{
			//if((MotorEncCnt[i] > 1) || (MotorEncCnt[i] < -1))
			if(MotorEncCnt[i] != 0)
				break;
			else
				j++;
		}
		if(j == 4)
			break;
	}
}

IIR2 torqueOutIIR2;

// 设置电机驱动绳索的张力，单位（Kg）
void SetMotorTorque(MP* motor, float tt)
{
	static float test = 0.0;
	static float te_pre = 0.0;
	float duty_cycle;
	float Kp = 0.02;
	float Ki = 0.1;
	float Kd = 0.0;
	float te = tt - motor->cableTension;	//  张力误差

	if(motor->index < 1 || motor->index > 10)
		return ;

	motor->unfiltedVel = (motor->feedbackInv) * MotorEncCnt[motor->index - 1] * C2R; // 当前电机速度(rpm) 

	// 输入滤波
	motor->filtedVel = FilterIIR(&(motor->iIIR), motor->unfiltedVel);

	// 误差小于0.5公斤，保持电流不变
	if(te < 0.2 && te > -0.2)
	{
		return;
	}
	te *= CURRENT_TENSION_FACTOR;
	//motor->current += motor->currentInv > 0 ? Kp * te : -Kp * te;
	motor->current = -(Kp * te + Ki * test + Kd * (te - te_pre)) ;

	motor->current = FilterIIR(&torqueOutIIR2, motor->current);	

	te_pre = te;

	if(motor->current > 4.5)
	{
		motor->current = 4.5;
	}
	else if(motor->current < -4.5)
	{
		motor->current = -4.5;
	}
	else
	{
		test += te;
	}
	duty_cycle = motor->current * 0.1 + 0.5112;			// 计算占空比
	SetEPwm(motor->index, duty_cycle);
}

// No more
