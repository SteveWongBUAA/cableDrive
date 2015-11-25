#ifndef _COPLEY_CONTROL_H_
#define _COPLEY_CONTROL_H_

typedef struct _MotorModel
{
	int index;				// 电机序号
	int dir;				// 电机方向
	float cmdVelocity;		// 指令速度
	float velocity;			// 实际速度
	float preVelocity;		// 上次的指令速度
	float maxVel;			// 最大速度(正数)
	float dutyCycle;		// 实际占空比
	float zeroDutyCycle;	// 零速度对应占空比
	float maxAcc;			// 最大加速度(正数)
}MotorModel;

float SetVelocity(MotorModel *mm, float vel);
int InitMotorModel(MotorModel *mm, int index, int dir, float maxVel, float maxAcc, float zeroDutyCycle);

#endif
// end of file
