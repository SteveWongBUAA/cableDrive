#include "CopleyControl.h"
#include "EPwmSetup.h"

const float DUTY_CYCLE_ZERO = 0.5112;	// 电机输出0点对应占空比
const float COPLEY_MAX_VELOCITY = 6000.0;

float SetVelocity(MotorModel *mm, float vel)
{
	vel = (mm->dir) > 0 ? vel : -vel;
	mm->cmdVelocity = vel;
	if((vel - mm->preVelocity) > mm->maxAcc)
		mm->velocity = mm->preVelocity + mm->maxAcc;
	else if((vel - mm->preVelocity) < -mm->maxAcc)
		mm->velocity = mm->preVelocity - mm->maxAcc;
	else
		mm->velocity = vel;
	
	if(mm->velocity > mm->maxVel)
		mm->velocity = mm->maxVel;
	else if(mm->velocity < -mm->maxVel)
		mm->velocity = -mm->maxVel;

	mm->preVelocity = mm->velocity;
	mm->dutyCycle = mm->velocity / COPLEY_MAX_VELOCITY + mm->zeroDutyCycle;	

	if(mm->index != 2 && mm->index != 3 && mm->index != 5 && mm->index != 6)
	{
		if(mm->velocity < 1.0 && mm->velocity > -1.0)
			SetEPwm(mm->index, 0);
		else
			SetEPwm(mm->index, mm->dutyCycle);		
	}
	else
	{
		SetEPwm(mm->index, mm->dutyCycle);
	}
	return mm->velocity;
}

int InitMotorModel(MotorModel *mm, int index, int dir, float maxVel, float maxAcc, float zeroDutyCycle)
{
	if(maxVel < 0.1)	return -1;
	if(maxAcc < 0.1)	return -2;
	if(index < 1 || index > 10) return -3;
	if(zeroDutyCycle < 0) return -4;

	mm->index = index;
	mm->dir = dir;
	mm->maxVel = maxVel;
	mm->maxAcc = maxAcc;

	mm->zeroDutyCycle = zeroDutyCycle;
	mm->cmdVelocity = 0.0;
	mm->velocity = 0.0;
	mm->preVelocity = 0.0;
	mm->dutyCycle = 0;
	return 0;
}

// end of file
