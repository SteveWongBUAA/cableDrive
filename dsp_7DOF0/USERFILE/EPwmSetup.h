#ifndef EPWMSETUP_H_
#define EPWMSETUP_H_

#if (CPU_FRQ_150MHZ)
	#define CPU_CLK   150e6
#endif
#if (CPU_FRQ_100MHZ)
	#define CPU_CLK   100e6
#endif

#define PWM_CLK   15000                // 设置PWM的频率为15KHz
#define PWM_CNT        (CPU_CLK/(2*PWM_CLK))
#define TBCTLVAL  0x200E              // Up-down cnt, timebase = SYSCLKOUT

void EPwmSetup();
void SetEPwm(int index, float duty_cycle);
float GetDutyCycle(int index, float motorVel);

#endif /*EPWMSETUP_H_*/
