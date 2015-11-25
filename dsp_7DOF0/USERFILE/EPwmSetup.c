// TI File $Revision: /main/9 $
// Checkin $Date: August 10, 2007   09:05:08 $
//###########################################################################
//
// FILE:	Example_EpwmSetup.c
//
// TITLE:	Frequency measurement using EQEP peripheral
//
// DESCRIPTION:
//
// This file contains source for the ePWM initialization for the
// freq calculation module
//
//###########################################################################
// Original Author: SD
//
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "EPwmSetup.h"

// 标定后的占空比转速参数
const float K[10] = {12002.500, 12008.333, 12002.833, 12001.000, 12003.333, 12002.500, 12000.833, 12002.833, 12002.333, 0};
const float B[10] = {-6132.139, -6103.278, -6133.194, -6103.167, -6130.111, -6128.028, -6133.306, -6130.417, -6145.167, 0};
// 电机转速比
const int RR[10] = {111, 111, 111, 111, 111,
	111, 111, 111, 111, 111};

void EPwm1Config(float duty_cycleA, float duty_cycleB)
{
	if(duty_cycleA > 1 | duty_cycleA < 0 | duty_cycleB > 1 | duty_cycleB < 0 )
	{
		duty_cycleA = 0.5;
		duty_cycleB = 0.5;
	}
	EPwm1Regs.TBSTS.all=0;
	EPwm1Regs.TBPHS.half.TBPHS=0;
	EPwm1Regs.TBCTR=0;

	EPwm1Regs.CMPCTL.all=0x50;        // CMPA和CMPB立即模式
	EPwm1Regs.CMPA.half.CMPA = (unsigned short)(PWM_CNT * (1- duty_cycleA));		// 50% 占空比
	EPwm1Regs.CMPB = (unsigned short)(PWM_CNT * (1- duty_cycleB));					// 25% 占空比

	EPwm1Regs.AQCTLA.all=0x0060;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                  	// EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm1Regs.AQCTLB.all=0x0600;        // EPWMxB = 1 when CTR=CMPB and counter inc
	                                  	// EPWMxB = 0 when CTR=CMPB and counter dec
	EPwm1Regs.AQSFRC.all=0;
	EPwm1Regs.AQCSFRC.all=0;

	EPwm1Regs.DBCTL.all=0;          // EPWMxB is inverted
	EPwm1Regs.DBRED=0;
	EPwm1Regs.DBFED=0;

	EPwm1Regs.TZSEL.all=0;
	EPwm1Regs.TZCTL.all=0;
	EPwm1Regs.TZEINT.all=0;
	EPwm1Regs.TZFLG.all=0;
	EPwm1Regs.TZCLR.all=0;
	EPwm1Regs.TZFRC.all=0;

	EPwm1Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm1Regs.ETFLG.all=0;
	EPwm1Regs.ETCLR.all=0;
	EPwm1Regs.ETFRC.all=0;

	EPwm1Regs.PCCTL.all=0;

	EPwm1Regs.TBCTL.all=0x0010 | TBCTLVAL;			// Enable Timer, TBPRD register directly accesses
	EPwm1Regs.TBPRD = PWM_CNT;	
}

void EPwm2Config(float duty_cycleA, float duty_cycleB)
{
	if(duty_cycleA > 1 | duty_cycleA < 0 | duty_cycleB > 1 | duty_cycleB < 0 )
	{
		duty_cycleA = 0.5;
		duty_cycleB = 0.5;
	}
	EPwm2Regs.TBSTS.all=0;
	EPwm2Regs.TBPHS.half.TBPHS=0;
	EPwm2Regs.TBCTR=0;

	EPwm2Regs.CMPCTL.all=0x50;        // CMPA和CMPB立即模式
	EPwm2Regs.CMPA.half.CMPA = (unsigned short)(PWM_CNT * (1- duty_cycleA));
	EPwm2Regs.CMPB = (unsigned short)(PWM_CNT * (1- duty_cycleB));

	EPwm2Regs.AQCTLA.all=0x0060;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                  	// EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm2Regs.AQCTLB.all=0x0600;        // EPWMxB = 1 when CTR=CMPB and counter inc
	                                  	// EPWMxB = 0 when CTR=CMPB and counter dec
	EPwm2Regs.AQSFRC.all=0;
	EPwm2Regs.AQCSFRC.all=0;

	EPwm2Regs.DBCTL.all=0;          // EPWMxB is inverted
	EPwm2Regs.DBRED=0;
	EPwm2Regs.DBFED=0;

	EPwm2Regs.TZSEL.all=0;
	EPwm2Regs.TZCTL.all=0;
	EPwm2Regs.TZEINT.all=0;
	EPwm2Regs.TZFLG.all=0;
	EPwm2Regs.TZCLR.all=0;
	EPwm2Regs.TZFRC.all=0;

	EPwm2Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm2Regs.ETFLG.all=0;
	EPwm2Regs.ETCLR.all=0;
	EPwm2Regs.ETFRC.all=0;

	EPwm2Regs.PCCTL.all=0;

	EPwm2Regs.TBCTL.all=0x0010 | TBCTLVAL;			// Enable Timer, TBPRD register directly accesses
	EPwm2Regs.TBPRD = PWM_CNT;	
}

void EPwm3Config(float duty_cycleA, float duty_cycleB)
{
	if(duty_cycleA > 1 | duty_cycleA < 0 | duty_cycleB > 1 | duty_cycleB < 0 )
	{
		duty_cycleA = 0.5;
		duty_cycleB = 0.5;
	}
	EPwm3Regs.TBSTS.all=0;
	EPwm3Regs.TBPHS.half.TBPHS=0;
	EPwm3Regs.TBCTR=0;

	EPwm3Regs.CMPCTL.all=0x50;        // CMPA和CMPB立即模式
	EPwm3Regs.CMPA.half.CMPA = (unsigned short)(PWM_CNT * (1- duty_cycleA));		// 50% 占空比
	EPwm3Regs.CMPB = (unsigned short)(PWM_CNT * (1- duty_cycleB));					// 25% 占空比

	EPwm3Regs.AQCTLA.all=0x0060;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                  	// EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm3Regs.AQCTLB.all=0x0600;        // EPWMxB = 1 when CTR=CMPB and counter inc
	                                  	// EPWMxB = 0 when CTR=CMPB and counter dec
	EPwm3Regs.AQSFRC.all=0;
	EPwm3Regs.AQCSFRC.all=0;

	EPwm3Regs.DBCTL.all=0;          // EPWMxB is inverted
	EPwm3Regs.DBRED=0;
	EPwm3Regs.DBFED=0;

	EPwm3Regs.TZSEL.all=0;
	EPwm3Regs.TZCTL.all=0;
	EPwm3Regs.TZEINT.all=0;
	EPwm3Regs.TZFLG.all=0;
	EPwm3Regs.TZCLR.all=0;
	EPwm3Regs.TZFRC.all=0;

	EPwm3Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm3Regs.ETFLG.all=0;
	EPwm3Regs.ETCLR.all=0;
	EPwm3Regs.ETFRC.all=0;

	EPwm3Regs.PCCTL.all=0;

	EPwm3Regs.TBCTL.all=0x0010 | TBCTLVAL;			// Enable Timer, TBPRD register directly accesses
	EPwm3Regs.TBPRD = PWM_CNT;	
}

void EPwm4Config(float duty_cycleA, float duty_cycleB)
{
	if(duty_cycleA > 1 | duty_cycleA < 0 | duty_cycleB > 1 | duty_cycleB < 0 )
	{
		duty_cycleA = 0.5;
		duty_cycleB = 0.5;
	}
	EPwm4Regs.TBSTS.all=0;
	EPwm4Regs.TBPHS.half.TBPHS=0;
	EPwm4Regs.TBCTR=0;

	EPwm4Regs.CMPCTL.all=0x50;        // CMPA和CMPB立即模式
	EPwm4Regs.CMPA.half.CMPA = (unsigned short)(PWM_CNT * (1- duty_cycleA));		// 50% 占空比
	EPwm4Regs.CMPB = (unsigned short)(PWM_CNT * (1- duty_cycleB));					// 25% 占空比

	EPwm4Regs.AQCTLA.all=0x0060;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                  	// EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm4Regs.AQCTLB.all=0x0600;        // EPWMxB = 1 when CTR=CMPB and counter inc
	                                  	// EPWMxB = 0 when CTR=CMPB and counter dec
	EPwm4Regs.AQSFRC.all=0;
	EPwm4Regs.AQCSFRC.all=0;

	EPwm4Regs.DBCTL.all=0;          // EPWMxB is inverted
	EPwm4Regs.DBRED=0;
	EPwm4Regs.DBFED=0;

	EPwm4Regs.TZSEL.all=0;
	EPwm4Regs.TZCTL.all=0;
	EPwm4Regs.TZEINT.all=0;
	EPwm4Regs.TZFLG.all=0;
	EPwm4Regs.TZCLR.all=0;
	EPwm4Regs.TZFRC.all=0;

	EPwm4Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm4Regs.ETFLG.all=0;
	EPwm4Regs.ETCLR.all=0;
	EPwm4Regs.ETFRC.all=0;

	EPwm4Regs.PCCTL.all=0;

	EPwm4Regs.TBCTL.all=0x0010 | TBCTLVAL;			// Enable Timer, TBPRD register directly accesses
	EPwm4Regs.TBPRD = PWM_CNT;	
}

void EPwm5Config(float duty_cycleA, float duty_cycleB)
{
	if(duty_cycleA > 1 | duty_cycleA < 0 | duty_cycleB > 1 | duty_cycleB < 0 )
	{
		duty_cycleA = 0.5;
		duty_cycleB = 0.5;
	}
	EPwm5Regs.TBSTS.all=0;
	EPwm5Regs.TBPHS.half.TBPHS=0;
	EPwm5Regs.TBCTR=0;

	EPwm5Regs.CMPCTL.all=0x50;        // CMPA和CMPB立即模式
	EPwm5Regs.CMPA.half.CMPA = (unsigned short)(PWM_CNT * (1- duty_cycleA));		// 50% 占空比
	EPwm5Regs.CMPB = (unsigned short)(PWM_CNT * (1- duty_cycleB));					// 25% 占空比

	EPwm5Regs.AQCTLA.all=0x0060;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                  	// EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm5Regs.AQCTLB.all=0x0600;        // EPWMxB = 1 when CTR=CMPB and counter inc
	                                  	// EPWMxB = 0 when CTR=CMPB and counter dec
	EPwm5Regs.AQSFRC.all=0;
	EPwm5Regs.AQCSFRC.all=0;

	EPwm5Regs.DBCTL.all=0;          // EPWMxB is inverted
	EPwm5Regs.DBRED=0;
	EPwm5Regs.DBFED=0;

	EPwm5Regs.TZSEL.all=0;
	EPwm5Regs.TZCTL.all=0;
	EPwm5Regs.TZEINT.all=0;
	EPwm5Regs.TZFLG.all=0;
	EPwm5Regs.TZCLR.all=0;
	EPwm5Regs.TZFRC.all=0;

	EPwm5Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm5Regs.ETFLG.all=0;
	EPwm5Regs.ETCLR.all=0;
	EPwm5Regs.ETFRC.all=0;

	EPwm5Regs.PCCTL.all=0;

	EPwm5Regs.TBCTL.all=0x0010 | TBCTLVAL;			// Enable Timer, TBPRD register directly accesses
	EPwm5Regs.TBPRD = PWM_CNT;	
}

void EPwm6Config(float duty_cycleA, float duty_cycleB)
{
	if(duty_cycleA > 1 | duty_cycleA < 0 | duty_cycleB > 1 | duty_cycleB < 0 )
	{
		duty_cycleA = 0.5;
		duty_cycleB = 0.5;
	}
	EPwm6Regs.TBSTS.all=0;
	EPwm6Regs.TBPHS.half.TBPHS=0;
	EPwm6Regs.TBCTR=0;

	EPwm6Regs.CMPCTL.all=0x50;        // CMPA和CMPB立即模式
	EPwm6Regs.CMPA.half.CMPA = (unsigned short)(PWM_CNT * (1- duty_cycleA));		// 50% 占空比
	EPwm6Regs.CMPB = (unsigned short)(PWM_CNT * (1- duty_cycleB));					// 25% 占空比

	EPwm6Regs.AQCTLA.all=0x0060;        // EPWMxA = 1 when CTR=CMPA and counter inc
	                                  	// EPWMxA = 0 when CTR=CMPA and counter dec
	EPwm6Regs.AQCTLB.all=0x0600;        // EPWMxB = 1 when CTR=CMPB and counter inc
	                                  	// EPWMxB = 0 when CTR=CMPB and counter dec
	EPwm6Regs.AQSFRC.all=0;
	EPwm6Regs.AQCSFRC.all=0;

	EPwm6Regs.DBCTL.all=0;          // EPWMxB is inverted
	EPwm6Regs.DBRED=0;
	EPwm6Regs.DBFED=0;

	EPwm6Regs.TZSEL.all=0;
	EPwm6Regs.TZCTL.all=0;
	EPwm6Regs.TZEINT.all=0;
	EPwm6Regs.TZFLG.all=0;
	EPwm6Regs.TZCLR.all=0;
	EPwm6Regs.TZFRC.all=0;

	EPwm6Regs.ETSEL.all=0;            // Interrupt when TBCTR = 0x0000
	EPwm6Regs.ETFLG.all=0;
	EPwm6Regs.ETCLR.all=0;
	EPwm6Regs.ETFRC.all=0;

	EPwm6Regs.PCCTL.all=0;

	EPwm6Regs.TBCTL.all=0x0010 | TBCTLVAL;			// Enable Timer, TBPRD register directly accesses
	EPwm6Regs.TBPRD = PWM_CNT;	
}

void EPwmSetup()
{
	// 使能前10个Epwm的I/O引脚
    InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
	InitEPwm5Gpio();
	//InitEPwm6Gpio();

	EPwm1Config(0, 0);
	EPwm2Config(0, 0);
	EPwm3Config(0, 0);
	EPwm4Config(0, 0);
	EPwm5Config(0, 0);

}

void SetEPwm(int index, float duty_cycle)
{
	unsigned short cnt;
	if(duty_cycle > 1 | duty_cycle < 0)
	{
		duty_cycle = 0.5;
	}
	cnt = PWM_CNT * (1- duty_cycle);
	switch(index)
	{
	case 1:
		EPwm1Regs.CMPA.half.CMPA = cnt;
		break;
	case 2:
		EPwm1Regs.CMPB = cnt;
		break;
	case 3:
		EPwm2Regs.CMPA.half.CMPA = cnt;
		break;
	case 4:
		EPwm2Regs.CMPB = cnt;
		break;
	case 5:
		EPwm3Regs.CMPA.half.CMPA = cnt;
		break;
	case 6:
		EPwm3Regs.CMPB = cnt;
		break;
	case 7:
		EPwm4Regs.CMPA.half.CMPA = cnt;
		break;
	case 8:
		EPwm4Regs.CMPB = cnt;
		break;
	case 9:
		EPwm5Regs.CMPA.half.CMPA = cnt;
		break;
	case 10:
		EPwm5Regs.CMPB = cnt;
		break;
	default:
		break;
	}
}

// 由电机转速（减速后）计算所需要占空比， 电机转速单位rpm
float GetDutyCycle(int index, float motorVel)
{
	if(index > 9 || index < 1)
		return 0.0;
	else
		index--;
	motorVel *= RR[index];	// 乘以减速比
	// 限速5500转
	if(motorVel > 5500)
		motorVel = 5500;
	else if(motorVel < -5500)
		motorVel = -5500;
	// 计算对应占空比
	return (motorVel - B[index]) / K[index];
}
