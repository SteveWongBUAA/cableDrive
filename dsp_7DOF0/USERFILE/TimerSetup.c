#include "TimerSetup.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "Fpga.h"

int16 Timer1Flag = 0;
int16 Timer2Flag = 0;
int16 TimesUpFlag = 0;

int16 IsTimer1Up(void)
{
	return Timer1Flag;
}

void ClrTimer1Flag(void)
{
	Timer1Flag = 0;
}

int16 IsTimer2Up(void)
{
	return Timer2Flag;
}

void ClrTimer2Flag(void)
{
	Timer2Flag = 0;
}

int16 IsTimesUp(void)
{
	return TimesUpFlag;
}

void ClrTimesUpFlag(void)
{
	TimesUpFlag = 0;
}

interrupt void ISRTimer0(void)
{
    CpuTimer0.InterruptCount++;
	TimesUpFlag = 1;
	Timer2Flag = ((CpuTimer0.InterruptCount % 100) == 0) ? 1 : 0;	// 100 * 5ms = 0.5s
	Timer1Flag = ((CpuTimer0.InterruptCount % 3) == 0) ? 1 : 0;

   // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    CpuTimer0Regs.TCR.bit.TIF=1;
    CpuTimer0Regs.TCR.bit.TRB=1;
}

void TimerSetup(float uSecond)
{
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.TINT0 = &ISRTimer0;
   //PieVectTable.XINT13 = &cpu_timer1_isr;
   //PieVectTable.TINT2 = &cpu_timer2_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2833x_CpuTimers.c
	InitCpuTimers();   // For this example, only initialize the Cpu Timers

// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 150MHz CPU Freq, 20 uSeconds Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 150, uSecond);		
	StartCpuTimer0();

// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:
    IER |= M_INT1;
   //IER |= M_INT13;
   //IER |= M_INT14;

// Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	
// Enable global Interrupts and higher priority real-time debug events:
//    EINT;   // Enable Global interrupt INTM
//    ERTM;   // Enable Global realtime interrupt DBGM
}

//===========================================================================
// No more.
//===========================================================================


