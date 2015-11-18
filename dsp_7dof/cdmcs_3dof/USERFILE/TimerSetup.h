#ifndef TIMERSETUP_H_
#define TIMERSETUP_H_

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

interrupt void ISRTimer0(void);
void TimerSetup(float uSecond);

int16 IsTimer1Up(void);
int16 IsTimer2Up(void);

void ClrTimer1Flag(void);
void ClrTimer2Flag(void);

int16 IsTimesUp(void);
void ClrTimesUpFlag(void);

#endif /* TIMERSETUP_H_ */
