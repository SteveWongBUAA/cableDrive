#ifndef FPGA_H_
#define FPGA_H_

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//void ReadIncEncoder(int16 *encoder);

//void ReadAbsEncoder(int16 *encoder);

void BlinkLED(void);

Uint16 GetLED(void);

void SetLED(Uint16 led);

//void ReadEncoder(int16 *absEncoder, int16 *incEncoder);

//void ReadAllSensor(int16 *absEncoder, int16* tension, int16 *incEncoder);

void ReadTension(int16 *tension);

#endif /*FPGA_H_*/

