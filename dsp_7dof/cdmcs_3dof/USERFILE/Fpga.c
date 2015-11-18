#include "Fpga.h"

int16 *LED_ADD = (int16 *)0x4001;
int16 *INC_ENCODER = (int16 *)0x4010;
int16 *ABS_ENCODER = (int16 *)0x4020;
int16 *TENSION_SENSOR = (int16 *)0x4028;
int16 *ENCODER_RD = (int16 *)0x4000;
Uint16  *ExRamStart = (Uint16 *)0x100000;

Uint16 led = 0;

void EncoderRd(void)
{
	*ENCODER_RD = 0x0000;
	DELAY_US(2);
	*ENCODER_RD = 0x0001;
	DELAY_US(10);
	*ENCODER_RD = 0x0000;
}
/*
void ReadIncEncoder(int16 *encoder)
{
	int i = 0;
	EncoderRd();	// 读取FPGA中的编码器数据
	for (; i < 7; i++)
	{
		encoder[i] = *(INC_ENCODER + i);
	}
	return;
}

void ReadAbsEncoder(int16 *encoder)
{
	int i = 0;
	EncoderRd();	// 读取FPGA中的编码器数据
	for (; i < 7; i++)
	{
		encoder[i] = *(ABS_ENCODER + i);
	}
	return;
}
*/
void BlinkLED(void)
{
	led = ~led;
	*LED_ADD = led;	
}

Uint16 GetLED(void)
{
	return led;
}

void SetLED(Uint16 led)
{
	*LED_ADD = led;
}
/*
void ReadEncoder(int16 *absEncoder, int16 *incEncoder)
{
	int i = 0;
	EncoderRd();	// 读取FPGA中的编码器数据
	for (; i < 7; i++)
	{
		incEncoder[i] = *(INC_ENCODER + i);
	}
	for (i = 0; i < 7; i++)
	{
		absEncoder[i] = *(ABS_ENCODER + i);
	}
}*/

void ReadTension(int16 *tension)
{
	int i = 0;
	for (; i < 8; i++)
	{
		tension[i] = *(TENSION_SENSOR + i);
	}
}

/*
void ReadAllSensor(int16 *absEncoder, int16* tension, int16 *incEncoder)
{
	int i = 0;
	EncoderRd();	// 读取FPGA中的编码器数据
	for (; i < 7; i++)
	{
		incEncoder[i] = *(INC_ENCODER + i);
	}
	for (i = 0; i < 8; i++)
	{
		absEncoder[i] = *(ABS_ENCODER + i);
	}
	for (i = 0; i < 8; i++)
	{
		tension[i] = *(TENSION_SENSOR + i);
	}

}
*/
// No More


