#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "EPwmSetup.h"			 
#include "TimerSetup.h"
#include "SciSetup.h"

#include "Algorithm.h"
#include "Fpga.h"
#include "CopleyControl.h"

#include <stdio.h>

#define TENSION_FACTOR 0.0003052 // 1 / 32767.0 * 10.0

#define TRUE	1
#define FALSE	0

/* �����������ӷ����ϵ�������ת��Ӧ�����ս�����ת��Ӧ�����ͷ� */

// Prototype statements for functions found within this file.
void InitAllMotor(void);
void ScicXmtStatus(void);
void ScicXmtTransferStatus(int status, Uint16 count);

#define CPR 1024	// count per revolution
#define MOTOR_ACC 2.5 // rpm/ms
#define vFactor 0.054054
#define vk -55.457//���NO.4pwm����Ͳ���������Ĳ���k
#define vb -8.9931//���NO.4pwm����Ͳ���������Ĳ���b

const float CONTROL_FREQ = 200.0;

float *AddrTrajJointAngles[7] = {(float *)0x100000, (float *)0x101000, (float *)0x102000, (float *)0x103000,
								 (float *)0x104000, (float *)0x105000, (float *)0x106000};
float *AddrTrajCurrJointAngles[7] = {(float *)0x107000, (float *)0x108000, (float *)0x109000, (float *)0x10A000,
								 (float *)0x10B000, (float *)0x10C000, (float *)0x10D000};
char *msg = (char *)0x113000;

// ȫ�ֱ���
MotorModel MM[10];			// ���ģ��
const float ZeroDutyCycleArray[10] = {0.5100, 0.5105, 0.5105, 0.5085, 0.5105, 
								      0.5120, 0.5110, 0.5110, 0.5110, 0.5110};
int16 JointEncCnts[7];			// ��ǰ�ؽڽǱ�����ADֵ
int16 Tension[8];				//��ǰ��������ADֵ
int16 TensionBackup[8];			//�󱸵��ĸ���������ADֵ
float TensionReal[8];			//��ʵ����������ֵ��kg��
float JointAngles[7];			// ��ǰ�ĹؽڽǶ�(rad)
float TargetJointAngles[7];		// �����Ĺؽڽ�
float ourv[7];
const int16 JointEncCntsZero[7] = 
		{25237, 6659, 11645, 17500, 21344, 17645, 9200 };		// 10645
float motorVelForFeedback[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	// �����㷨�õ��ĵ���ٶ�
int TrajCounter = 0;
int16 v = 0;

void main(void)
{
	int i;

    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2833x_SysCtrl.c file.
    InitSysCtrl();

    // Step 2. Initalize GPIO:
    // This example function is found in the DSP2833x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    // InitGpio(); Skipped for this example

    // For this example, only init the pins for the SCI-A port.
    // This function is found in the DSP2833x_Sci.c file.
    InitScicGpio();

    InitXintf16Gpio();
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2833x_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2833x_DefaultIsr.c.
    // This function is found in DSP2833x_PieVect.c.
    InitPieVectTable();

    // Step 4. Initialize all the Device Peripherals:
    // This function is found in DSP2833x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example

    // Step 5. User specific code:

    // ��ʼ��PWMģ��
    EPwmSetup();

    // ��ʼ����ʱ��ģ��, ���5ms�����ж�
    TimerSetup(1000000.0 / CONTROL_FREQ);

	// ��ʼ�����е��
	InitAllMotor();

    // ��ʼ������ģ��
    ScicInit();	   		// ��ʼ������ͨ�Ų�����������115200
    ScicFifoInit();  	// ��ʼ������FIFOģ��

	for(i = 0; i < 1024; i++)
		COM_REG[i] = 0;
	for(i = 0; i < 0x7000; ++i)
		*((float *)0x100000 + i) = 0;

    //GenerateSineWave(Wave, 5, 200, 0.2, 0.0, 2000);
    //GenerateSquareWave(Wave, -500, 500, FS, 5, 2000);

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    while(1)
    {
    	// 5ms�ж�
        if(IsTimesUp())
        {  
			int i;
			// ��ȡAD�ɼ��Ĺؽڽ�����
        	//ReadAbsEncoder(JointEncCnts);
        	ReadAllSensor(TensionBackup,Tension,JointEncCnts);
			// ���������Ĺؽڽ�
			for(i = 0; i < 7; i++)
			{
				// ת��Ϊʵ�ʽǶ�(rad)
				//JointAngles[i] = (JointEncCnts[i] - JointEncCntsZero[i]) * 0.0136 * 0.017453;
				// ת��Ϊʵ�ʽǶ�(rad)
				TensionReal[i] = Tension[i] * TENSION_FACTOR;
				// ��ȡ��λ�����͵������ؽڽ�(rad)
				TargetJointAngles[i] = ((int16)COM_REG[0x40 + i]) / 10.0 * 0.017453;
				ourv[i] = JointEncCnts[i] * vFactor;
			}

			COM_REG[0x300] = 0x01;
			// 0x01����ʾ����ٶȿ��ƣ�0x02����ʾ�ؽڽǶȿ���
			if (COM_REG[0x300] == 0x01)
			{
				// �����ٶ�ָ��
				for (i = 0; i < 10; i++)
					//SetVelocity((MM + i), ((int16)COM_REG[0x50 + i]));
					SetVelocity((MM + i), vk*v+vb);
			}
			else if(COM_REG[0x300] == 0x02)
			{
				ControlShoulder(JointAngles, TargetJointAngles, motorVelForFeedback);				// ��ؽڱջ����ƣ��ٶ���Ϊ�ͷţ���Ϊ�ս�
				motorVelForFeedback[4] = -ControlElbow(JointAngles[3], TargetJointAngles[3]);		// ��ؽڱջ�����
				motorVelForFeedback[5] = -motorVelForFeedback[4];
				ControlWrist(JointAngles + 4, TargetJointAngles + 4, motorVelForFeedback + 6);	// ��ؽڱջ�����
				for (i = 0; i < 10; i++)
				{
					motorVelForFeedback[i] *= (9.5493 * 110.0);
					// ����ٶ�Լ��
					if(motorVelForFeedback[i] > 500.0)
						motorVelForFeedback[i] = 500.0;
					else if(motorVelForFeedback[i] < -500.0)
						motorVelForFeedback[i] = -500.0;
					SetVelocity((MM + i), motorVelForFeedback[i]);
				}									
			}
			else if(COM_REG[0x300] == 0x03)
			{
				// ���ݴ���ģʽ
				if(COM_REG[0x100] == 0xAAAA)
				{
					int n = COM_REG[0x101];
					if(n > 0 && n < 2048)
					{
						for(i = 0; i < 7; ++i)
							*(AddrTrajJointAngles[i] + n - 1) = (float)(*(short *)&COM_REG[0x102 + i]) * 1.7453292e-4; 					
					}
					COM_REG[0x100] = 0x0000;
					ScicXmtTransferStatus(TRUE, n);
				}
			}
			else if(COM_REG[0x300] == 0x04)
			{
				if(COM_REG[0x100] == 0xBBBB)
				{
					float ja[7];
					for(i = 0; i < 7; ++i) 
						ja[i] = *(AddrTrajJointAngles[i] + TrajCounter);
					ControlShoulder(JointAngles, ja, motorVelForFeedback);						// ��ؽڱջ����ƣ��ٶ���Ϊ�ͷţ���Ϊ�ս�
					motorVelForFeedback[4] = -ControlElbow(JointAngles[3], ja[3]);				// ��ؽڱջ�����
					ControlWrist(JointAngles + 4, ja + 4, motorVelForFeedback + 6);				// ��ؽڱջ�����
					for (i = 0; i < 10; i++)
					{
						motorVelForFeedback[i] *= (9.5493 * 110.0);
						// ����ٶ�Լ��
						if(motorVelForFeedback[i] > 500.0)
							motorVelForFeedback[i] = 500.0;
						else if(motorVelForFeedback[i] < -500.0)
							motorVelForFeedback[i] = -500.0;
						SetVelocity((MM + i), motorVelForFeedback[i]);
					}
					if(TrajCounter < COM_REG[0x101])
					{
						for(i = 0; i < 7; ++i) 
							*(AddrTrajCurrJointAngles[i] + TrajCounter) = JointAngles[i];
						TrajCounter++;
					}
					else if(TrajCounter == COM_REG[0x101])
					{
						COM_REG[0x100] = 0x0000;
						TrajCounter = 0;
					}	
				}
				else
				{
					for (i = 0; i < 10; i++)
						SetVelocity((MM + i), 0);	//ͣ��					
				}				
			}
			else
			{
				for (i = 0; i < 10; i++)
					SetVelocity((MM + i), 0);	//ͣ��
			}
							
            ClrTimesUpFlag();
      	
			if(COM_REG[0] == 0x0001)
			{
				sprintf(msg, "Machine stop!\r\n");
            	scic_msg(msg);
				for (i = 0; i < 10; i++)
					SetEPwm(i + 1, 0);	
				break;
			}
				
        }

		/* 0.5s ����λ��ͨѶ */
        if(IsTimer2Up())
        {
            BlinkLED();			// 0.5����˸LED
			ScicXmtStatus();	// ���ͻ�����״̬
            ClrTimer2Flag();
        }
    }
}

void InitAllMotor(void)
{
	// ��ؽڵ��
	InitMotorModel(&MM[0], 1, 1, 1000.0, 10, ZeroDutyCycleArray[0]);	// ����ٶ�3000 rpm, �����ٶ� 100 rpm/s = 0.5 rpm / 5ms;
	InitMotorModel(&MM[1], 2, 1, 1000.0, 10, ZeroDutyCycleArray[1]);
	InitMotorModel(&MM[2], 3, -1, 1000.0, 10, ZeroDutyCycleArray[2]);
	InitMotorModel(&MM[3], 4, -1, 1000.0, 10, ZeroDutyCycleArray[3]);
	// ��ؽڵ��
	InitMotorModel(&MM[4], 5, 1, 1000.0, 10, ZeroDutyCycleArray[4]);
	InitMotorModel(&MM[5], 6, -1, 1000.0, 10, ZeroDutyCycleArray[5]);
	// ��ؽڵ��
	InitMotorModel(&MM[6], 7, -1, 1000.0, 10, ZeroDutyCycleArray[6]);
	InitMotorModel(&MM[7], 8, -1, 1000.0, 10, ZeroDutyCycleArray[7]);
	InitMotorModel(&MM[8], 9,  1, 1000.0, 10, ZeroDutyCycleArray[8]);
	InitMotorModel(&MM[9], 10, 1, 1000.0, 10, ZeroDutyCycleArray[9]);
}

// ����״̬
void ScicXmtStatus(void)
{
	int i = 0;
	Uint16 array[18];
	Uint16 checksum = 0;
	array[0] = 0xFF;
	array[1] = 0xFA;
	array[2] = 15;
	for(i = 0; i < 7; i++)
	{
		array[2 * i + 3] = JointEncCnts[i] & 0xFF;
		array[2 * i + 4] = JointEncCnts[i] >> 8;
		checksum += array[2 * i + 3] + array[2 * i + 4];
	}
	array[17] = checksum & 0xFF;
	ScicXmt(array, 18);
}

void ScicXmtTransferStatus(int status, Uint16 count)
{
	Uint16 array[7];
	array[0] = 0xFF;
	array[1] = 0xFA;
	array[2] = 0x04;
	array[3] = status;
	array[4] = count & 0xFF;
	array[5] = count >> 8;
	array[6] = array[3] + array[4] + array[5];
	ScicXmt(array, 7);
}

//===========================================================================
// No more.
//===========================================================================

