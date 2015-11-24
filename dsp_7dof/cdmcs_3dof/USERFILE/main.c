#include <stdio.h>
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "Ecan.h"
#include "EPwmSetup.h"			 
#include "TimerSetup.h"
#include "SciSetup.h"
#include "Algorithm.h"
#include "Fpga.h"
#include "CopleyControl.h"
#include "QP_1.h"


//debug
//#define TENSION_FACTOR 0.0015259 // 1 / 32767.0 * 50.0
//#define TENSION_FACTOR 0.00015259 // 1 / 65536.0 * 10.0
#define TENSION_FACTOR 0.0003052 // 1 / 32767.0 * 10.0
#define pi 3.14159
#define TRUE	1
#define FALSE	0


/* 电机方向和绳子方向关系：电机正转对应绳子收紧，反转对应绳子释放 */

// Prototype statements for functions found within this file.
void InitAllMotor(void);
void ScicXmtStatus(void);
void ScicXmtTransferStatus(int status, Uint16 count);

#define CPR 1024	// count per revolution
#define MOTOR_ACC 2.5 // rpm/ms
#define maxWaitIMU 3 //最多尝试3次进入IMU COMMAND MODE，否则跳出循环

const float CONTROL_FREQ = 200.0;

float *AddrTrajJointAngles[7] = {(float *)0x100000, (float *)0x101000, (float *)0x102000, (float *)0x103000,
								 (float *)0x104000, (float *)0x105000, (float *)0x106000};
float *AddrTrajCurrJointAngles[7] = {(float *)0x107000, (float *)0x108000, (float *)0x109000, (float *)0x10A000,
								 (float *)0x10B000, (float *)0x10C000, (float *)0x10D000};
char *msg = (char *)0x113000;

// 全局变量


MotorModel MM[10];			// 电机模型
const float ZeroDutyCycleArray[10] = {0.5100, 0.5105, 0.5105, 0.5085, 0.5105, 
								      0.5120, 0.5110, 0.5110, 0.5110, 0.5110};
int16 tension[8];				// 当前力传感器AD值
int16 incEncoder[7];			// 当前增量编码器值,来自FPGA,白色端子
int16 absEncoder[8];			// 当前绝对编码器值
float TensionT[8];				// 当前的拉力(kg)
float MechPara[20];				// 从上位机传过来的机械参数
float IMUdata[9];	//IMU数据，三个IMU的欧拉角xyz
Uint32 IMUhex[9];//IMU数据（十六进制）

//float R1[9];//旋转矩阵R1
//float R2[9];//旋转矩阵R2

//float jacob_e[6];
//float ans=0;
//float x[4];



//int16 JointEncCnts[7];			// 当前关节角编码器AD值
//int16 absEncoder[7];			// 当前绝对编码器AD值
//float JointAngles[7];			// 当前的关节角度(rad)
//float TargetJointAngles[7];		// 期望的关节角
//Uint32 gotoCmdFlag[3];
//Uint32 setOffsetFlag[3];
//Uint32 missCount[3];
//float q11,q12,q13,q21,q22,q23;
//float Posi1[3] = {-0.4, -0.1, -0.1};
//float Posi2[3] = {0.2 ,0.3, 0.3};
//float PosiCur[3] = {-0.1, -0.1, 0.2};
//float fp[3] = {0, 0, 0};
//float d = 0;
//Uint32 IMUMSGL[9];
//Uint32 IMUMSGH[9];
//Uint32 OPC[9];
//Uint32 RMP[9];

/*
const int16 JointEncCntsZero[7] = 
		{25237, 6659, 11645, 17500, 21344, 17645, 9200 };		// 10645
float motorVelForFeedback[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	// 反馈算法得到的电机速度
int TrajCounter = 0;
*/
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

    // 初始化PWM模块
    EPwmSetup();

    // 初始化定时器模块, 间隔5ms触发中断
    TimerSetup(1000000.0 / CONTROL_FREQ);

	// 初始化所有电机
	InitAllMotor();

    // 初始化串口模块
    ScicInit();	   		// 初始化串口通信参数，波特率115200
    ScicFifoInit();  	// 初始化串口FIFO模块

	for(i = 0; i < 1024; i++)
		COM_REG[i] = 0;
	for(i = 0; i < 0x7000; ++i)
		*((float *)0x100000 + i) = 0;

    InitMyEcan();//初始化CAN

	//GenerateSineWave(Wave, 5, 200, 0.2, 0.0, 2000);
    //GenerateSquareWave(Wave, -500, 500, FS, 5, 2000);



//    int wait = 0;
//    while(1)
//    {
//    	//IMU进入命令模式，成功返回1，失败返回0
//    	wait ++;
//    	if(IMU1_GotoCmd() && IMU2_GotoCmd() &&  IMU3_GotoCmd())
//    		break;
//    	if(wait > maxWaitIMU)
//    		break;
//    }


//    float J[12] = { 0.0000 , 111.7128, -135.5402, -0.0000,
//    84.4020, - 79.7948, - 96.8144,   47.1136,
//    - 0.5231, - 62.2949,   41.0156,   58.4505 };
//    float tt[3] = {500,500,800};
//    ans = QP(J,tt,x);



    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    int led = 0;
    while(1)
    {

    	// 5ms中断
        if(IsTimesUp())
        {  
			int i;
			// 读取AD采集的关节角数据
        	//ReadAbsEncoder(JointEncCnts);
        	// 读取所有传感器数据
        	//ReadAllSensor(absEncoder,tension,incEncoder);

        	//读取Euler角，IMUdata[0]-[9]代表第一个IMU的欧拉角X,Y,Z,第二个IMU的欧拉角X,Y,Z,第三个IMU的欧拉角X,Y,Z
        	//返回一个数，这个数大于missCountTolerated（在Ecan.c中设置）则表示错误，同时IMUdata[n]=4,超过正常的欧拉角范围
//        	IMU1_read_LpCan(IMUhex,IMUdata);
//        	IMU2_read_LpCan(IMUhex,IMUdata);
//        	IMU3_read_LpCan(IMUhex,IMUdata);

        	//读取上位机传过来的机械参数
        	for(i = 0; i < 20; i++)
        		MechPara[i] = ((int16)COM_REG[0x60 + i]) / 100.0;

        	//读取下位机测到的拉力AD值
        	ReadTension(tension);
        	//ReadIncEncoder(incEncoder);
        	//ReadAbsEncoder(absEncoder);
        	for(i = 0; i < 8; i++)
			{
				// 转换为实际测到的力(kg)
				TensionT[i] = tension[i] * TENSION_FACTOR ;//1/65536
			}

        	//test
        	float P1[3] = {0,0,0};
        	float P2[3] = {1,1,1};

        	ControlT(P1, P2, MechPara, IMUdata, TensionT);

			//COM_REG[0x300] = 0x01;//原来是上位机给命令，如果要测试可以用这个强行写命令
			// 0x01：表示电机速度控制，0x02：表示关节角度控制
			if (COM_REG[0x300] == 0x01)
			{
				// 发送速度指令
				for (i = 0; i < 10; i++)
					SetVelocity((MM + i), ((int16)COM_REG[0x50 + i]));//给MM+i号电机写COM_REG[]这么大的速度
			}
			else if(COM_REG[0x300] == 0x02)
			{
//				ControlShoulder(JointAngles, TargetJointAngles, motorVelForFeedback);				// 肩关节闭环控制，速度正为释放，负为收紧
//				motorVelForFeedback[4] = -ControlElbow(JointAngles[3], TargetJointAngles[3]);		// 肘关节闭环控制
//				motorVelForFeedback[5] = -motorVelForFeedback[4];
//				ControlWrist(JointAngles + 4, TargetJointAngles + 4, motorVelForFeedback + 6);	// 腕关节闭环控制
//				for (i = 0; i < 10; i++)
//				{
//					motorVelForFeedback[i] *= (9.5493 * 110.0);
//					// 电机速度约束
//					if(motorVelForFeedback[i] > 500.0)
//						motorVelForFeedback[i] = 500.0;
//					else if(motorVelForFeedback[i] < -500.0)
//						motorVelForFeedback[i] = -500.0;
//					SetVelocity((MM + i), motorVelForFeedback[i]);
//				}
			}
			else if(COM_REG[0x300] == 0x03)
			{
				// 数据传输模式
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
			/*
			else if(COM_REG[0x300] == 0x04)
			{
				if(COM_REG[0x100] == 0xBBBB)
				{
					float ja[7];
					for(i = 0; i < 7; ++i) 
						ja[i] = *(AddrTrajJointAngles[i] + TrajCounter);
					ControlShoulder(JointAngles, ja, motorVelForFeedback);						// 肩关节闭环控制，速度正为释放，负为收紧
					motorVelForFeedback[4] = -ControlElbow(JointAngles[3], ja[3]);				// 肘关节闭环控制
					ControlWrist(JointAngles + 4, ja + 4, motorVelForFeedback + 6);				// 腕关节闭环控制
					for (i = 0; i < 10; i++)
					{
						motorVelForFeedback[i] *= (9.5493 * 110.0);
						// 电机速度约束
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
						SetVelocity((MM + i), 0);	//停机					
				}				
			}
			*/
			else if(COM_REG[0x300] == 0x05)//reset IMU data
			{
				IMU1_SetOffset();
				IMU2_SetOffset();
				IMU3_SetOffset();
				COM_REG[0x300] = 0x00;
			}
			else
			{
				for (i = 0; i < 10; i++)
					SetVelocity((MM + i), 0);	//停机
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

		/* 0.5s 与上位机通讯 */
        if(IsTimer2Up())
        {
            //BlinkLED();			// 0.5秒闪烁LED
            SetLED(led);
            led ++;
            if(led > 15)
            	led = 0;
			ScicXmtStatus();	// 发送机器人状态
            ClrTimer2Flag();
        }
    }
}

void InitAllMotor(void)
{
	// 肩关节电机
	InitMotorModel(&MM[0], 1, 1, 1000.0, 10, ZeroDutyCycleArray[0]);	// 最大速度3000 rpm, 最大加速度 100 rpm/s = 0.5 rpm / 5ms;
	InitMotorModel(&MM[1], 2, 1, 1000.0, 10, ZeroDutyCycleArray[1]);
	InitMotorModel(&MM[2], 3, -1, 1000.0, 10, ZeroDutyCycleArray[2]);
	InitMotorModel(&MM[3], 4, -1, 1000.0, 10, ZeroDutyCycleArray[3]);
	// 肘关节电机
	InitMotorModel(&MM[4], 5, 1, 1000.0, 10, ZeroDutyCycleArray[4]);
	InitMotorModel(&MM[5], 6, -1, 1000.0, 10, ZeroDutyCycleArray[5]);
	// 腕关节电机
	InitMotorModel(&MM[6], 7, -1, 1000.0, 10, ZeroDutyCycleArray[6]);
	InitMotorModel(&MM[7], 8, -1, 1000.0, 10, ZeroDutyCycleArray[7]);
	InitMotorModel(&MM[8], 9,  1, 1000.0, 10, ZeroDutyCycleArray[8]);
	InitMotorModel(&MM[9], 10, 1, 1000.0, 10, ZeroDutyCycleArray[9]);
}

// 发送状态
void ScicXmtStatus(void)
{
	int i = 0;
	Uint16 checksum = 0;

	//Uint16 array[40];//IMU
	//Uint16 array[20];//TF
	Uint16 array[56];//2字节的帧头，1字节的数据长度，16字节的TF数据，36字节的IMU数据，1字节的checksum。 2+1+16+36+1=56.

	array[0] = 0xFF;
	array[1] = 0xFA;//帧头

	array[2] = 53;//数据长度，TF+IMU 16+36+1
	//array[2] = 17;//TF
	//array[2] = 37;//IMU

	//TF数据
	for(i = 0; i < 8; i++)
	{
		array[2 * i + 3] = tension[i] & 0xFF;
		array[2 * i + 4] = tension[i] >> 8;
		checksum += array[2 * i + 3] + array[2 * i + 4];
	}
//18

	//IMU数据
	for(i = 0; i < 9; i++)
	{
		array[18 + 4 * i + 1] = IMUhex[i] >> 24;
		array[18 + 4 * i + 2] = (IMUhex[i] >> 16) & 0x000000FF;
		array[18 + 4 * i + 3] = (IMUhex[i] >> 8) & 0x000000FF;
		array[18 + 4 * i + 4] = IMUhex[i] & 0x000000FF;
		checksum += array[18 + 4 * i + 1] + array[18 + 4 * i + 2] + array[18 + 4 * i + 3] + array[18 + 4 * i + 4];
	}

	//IMU
//	for(i = 0; i < 9; i++)
//	{
//		array[4 * i + 3] = IMUhex[i] >> 24;
//		array[4 * i + 4] = (IMUhex[i] >> 16) & 0x000000FF;
//		array[4 * i + 5] = (IMUhex[i] >> 8) & 0x000000FF;
//		array[4 * i + 6] = IMUhex[i] & 0x000000FF;
//		checksum += array[4 * i + 3] + array[4 * i + 4] + array[4 * i + 5] + array[4 * i + 6];
//	}

	//array[39] = checksum & 0xFF;//IMU
	//array[19] = checksum & 0xFF;//TF
	//checksum
	array[55] = checksum & 0xFF;//TF+IMU
	//ScicXmt(array, 20);//TF
	//ScicXmt(array, 40);//IMU
	ScicXmt(array, 56);//TF+IMU
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
