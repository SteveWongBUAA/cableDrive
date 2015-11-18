#ifndef ECAN_H_
#define ECAN_H_

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

void InitMyEcan();

//void mailbox_read(int16 i);

Uint32 InverseOrder32(Uint32 a);


//void IMU_reset();

void IMU_read_CANopen(float32 *IMUdata);

Uint32 IMU1_GotoCmd();
Uint32 IMU2_GotoCmd();
Uint32 IMU3_GotoCmd();//进入命令模式，成功返回1，失败返回0

void IMU_GotoStream();

Uint32 IMU1_read_LpCan(Uint32 *IMUhex,float32 *IMUdata);
Uint32 IMU2_read_LpCan(Uint32 *IMUhex,float32 *IMUdata);
Uint32 IMU3_read_LpCan(Uint32 *IMUhex,float32 *IMUdata);//读取Euler角，IMUdata[0]-[9]代表第一个IMU的欧拉角X，Y,Z、第二个IMU的欧拉角X,Y,Z,第三个IMU的欧拉角XYZ

Uint32 IMU1_SetOffset();
Uint32 IMU2_SetOffset();
Uint32 IMU3_SetOffset();
#endif /*ECAN_H_*/
