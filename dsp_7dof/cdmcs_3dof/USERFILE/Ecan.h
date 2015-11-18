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
Uint32 IMU3_GotoCmd();//��������ģʽ���ɹ�����1��ʧ�ܷ���0

void IMU_GotoStream();

Uint32 IMU1_read_LpCan(Uint32 *IMUhex,float32 *IMUdata);
Uint32 IMU2_read_LpCan(Uint32 *IMUhex,float32 *IMUdata);
Uint32 IMU3_read_LpCan(Uint32 *IMUhex,float32 *IMUdata);//��ȡEuler�ǣ�IMUdata[0]-[9]�����һ��IMU��ŷ����X��Y,Z���ڶ���IMU��ŷ����X,Y,Z,������IMU��ŷ����XYZ

Uint32 IMU1_SetOffset();
Uint32 IMU2_SetOffset();
Uint32 IMU3_SetOffset();
#endif /*ECAN_H_*/
