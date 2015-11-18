#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "QP_1.h"

#define EPS 1e-6			// 精度
#define RPM 19.09859		// 1rad/s 转换为 rpm
#define SRR 110				// 电机减速比

//void Matrix33Mult(float *A, float* B, float* res);
//void EA2SO3(float gamma, float beta, float alpha, float *res);
//void ControlShoulder(const float* ja_curt, const float* ja_tart, float* motorVel);
//void ControlWrist(const float* ja_curt, const float* ja_tart, float* motorVel);
//float ControlElbow(float ja_curt, float ja_tart);

//void GenerateSineWave(float *res, float A, float fs, float f, float phase, Uint16 len);
//void GenerateSquareWave(float *res, float h, float l, float fs, float f, Uint16 len);
//float FootPoint(const float* Posi1, const float* Posi2, const float* PosiCur, float* fp);
//void SolveJacob(const float* MechPara, const float* R1, const float* R2, float* jacob_s, float* jacob_e);
//void EA2SO3(float32 gamma, float32 beta, float32 alpha, float32 *res);
int ControlT(const float* P1, const float* P2, const float* MechPara, const float* IMUdata, const float* TensionT);

#endif /*ALGORITHM_H_*/

