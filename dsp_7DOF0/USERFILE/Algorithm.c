#include "Algorithm.h"
#include <math.h>

#define PI 3.141593
/* 所有距离单位为mm，角度单位为rad */

// 肩关节参数
#define SH_H 176.0
#define SH_P 176.0
#define SH_D 20.0
#define SH_L 180.0

float SHOULDER_P1[3] = {SH_P * 0.707107, 0, 0};
float SHOULDER_P2[3] = {0, -SH_P * 0.707107, 0};
float SHOULDER_P3[3] = {-SH_P * 0.707107, 0, 0};
float SHOULDER_P4[3] = {0, SH_P * 0.707107, 0};

float SHOULDER_B1[3] = {(SH_L - SH_D) * 0.707107, -(SH_L - SH_D) * 0.707107, -SH_H};
float SHOULDER_B2[3] = {SH_L * 0.707107, -SH_L * 0.707107, -SH_H};
float SHOULDER_B3[3] = {-(SH_L - SH_D) * 0.707107, (SH_L - SH_D) * 0.707107, -SH_H};
float SHOULDER_B4[3] = {-SH_L * 0.707107, SH_L * 0.707107, -SH_H};

// 腕关节参数
#define WR_H 120.0
#define WR_P 120.0
#define WR_D 20.0
#define WR_L 120.0

const float WRIST_P1[3] = {WR_P * 0.707107, 0, 0};
const float WRIST_P2[3] = {0, -WR_P * 0.707107, 0};
const float WRIST_P3[3] = {-WR_P * 0.707107, 0, 0};
const float WRIST_P4[3] = {0, WR_P * 0.707107, 0};

const float WRIST_B1[3] = {(WR_L - WR_D) * 0.707107, -(WR_L - WR_D) * 0.707107, -WR_H};
const float WRIST_B2[3] = {WR_L * 0.707107, -WR_L * 0.707107, -WR_H};
const float WRIST_B3[3] = {-(WR_L - WR_D) * 0.707107, (WR_L - WR_D) * 0.707107, -WR_H};
const float WRIST_B4[3] = {-WR_L * 0.707107, WR_L * 0.707107, -WR_H};

const float LAMBDA = 1.0;	// 20

// 电机驱动轮半径的倒数
const float INV_MOTOR_RADIUS = 0.04;

void Normliazation(float *Vec);

void Matrix33Mult(float *A, float* B, float* res)
{
	res[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	res[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	res[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
	res[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	res[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	res[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
	res[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	res[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	res[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

void EA2SO3(float32 gamma, float32 beta, float32 alpha, float32 *res)
{
	float sg, cg, sb, cb, sa, ca;
	sg = sin(gamma);
	cg = cos(gamma);
	sb = sin(beta);
	cb = cos(beta);
	sa = sin(alpha);
	ca = cos(alpha);
	res[0] = cb * cg;
	res[1] = cg * sa * sb - ca * sg;
	res[2] = ca * cg * sb + sa * sg;
	res[3] = cb * sg;
	res[4] = ca * cg + sa * sb * sg;
	res[5] = ca * sb * sg - cg * sa;
	res[6] = -sb;
	res[7] = cb * sa;
	res[8] = ca * cb;
}

void MatMultVec(const float *Mat,const float *Vec, float *res)
{
	res[0] = Mat[0] * Vec[0] + Mat[1] * Vec[1] + Mat[2] * Vec[2];
	res[1] = Mat[3] * Vec[0] + Mat[4] * Vec[1] + Mat[5] * Vec[2];
	res[2] = Mat[6] * Vec[0] + Mat[7] * Vec[1] + Mat[8] * Vec[2];
}

void VecMultMat(const float *Vec, const float *Mat, float *res)
{
	res[0] = Mat[0] * Vec[0] + Mat[3] * Vec[1] + Mat[6] * Vec[2];
	res[1] = Mat[1] * Vec[0] + Mat[4] * Vec[1] + Mat[7] * Vec[2];
	res[2] = Mat[2] * Vec[0] + Mat[5] * Vec[1] + Mat[8] * Vec[2];
}

void VecSub(const float *Vec1, const float *Vec2, float *res)
{
	res[0] = Vec1[0] - Vec2[0];
	res[1] = Vec1[1] - Vec2[1];
	res[2] = Vec1[2] - Vec2[2];
}

void VecCross(const float *Vec1, const float *Vec2, float *res)
{
	res[0] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
	res[1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
	res[2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];
}
// 0 - 肩关节；1 - 腕关节
void SolveD(float gamma, float beta, float alpha, float *D, int WrOrSh)
{
	float u1[3], u2[3], u3[3], u4[3];
	float R[9];
	if (WrOrSh == 0)	// 肩关节
	{
		EA2SO3(gamma, beta, alpha, R);
		MatMultVec(R, SHOULDER_P1, u1);
		MatMultVec(R, SHOULDER_P2, u2);
		MatMultVec(R, SHOULDER_P3, u3);
		MatMultVec(R, SHOULDER_P4, u4);
		VecSub(u1, SHOULDER_B1, u1);
		VecSub(u2, SHOULDER_B2, u2);
		VecSub(u3, SHOULDER_B3, u3);
		VecSub(u4, SHOULDER_B4, u4);
		Normliazation(u1);
		Normliazation(u2);
		Normliazation(u3);
		Normliazation(u4);
		VecCross(SHOULDER_B1, u1, D);
		VecCross(SHOULDER_B2, u2, D + 3);
		VecCross(SHOULDER_B3, u3, D + 6);
		VecCross(SHOULDER_B4, u4, D + 9);		
	}
	else
	{
		EA2SO3(gamma, beta, alpha, R);
		MatMultVec(R, WRIST_P1, u1);
		MatMultVec(R, WRIST_P2, u2);
		MatMultVec(R, WRIST_P3, u3);
		MatMultVec(R, WRIST_P4, u4);
		VecSub(u1, WRIST_B1, u1);
		VecSub(u2, WRIST_B2, u2);
		VecSub(u3, WRIST_B3, u3);
		VecSub(u4, WRIST_B4, u4);
		Normliazation(u1);
		Normliazation(u2);
		Normliazation(u3);
		Normliazation(u4);
		VecCross(WRIST_B1, u1, D);
		VecCross(WRIST_B2, u2, D + 3);
		VecCross(WRIST_B3, u3, D + 6);
		VecCross(WRIST_B4, u4, D + 9);		
	}
	
}

void SolveH(float gamma, float beta, float *H)
{
	float sg = sin(gamma);
	float cg = cos(gamma);
	float sb = sin(beta);
	float cb = cos(beta);
	H[0] = 0.0;
	H[1] = -sg;
	H[2] = cb * cg;
	H[3] = 0.0;
	H[4] = cg;
	H[5] = cb * sg;
	H[6] = 1.0;
	H[7] = 0.0;
	H[8] = -sb;
}

void Normliazation(float *Vec)
{
	float res = sqrt(Vec[0] * Vec[0] + Vec[1] * Vec[1] + Vec[2] * Vec[2]);
	if((res < EPS) && (res > -EPS))
	{
		Vec[0] = 0.0;
		Vec[1] = 0.0;
		Vec[2] = 0.0;
	}
	else
	{
		Vec[0] = Vec[0] / res;
		Vec[1] = Vec[1] / res;
		Vec[2] = Vec[2] / res;
	}
}

void ControlShoulder(const float* ja_curt, const float* ja_tart, float* motorVel)
{
	float H[9];
	float D[12];
	float delta_ja[3];
	float tmp[3];
	VecSub(ja_curt, ja_tart, delta_ja); // delta_q
	SolveH(ja_curt[0], ja_curt[1], H);	// H
	SolveD(ja_curt[0], ja_curt[1], ja_curt[2], D, 0);	// D
	MatMultVec(H, delta_ja, tmp); // tmp = Hq
	motorVel[0] = -(D[0] * tmp[0] + D[1] * tmp[1] + D[2] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	motorVel[1] = -(D[3] * tmp[0] + D[4] * tmp[1] + D[5] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	motorVel[2] = -(D[6] * tmp[0] + D[7] * tmp[1] + D[8] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	motorVel[3] = -(D[9] * tmp[0] + D[10] * tmp[1] + D[11] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	return ;
}

void ControlWrist(const float* ja_curt, const float* ja_tart, float* motorVel)
{
	float H[9];
	float D[12];
	float delta_ja[3];
	float tmp[3];
	VecSub(ja_curt, ja_tart, delta_ja); // delta_q
	SolveH(ja_curt[0], ja_curt[1], H);	// H
	SolveD(ja_curt[0], ja_curt[1], ja_curt[2], D, 1);	// D
	MatMultVec(H, delta_ja, tmp); // tmp = Hq
	motorVel[0] = -(D[0] * tmp[0] + D[1] * tmp[1] + D[2] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	motorVel[1] = -(D[3] * tmp[0] + D[4] * tmp[1] + D[5] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	motorVel[2] = -(D[6] * tmp[0] + D[7] * tmp[1] + D[8] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	motorVel[3] = -(D[9] * tmp[0] + D[10] * tmp[1] + D[11] * tmp[2]) * LAMBDA * INV_MOTOR_RADIUS;
	return ;
}

float ControlElbow(float ja_curt, float ja_tart)
{
	return -(ja_curt - ja_tart) * 1.2 * 38.0 / 25.0;
}

// 生成波形频率为f，采样频率为fs，幅值为A，初始相位为phase, 数据长度为len，结果保存于res
void GenerateSineWave(float *res, float A, float fs, float f, float phase, Uint16 len)
{
	Uint16 i = 0;
	float tmp = 2 * PI * f / fs;
	for(; i < len; i++)
	{
		res[i] = A * sin(tmp * i + phase);
	}
}
// 生成波形频率为f，采样频率为fs，幅值为A，初始相位为phase, 数据长度为len，结果保存于res
void GenerateSquareWave(float *res, float h, float l, float fs, float f, Uint16 len)
{
	Uint16 i = 0;
	Uint16 N = fs / f;
	for(; i < len; i++)
	{
		int j = (i / N) % 2;
		res[i] = (j == 0) ? h : l;
	}
}

// No more

