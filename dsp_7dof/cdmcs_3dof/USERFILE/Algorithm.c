#include "Algorithm.h"
#include <math.h>

#define PI 3.141593
/* 所有距离单位为mm，角度单位为rad */

/*
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
*/
const float LAMBDA = 1.0;	// 20

// 电机驱动轮半径的倒数
const float INV_MOTOR_RADIUS = 0.04;

void Normalization(float *Vec);

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

void EA2SO3(const float32 gamma,const float32 beta,const float32 alpha, float32 *res)//gamma-Z,beta-Y,alpha-X,IMU输出顺序是XYZ.欧拉角旋转顺序是ZYX
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

void VecAssigned(float* dst, const float* org)
{
	int i = 0;
	for(;i<3;i++)
		dst[i] = org[i];
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

void VecAdd(const float *Vec1, const float *Vec2, float *res)
{
	res[0] = Vec1[0] + Vec2[0];
	res[1] = Vec1[1] + Vec2[1];
	res[2] = Vec1[2] + Vec2[2];
}

void VecCross(const float *Vec1, const float *Vec2, float *res)
{
	res[0] = Vec1[1] * Vec2[2] - Vec1[2] * Vec2[1];
	res[1] = Vec1[2] * Vec2[0] - Vec1[0] * Vec2[2];
	res[2] = Vec1[0] * Vec2[1] - Vec1[1] * Vec2[0];
}

float VecDot(const float *Vec1, const float *Vec2)
{
	return (Vec1[0] * Vec2[0] + Vec1[1] * Vec2[1] + Vec1[2] * Vec2[2]);
}
// 0 - 肩关节；1 - 腕关节
/*
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
		Normalization(u1);
		Normalization(u2);
		Normalization(u3);
		Normalization(u4);
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
		Normalization(u1);
		Normalization(u2);
		Normalization(u3);
		Normalization(u4);
		VecCross(WRIST_B1, u1, D);
		VecCross(WRIST_B2, u2, D + 3);
		VecCross(WRIST_B3, u3, D + 6);
		VecCross(WRIST_B4, u4, D + 9);		
	}
	
}
*/
/*
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
*/
void Normalization(float *Vec)
{
	float res = sqrt(Vec[0] * Vec[0] + Vec[1] * Vec[1] + Vec[2] * Vec[2]);
	if((res < EPS) && (res > -EPS))
	{
		Vec[0] = 0.0;
		Vec[2] = 0.0;
	}
	else
	{
		Vec[0] = Vec[0] / res;
		Vec[1] = Vec[1] / res;
		Vec[2] = Vec[2] / res;
	}
}

/*
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

*/

//给定两个R矩阵，和测得的l1,l2,（l1,l2均为3*1向量即[0;0;l1]）求末端轨迹P
void SolveP(const float* R1, const float* R2, const float* l1, const float* l2, float *P)
{
	float M1[3];
	float M2[3];
	MatMultVec(R1, l1, M1);
	MatMultVec(R2, l2, M2);
	VecAdd(M1,M2,P);
}

//求点到直线的垂足，返回点到垂足的距离。输入直线的两端点，当前点，输出垂足点，返回点到垂足的距离.Tested.
float FootPoint(const float* Posi1, const float* Posi2, const float* PosiCur, float* fp)
{
	float SE[3];
	float SC[3];
	float CE[3];
	int i;
	VecSub(Posi2,Posi1,SE);
	VecSub(PosiCur,Posi1,SC);
	VecSub(Posi2,PosiCur,CE);
	if(VecDot(SE,SC) < 0)
		for(i=0;i<3;i++)
			fp[i] = Posi1[i];

	else if(VecDot(SE,CE) < 0)
		for(i=0;i<3;i++)
			fp[i] = Posi2[i];
	else
	{
		fp[1] = ( (SE[0]*SE[0]/SE[1]+SE[2]*SE[2]/SE[1])*Posi1[1] - SE[0]*(Posi1[0]-PosiCur[0]) + SE[1]*PosiCur[1] - SE[2]*(Posi1[2]-PosiCur[2]) ) / ( SE[0]*SE[0]/SE[1]+SE[1]+SE[2]*SE[2]/SE[1] );
		fp[0] = SE[0]/SE[1]*(fp[1]-Posi1[1]) + Posi1[0];
		fp[2] = SE[2]/SE[1]*(fp[1]-Posi1[1]) + Posi1[2];
	}

	return sqrt((fp[0]-PosiCur[0])*(fp[0]-PosiCur[0]) + (fp[1]-PosiCur[1])*(fp[1]-PosiCur[1]) + (fp[2]-PosiCur[2])*(fp[2]-PosiCur[2]));
}

//构造末端所需要的力F，输入起始点P，垂足Pp,P与Pp的距离d，控制参数an，输出力F
void MakeF(const float *P, const float *Pp, const float d, const float an, float* F)
{
	float para = an * log(1+d);
	float Fn[3] = {0,0,0};
	float Ft[3] = {0,0,0};
	Fn[0] = para * (Pp[0]-P[0]);
	Fn[1] = para * (Pp[1]-P[1]);
	Fn[2] = para * (Pp[2]-P[2]);
	VecAdd(Fn,Ft,F);
}

//末端力转化到关节力矩,输入末端轨迹R1,R2,l1,l2,输出ts，te
void TransformF(const float* R1, const float* R2, const float* l1,const float* l2, const float *F, float *ts, float *te)
{
	float P1[3];
	float P2[3];
	float P[3];
	MatMultVec(R1, l1, P1);
	MatMultVec(R2, l2, P2);
	VecAdd(P1,P2,P);
	VecCross(F,P,ts);

	float tmp[3];
	VecCross(F,P2,tmp);
	te[0] = -VecDot(tmp,R1);
	te[1] = 0;
	te[2] = 0;

}

//求中间向量
void SolveTmpVec(const float r, const float alpha, const float d, float* res)
{
	res[0] = r * cos(alpha);
	res[1] = r * sin(alpha);
	res[2] = d;
}
//求雅克比矩阵。
void SolveJacob(const float* MechPara, const float* R1, const float* R2, float* jacob_s, float* jacob_e)
{
	float A1[3],A2[3],A3[3],A4[3],B1[3],B2[3],B3[3],B4[3],B5[3],B6[3],C1[3],C2[3];
	const float l1 = MechPara[0];
	//const float l2 = MechPara[1];
	const float d1 = MechPara[2];
	const float r1 = MechPara[3];
	const float d2 = MechPara[4];
	const float r2 = MechPara[5];
	const float d3 = MechPara[6];
	const float r3 = MechPara[7];
	const float alphaa1 = MechPara[8];
	const float psta2 = MechPara[9];
	const float psta3 = MechPara[10];
	const float alphaa4 = MechPara[11];
	const float alphab1 = MechPara[12];
	const float alphab2 = MechPara[13];
	const float alphab3 = MechPara[14];
	const float alphab4 = MechPara[15];
	const float alphab5 = MechPara[16];
	const float alphab6 = MechPara[17];
	const float alphac1 = MechPara[18];
	const float alphac2 = MechPara[19];
	SolveTmpVec(r1,alphaa1,d1,A1);
	SolveTmpVec(r1,alphaa4,d1,A4);
	SolveTmpVec(r2,alphab1,d2,B1);
	SolveTmpVec(r2,alphab2,d2,B2);
	SolveTmpVec(r2,alphab3,d2,B3);
	SolveTmpVec(r2,alphab4,d2,B4);
	SolveTmpVec(r2,alphab5,d2,B5);
	SolveTmpVec(r2,alphab6,d2,B6);
	SolveTmpVec(r3,alphac1,d3,C1);
	SolveTmpVec(r3,alphac2,d3-84,C2);
	A2[0] = psta2; A2[1] = r1; A2[2] = d1;
	A3[0] = psta3; A3[1] = -r1; A3[2] = d1;
	float A[3] = {0,0,0};
	float B[3];
	float tmpl[3];
	tmpl[0] = tmpl[1] = 0; tmpl[2] = l1;
	MatMultVec(R1,tmpl,B);

	float B1_r[3], B2_r[3], B3_r[3], B4_r[3], B5_r[3], B6_r[3], C1_r[3], C2_r[3], ld1[3], ld2[3], ld3[3], ld4[3], ld5[3], ld6[3];
	MatMultVec(R1,B1,B1_r);
	MatMultVec(R1,B2,B2_r);
	MatMultVec(R1,B3,B3_r);
	MatMultVec(R1,B4,B4_r);
	MatMultVec(R1,B5,B5_r);
	MatMultVec(R1,B6,B6_r);

	MatMultVec(R2,C1,C1_r);
	MatMultVec(R2,C2,C2_r);
	VecAdd(B,C1_r,C1_r);
	VecAdd(B,C2_r,C2_r);

	VecSub(B1_r,A1,ld1);
	VecSub(B2_r,A2,ld2);
	VecSub(B3_r,A3,ld3);
	VecSub(B4_r,A4,ld4);
	VecSub(C1_r,B5,ld5);
	VecSub(C2_r,B6,ld6);
	Normalization(ld1);
	Normalization(ld2);
	Normalization(ld3);
	Normalization(ld4);
	Normalization(ld5);
	Normalization(ld6);

	float B1_r_A[3], B2_r_A[3], B3_r_A[3], B4_r_A[3], C1_r_B[3], C2_r_B[3];
	VecSub(B1_r,A,B1_r_A);
	VecSub(B2_r,A,B2_r_A);
	VecSub(B3_r,A,B3_r_A);
	VecSub(B4_r,A,B4_r_A);
	VecSub(C1_r,B,C1_r_B);
	VecSub(C2_r,B,C2_r_B);

	float tmp[3];
	VecCross(B1_r_A,ld1,tmp);
	jacob_s[0] = tmp[0]; jacob_s[4] = tmp[1]; jacob_s[8] = tmp[2];
	VecCross(B2_r_A,ld2,tmp);
	jacob_s[1] = tmp[0]; jacob_s[5] = tmp[1]; jacob_s[9] = tmp[2];
	VecCross(B3_r_A,ld3,tmp);
	jacob_s[2] = tmp[0]; jacob_s[6] = tmp[1]; jacob_s[10] = tmp[2];
	VecCross(B4_r_A,ld4,tmp);
	jacob_s[3] = tmp[0]; jacob_s[7] = tmp[1]; jacob_s[11] = tmp[2];
	VecCross(C1_r_B,ld5,tmp);
	jacob_e[0] = tmp[0]; jacob_e[2] = tmp[1]; jacob_e[4] = tmp[2];
	VecCross(C2_r_B,ld6,tmp);
	jacob_e[1] = tmp[0]; jacob_e[3] = tmp[1]; jacob_e[5] = tmp[2];

}

int ControlT(const float* P1, const float* P2, const float* MechPara, const float* IMUdata, const float* TensionT)
{
	float R1[9],R2[9];
	float l1[3],l2[3];
	l1[0]=l1[1]=0;l1[2]=MechPara[0];
	l2[0]=l1[2]=0;l2[2]=MechPara[1];
	if(IMUdata[0]==4||IMUdata[1]==4||IMUdata[2]==4||IMUdata[3]==4||IMUdata[4]==4||IMUdata[5]==4)//IMU数据错误
		return -1;
	EA2SO3(*(IMUdata+2),*(IMUdata+1),*(IMUdata),R1);//求矩阵R1，R2
	EA2SO3(*(IMUdata+5),*(IMUdata+4),*(IMUdata+3),R2);
	float P[3];
	SolveP(R1, R2, l1, l2, P);//求腕部轨迹P
	float d;
	float fp[3];
	d = FootPoint(P1, P2, P, fp);//求P到轨迹P1P2的距离d与垂足fp
	const float an = 1;//构造末端力的参数
	float F[3];
	MakeF(P, fp, d, an, F);//构造末端所需的力F
	float taos[3],taoe[3];
	TransformF(R1, R2, l1, l2, F, taos, taoe);//末端力转化到关节力矩
	float jacob_s[12],jacob_e[6];
	SolveJacob(MechPara, R1, R2, jacob_s, jacob_e);//求Jacob矩阵
	float ts[4];
	if(QP(jacob_s,taos,ts) == -1)//求优发散了
	{
		return -1;
	}
	else
	{
		return 1;
//		if(QP(jacob_e,taoe,te == -1))//求优发散了
//			return -1;
//		else//终于求出来最优解了
//		{
//
//		}
	}
}
// No more

