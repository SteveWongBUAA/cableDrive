/*
 * QP_1.c
 *��Ϸ�����
 *  Created on: 2015-7-10
 *      Author: stevewong
 */

#include "QP_1.h"
// QP.cpp : �������̨Ӧ�ó������ڵ㡣

#include "math.h"


#define  e1 0.1//0.1��ѭ����ֹ����
#define  e2 0.1//0.1��ѭ����ֹ����
#define  n 10//�����С����ѭ�������ѭ������
#define tl 2//���½�
#define tu 20//���Ͻ�
#define dfdxMin 0.001//0.001Сѭ����ֹ����
#define xMax 100//100��ɢ����
#define MAX_INLOOP_NUM 1000//Сѭ��������
#define MAX_OUTLOOP_NUM 10//��ѭ��������

/*
float J[3][4] = { 0.0000 , 111.7128, -135.5402, -0.0000,
84.4020, - 79.7948, - 96.8144,   47.1136,
- 0.5231, - 62.2949,   41.0156,   58.4505 };
float tt[3] = {500,500,800};
*/

float fun2(float x[], float* J, float* tt, float r)
{
	float f2;

	//f2 = pow(x[0], 2) - 3 * x[1] + pow(x[1], 2) - r*mylog(x[0] - 1) + pow(x[1], 2) / r;
	/*
	f20 = pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2) + pow(x[3], 2);
	//ʹ��log����
	//f210 = -r * (mylog(x[0] - tl) + mylog(tu - x[0]) + mylog(x[1] - tl) + mylog(tu - x[1]) + mylog(x[2] - tl) + mylog(tu - x[2]) + mylog(x[3] - tl) + mylog(tu - x[3]));
	//ʹ�õ���1/x
	f211 = r * (1/(x[0] - tl) + 1/(tu - x[0]) + 1/(x[1] - tl) + 1/(tu - x[1]) + 1/(x[2] - tl) + 1/(tu - x[2]) + 1/(x[3] - tl) + 1/(tu - x[3]));
	f22 = (pow((J[0] * x[0] + J[1] * x[1] + J[2] * x[2] + J[3] * x[3] - tt[0]), 2)
		+ pow((J[4] * x[0] + J[5] * x[1] + J[6] * x[2] + J[7] * x[3] - tt[1]), 2)
		+ pow((J[8] * x[0] + J[9] * x[1] + J[10] * x[2] + J[11] * x[3] - tt[2]), 2)) / sqrt(r);
	float f220 = pow((J[0] * x[0] + J[1] * x[1] + J[2] * x[2] + J[3] * x[3] - tt[0]), 2);
	float f221 = pow((J[4] * x[0] + J[5] * x[1] + J[6] * x[2] + J[7] * x[3] - tt[1]), 2);
	float f222 = pow((J[8] * x[0] + J[9] * x[1] + J[10] * x[2] + J[11] * x[3] - tt[2]), 2);
	f2 = f20 + f211 + f22;*/
	f2 = pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2) + pow(x[3], 2)
		+ r * (1 / (x[0] - tl) + 1 / (tu - x[0]) + 1 / (x[1] - tl) + 1 / (tu - x[1]) + 1 / (x[2] - tl) + 1 / (tu - x[2]) + 1 / (x[3] - tl) + 1 / (tu - x[3]))
		+ (pow((J[0] * x[0] + J[1] * x[1] + J[2] * x[2] + J[3] * x[3] - tt[0]), 2)
		+ pow((J[4] * x[0] + J[5] * x[1] + J[6] * x[2] + J[7] * x[3] - tt[1]), 2)
		+ pow((J[8] * x[0] + J[9] * x[1] + J[10] * x[2] + J[11] * x[3] - tt[2]), 2)) / sqrt(r);
	//printf("x0 = %f\nx1 = %f\nx2 = %f\nx3 = %f\n", x[0], x[1], x[2], x[3]);
	//printf("�ͷ�����f = %f\nϵ��r = %f\nĿ�꺯��f20 = %f\n����ʽԼ��f21 = %f\n��ʽԼ��f22 = %f\n",f2, r,f20, f211, f22);


	return f2;
}

//���ι滮��J*t=tt,tl<tt<tu,min(tt^2),��ɢ����-1,��������Ŀ�꺯��ֵ��tested on one jacob matrix
float QP(float* J,float* tt,float* x)
{
	int i,j;
	float r,r_alpha, f,fpast, c, c_alpha, dx, df,tmpx0,tmpx1,tmpx2,tmpx3,dfdx0,dfdx1,dfdx2,dfdx3,alpha;
	x[0] = 3, x[1] = 3; x[2] = 3; x[3] = 3;

	alpha = 0.001;//��ʼalpha
	r = 100;//���ӳͷ����ʼϵ��100
	c = 0.1;//���ӳͷ����½�����0.1
	r_alpha = 0.1;//ѧϰ�ʳ�ʼϵ��0.1
	c_alpha = 0.3;//ѧϰ���½�����0.3

	f = fun2(x,J,tt,r);
	for (i = 1;; i++)
	{
		r = c*r;
		r_alpha = c_alpha * r_alpha;

		tmpx0 = x[0];
		tmpx1 = x[1];
		tmpx2 = x[2];
		tmpx3 = x[3];
		//printf("\n��%d�δ�ѭ����ʼ������alpha = %f\n", i, alpha * r_alpha);
		for (j = 1; j < MAX_INLOOP_NUM; j++)
		{
			//ʹ�õ���
			dfdx0 = 2 * tmpx0
				+ (2 * J[0] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3)
				+ 2 * J[4] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3)
				+ 2 * J[8] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3)) / sqrt(r)
				- r*(1 / (tl - tmpx0) / (tl - tmpx0) - 1 / (tu - tmpx0) / (tu - tmpx0));
//
//			dfdx0 =  2 * J[0] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3);
//			dfdx0 += 2 * J[4] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3);
//			dfdx0 += 2 * J[8] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3);
//			dfdx0 /= sqrt(r);
//			dfdx0 -=  r*(1 / (tl - tmpx0) / (tl - tmpx0) + 1 / (tu - tmpx0) / (tu - tmpx0));
//			dfdx0 += 2 * tmpx0;

			dfdx1 = 2 * tmpx1
				+ (2 * J[1] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3)
				+ 2 * J[5] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3)
				+ 2 * J[9] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3)) / sqrt(r)
				- r*(1 / (tl - tmpx1) / (tl - tmpx1) - 1 / (tu - tmpx1) / (tu - tmpx1));

//			dfdx1 =  2 * J[1] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3);
//			dfdx1 += 2 * J[5] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3);
//			dfdx1 += 2 * J[9] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3);
//			dfdx1 /= sqrt(r);
//			dfdx1 -=  r*(1 / (tl - tmpx1) / (tl - tmpx1) + 1 / (tu - tmpx1) / (tu - tmpx1));
//			dfdx1 += 2 * tmpx1;

			dfdx2 = 2 * tmpx2
				+ (2 * J[2] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3)
				+ 2 * J[6] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3)
				+ 2 * J[10] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3)) / sqrt(r)
				- r*(1 / (tl - tmpx2) / (tl - tmpx2) - 1 / (tu - tmpx2) / (tu - tmpx2));
//
//			dfdx2 =  2 * J[2] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3);
//			dfdx2 += 2 * J[6] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3);
//			dfdx2 += 2 * J[10] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3);
//			dfdx2 /= sqrt(r);
//			dfdx2 -=  r*(1 / (tl - tmpx2) / (tl - tmpx2) + 1 / (tu - tmpx2) / (tu - tmpx2));
//			dfdx2 += 2 * tmpx2;

			dfdx3 = 2 * tmpx3
				+ (2 * J[3] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3)
				+ 2 * J[7] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3)
				+ 2 * J[11] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3)) / sqrt(r)
				- r*(1 / (tl - tmpx3) / (tl - tmpx3) - 1 / (tu - tmpx3) / (tu - tmpx3));

//			dfdx3 =  2 * J[3] * (J[0] * tmpx0 - tt[0] + J[1] * tmpx1 + J[2] * tmpx2 + J[3] * tmpx3);
//			dfdx3 += 2 * J[7] * (J[4] * tmpx0 - tt[1] + J[5] * tmpx1 + J[6] * tmpx2 + J[7] * tmpx3);
//			dfdx3 += 2 * J[11] * (J[8] * tmpx0 - tt[2] + J[9] * tmpx1 + J[10] * tmpx2 + J[11] * tmpx3);
//			dfdx3 /= sqrt(r);
//			dfdx3 -=  r*(1 / (tl - tmpx3) / (tl - tmpx3) + 1 / (tu - tmpx3) / (tu - tmpx3));
//			dfdx3 += 2 * tmpx3;

			tmpx0 = tmpx0 - alpha * r_alpha * dfdx0;
			tmpx1 = tmpx1 - alpha * r_alpha * dfdx1;
			tmpx2 = tmpx2 - alpha * r_alpha * dfdx2;
			tmpx3 = tmpx3 - alpha * r_alpha * dfdx3;
			if (tmpx0 > xMax || tmpx1 > xMax || tmpx2 > xMax || tmpx3 > xMax
			 ||	tmpx0 < -xMax  || tmpx1 < -xMax || tmpx2 < -xMax || tmpx3 < -xMax)//��ɢ��
			{
				//��ɢ
				return -1;
			}

			if ((alpha * r_alpha * dfdx0) < dfdxMin && (alpha * r_alpha * dfdx1) < dfdxMin && (alpha * r_alpha * dfdx2) < dfdxMin && (alpha * r_alpha * dfdx3) < dfdxMin
			&& (alpha * r_alpha * dfdx0) > -dfdxMin && (alpha * r_alpha * dfdx1) > -dfdxMin && (alpha * r_alpha * dfdx2) > -dfdxMin && (alpha * r_alpha * dfdx3) > -dfdxMin	)//������
			{
				//Сѭ������
				break;
			}
		}

//		if (j == MAX_INLOOP_NUM)
//		{
//			//printf("#################Сѭ��%d����δ��ɣ���ֹ##############\n",MAX_INLOOP_NUM);
//		}

		dx = sqrt(pow((x[0] - tmpx0), 2) + pow((x[1] - tmpx1), 2) + pow((x[2] - tmpx2), 2) + pow((x[3] - tmpx3), 2));//ǰ�����δ������x�仯ֵ
		fpast = fun2(x,J,tt,r);//ǰһ�δ�����ĺ���ֵ

		x[0] = tmpx0;//����x
		x[1] = tmpx1;
		x[2] = tmpx2;
		x[3] = tmpx3;
		f = fun2(x, J,tt,r);//��һ�δ�����ĺ���ֵ

		df = fabs((fpast - f) / fpast);
		if (dx<e1)
		{
			if (df<e2) break;
		}
		if (i >= MAX_OUTLOOP_NUM  - 1 )
		{
			//��ѭ��MAX_OUTLOOPNUM����δ����
			break;
		}
	}

	return f;
}

