#ifndef _COPLEY_CONTROL_H_
#define _COPLEY_CONTROL_H_

typedef struct _MotorModel
{
	int index;				// ������
	int dir;				// �������
	float cmdVelocity;		// ָ���ٶ�
	float velocity;			// ʵ���ٶ�
	float preVelocity;		// �ϴε�ָ���ٶ�
	float maxVel;			// ����ٶ�(����)
	float dutyCycle;		// ʵ��ռ�ձ�
	float zeroDutyCycle;	// ���ٶȶ�Ӧռ�ձ�
	float maxAcc;			// �����ٶ�(����)
}MotorModel;

float SetVelocity(MotorModel *mm, float vel);
int InitMotorModel(MotorModel *mm, int index, int dir, float maxVel, float maxAcc, float zeroDutyCycle);

#endif
// end of file
