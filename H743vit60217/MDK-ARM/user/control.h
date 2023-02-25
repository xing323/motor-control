#ifndef _CONTROL_H
#define _CONTROL_H
//���1��λ�ýṹ��
typedef struct{
	float err;                         //���
	float last_err;                    //�ϴ����
	float d_err;                       //���ĵ���
	float Kp,Ki,Kd;                    //���� ���� ΢��ϵ��
	float I_err;                       //���Ļ���
	float PWM_value;                   //�����ٶ�PWM
	unsigned int U_From_Pc;            //MATLAB����Ŀ�����
	unsigned int U;                    //�㷨���տ������������
	int PWM_TongBuXiuBu;               //���ͬ���޲�
	
}control_pid;


//���Ƶ�������ṹ��
extern control_pid Motor1pid ,Motor2pid, Motor3pid, Motor4pid,Motor5pid, Motor6pid, Motor1pid_sita, Motor2pid_sita, Motor3pid_sita, Motor4pid_sita, Motor5pid_sita, Motor6pid_sita;

void Control_Init(void);
void Motor_x_control_pid_Init(control_pid * Motorx);
void Pid_Set(control_pid * Motorx,float Kp,float Ki,float Kd);
float Motor_x_control_pid_PWM(control_pid * Motorx, float PV,float CV);
void forward_Value(float* data_In, float* data_Out,int length);

extern float motor1_followed_curve[5000];
#endif

