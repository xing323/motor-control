#include "main.h"
#include "control.h"

control_pid Motor1pid ,Motor2pid, Motor3pid, Motor4pid,Motor5pid, Motor6pid, Motor1pid_sita, Motor2pid_sita, Motor3pid_sita, Motor4pid_sita, Motor5pid_sita, Motor6pid_sita;

void Control_Init(void)
{
	Motor_x_control_pid_Init(&Motor1pid);
	Motor_x_control_pid_Init(&Motor2pid);
	Motor_x_control_pid_Init(&Motor3pid);
	Motor_x_control_pid_Init(&Motor4pid);
	Motor_x_control_pid_Init(&Motor5pid);
	Motor_x_control_pid_Init(&Motor6pid);
}

//////////////////////////
//初始化
void Motor_x_control_pid_Init(control_pid * Motorx)
{
	Motorx->err=0;         //速度误差
	Motorx->last_err=0;
	Motorx->I_err=0;       //误差积分值
	Motorx->d_err=0;       //误差的导数
	Motorx->Kp=1;
	Motorx->Ki=0;
	Motorx->Kd=0;
	Motorx->PWM_value=0;
}
//设置PID参数
void Pid_Set(control_pid * Motorx,float Kp,float Ki,float Kd)
{
	Motorx->Kp=Kp;
	Motorx->Ki=Ki;
	Motorx->Kd=Kd;
}
//PID计算过程，返回PWM值
float Motor_x_control_pid_PWM(control_pid * Motorx, float PV,float CV)
{
	Motorx->err=PV-CV;
	Motorx->d_err=Motorx->err-Motorx->last_err;
	
	if(((Motorx->err)<2)&&(Motorx->err>-2))
	{
		Motorx->I_err+=Motorx->err;
		if(Motorx->I_err>10000)Motorx->I_err=10000;
		if(Motorx->I_err<-10000)Motorx->I_err=-10000;
		Motorx->PWM_value=Motorx->Kp*Motorx->err+Motorx->Ki*Motorx->I_err+Motorx->Kd*Motorx->d_err;
	}
	else
		Motorx->PWM_value=Motorx->Kp*Motorx->err+Motorx->Kd*Motorx->d_err;

	Motorx->last_err=Motorx->err;
	//饱和限幅
	if(Motorx->PWM_value>11999) Motorx->PWM_value = 11999;
	if(Motorx->PWM_value<-11999) Motorx->PWM_value = -11999;
	return Motorx->PWM_value;
}

//前馈值的生成
void forward_Value(float* data_In, float* data_Out,int length)
{
	float amp_Val = 3500;//增益值
	uint16_t i = 0;
	for(i=2999; i>0; i--)
		data_Out[i] = amp_Val* (data_In[i]- data_In[-1+i]);
	data_Out[0] =  0.99f* data_Out[1];
	data_Out[2999] = 0.0f;
}


