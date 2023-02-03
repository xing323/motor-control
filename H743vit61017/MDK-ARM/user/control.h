#ifndef _CONTROL_H
#define _CONTROL_H
//电机1和位置结构体
typedef struct{
	float err;          //误差
	float last_err;
	float d_err;        //误差的导数
	float Kp,Ki,Kd;
	float I_err;     //误差的积分
	float PWM_value;    //控制速度PWM
	unsigned int U_From_Pc;
	
}control_pid;


//控制电机参数结构体
extern control_pid Motor1pid ,Motor2pid, Motor3pid, Motor4pid,Motor5pid, Motor6pid, Motor1pid_sita, Motor2pid_sita, Motor3pid_sita, Motor4pid_sita, Motor5pid_sita, Motor6pid_sita;

void Control_Init(void);
void Motor_x_control_pid_Init(control_pid * Motorx);
void Pid_Set(control_pid * Motorx,float Kp,float Ki,float Kd);
float Motor_x_control_pid_PWM(control_pid * Motorx, float PV,float CV);
void forward_Value(float* data_In, float* data_Out,int length);

extern float motor1_followed_curve[5000];
#endif

