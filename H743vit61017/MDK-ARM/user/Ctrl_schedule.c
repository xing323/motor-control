#include "main.h"
#include "Ctrl_schedule.h"
#include "Motor.h"
#include "Dataex.h"
#include "control.h"
#include "usart.h"
#include "target.h"
#include "Inv_Kine.h"

#define Limit(a,up,down)  ((a>up?up:a)<down)?down:(a>up?up:a)
extern uint8_t receive_buff;
extern uint8_t u_Receive_Buff[30];

///////////////////******上位机控制matlab        Start        **************************//////
/**功能说明：从上位机串口读取PWM，作为驱动用
*/
void Contrl_From_PC(int safe_value)
{
	 HAL_UART_Receive_IT(&huart1 ,u_Receive_Buff ,17 );

	int Angle_Safe_Value = safe_value; 
	
	//安全标志位的设立
	if((Motor1.angle <= Angle_Safe_Value)&&(Motor1.angle >= -Angle_Safe_Value))
		Motor1.Angle_Safe_Flag = 1;
	else
		Motor1.Angle_Safe_Flag = 0;
	if((Motor2.angle <= Angle_Safe_Value)&&(Motor2.angle >= -Angle_Safe_Value))
		Motor2.Angle_Safe_Flag = 1;
	else
		Motor2.Angle_Safe_Flag = 0;
	if((Motor3.angle <= Angle_Safe_Value)&&(Motor3.angle >= -Angle_Safe_Value))
		Motor3.Angle_Safe_Flag = 1;
	else
		Motor3.Angle_Safe_Flag = 0;
	if((Motor4.angle <= Angle_Safe_Value)&&(Motor4.angle >= -Angle_Safe_Value))
		Motor4.Angle_Safe_Flag = 1;
	else
		Motor4.Angle_Safe_Flag = 0;
	if((Motor5.angle <= Angle_Safe_Value)&&(Motor5.angle >= -Angle_Safe_Value))
		Motor5.Angle_Safe_Flag = 1;
	else
		Motor5.Angle_Safe_Flag = 0;
	if((Motor6.angle <= Angle_Safe_Value)&&(Motor6.angle >= -Angle_Safe_Value))
		Motor6.Angle_Safe_Flag = 1;
	else
		Motor6.Angle_Safe_Flag = 0;

	if(f.u_Tran_End_Flag){
	//输出驱动PWM信号  角度大于600则停止
	if(Motor1.Angle_Safe_Flag)
		Motorx_Ctrl(1,(int16_t)Motor1pid.U_From_Pc);
	else
		Motorx_Ctrl(1,0);
  if(Motor2.Angle_Safe_Flag) 
		Motorx_Ctrl(2,(int16_t)Motor2pid.U_From_Pc);
	else
		Motorx_Ctrl(2,0);
	if(Motor3.Angle_Safe_Flag) 
		Motorx_Ctrl(3,(int16_t)Motor3pid.U_From_Pc);
	else
		Motorx_Ctrl(3,0);
	if(Motor4.Angle_Safe_Flag) 
		Motorx_Ctrl(4,(int16_t)Motor4pid.U_From_Pc);
	else
		Motorx_Ctrl(4,0);
  if(Motor5.Angle_Safe_Flag) 
		Motorx_Ctrl(5,(int16_t)Motor5pid.U_From_Pc);
	else
		Motorx_Ctrl(5,0);
	if(Motor6.Angle_Safe_Flag) 
	  Motorx_Ctrl(6,(int16_t)Motor6pid.U_From_Pc);
	else
		Motorx_Ctrl(6,0);
	
}
}



///////////////////******上位机控制信号        End        **************************//////


///////////////////******PID控制  Start**************************//////

void Control_Inc_updata(void)            //控制系统输入输出更新
{
	
	static float time = 0.0;
	static int Angle_Safe_Value = 700; 
	static uint16_t folwd_cnt = 0;
	
	Motor1.Target = 1;
	Motor2.Target = 0;
	Motor3.Target = 1;
	Motor4.Target = 600;
	Motor5.Target = 0;
	Motor6.Target = 600;
	
//	Motor1.Target = Left_InvKine.angle_M1_d[folwd_cnt];
//	Motor2.Target = Left_InvKine.angle_M2_d[folwd_cnt];
//	Motor3.Target = Left_InvKine.angle_M3_d[folwd_cnt];
//	Motor4.Target = Right_InvKine.angle_M1_d[folwd_cnt];
//	Motor5.Target = Right_InvKine.angle_M2_d[folwd_cnt];
//	Motor6.Target = Right_InvKine.angle_M3_d[folwd_cnt++];
	

//	
	if(folwd_cnt> 2999)  folwd_cnt = 2999;

	//安全标志位的设立
	if((Motor1.angle <= Angle_Safe_Value)&&(Motor1.angle >= -Angle_Safe_Value))
		Motor1.Angle_Safe_Flag = 1;
	else
		Motor1.Angle_Safe_Flag = 0;
	if((Motor2.angle <= Angle_Safe_Value)&&(Motor2.angle >= -Angle_Safe_Value))
		Motor2.Angle_Safe_Flag = 1;
	else
		Motor2.Angle_Safe_Flag = 0;
	if((Motor3.angle <= Angle_Safe_Value)&&(Motor3.angle >= -Angle_Safe_Value))
		Motor3.Angle_Safe_Flag = 1;
	else
		Motor3.Angle_Safe_Flag = 0;
	if((Motor4.angle <= Angle_Safe_Value)&&(Motor4.angle >= -Angle_Safe_Value))
		Motor4.Angle_Safe_Flag = 1;
	else
		Motor4.Angle_Safe_Flag = 0;
	if((Motor5.angle <= Angle_Safe_Value)&&(Motor5.angle >= -Angle_Safe_Value))
		Motor5.Angle_Safe_Flag = 1;
	else
		Motor5.Angle_Safe_Flag = 0;
	if((Motor6.angle <= Angle_Safe_Value)&&(Motor6.angle >= -Angle_Safe_Value))
		Motor6.Angle_Safe_Flag = 1;
	else
		Motor6.Angle_Safe_Flag = 0;
		
	  Motorx_Ctrl(1,(int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));
		Motorx_Ctrl(2,(int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));
		Motorx_Ctrl(3,(int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));
		Motorx_Ctrl(4,(int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));
		Motorx_Ctrl(5,(int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));
		Motorx_Ctrl(6,(int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));

	
//	//目标驱动设置Left_InvKine.angle_M3_dv[folwd_cnt]+
//	if(Motor1.Angle_Safe_Flag)
//		Motorx_Ctrl(1,(int16_t)Left_InvKine.angle_M1_dv[folwd_cnt]+ (int16_t)Motor_x_control_pid_PWM(&Motor1pid,Motor1.Target,Motor1.angle));
//	else
//		Motorx_Ctrl(1,0);
//  if(Motor2.Angle_Safe_Flag) 
//		Motorx_Ctrl(2,(int16_t)Left_InvKine.angle_M2_dv[folwd_cnt]+ (int16_t)Motor_x_control_pid_PWM(&Motor2pid,Motor2.Target,Motor2.angle));
//	else
//		Motorx_Ctrl(2,0);
//	if(Motor3.Angle_Safe_Flag) 
//		Motorx_Ctrl(3,(int16_t)-Left_InvKine.angle_M3_dv[folwd_cnt]+ (int16_t)Motor_x_control_pid_PWM(&Motor3pid,-Motor3.Target,Motor3.angle));
//	else
//		Motorx_Ctrl(3,0);
//	
//	if(Motor4.Angle_Safe_Flag) 
//		Motorx_Ctrl(4,(int16_t)Right_InvKine.angle_M1_dv[folwd_cnt]+ (int16_t)Motor_x_control_pid_PWM(&Motor4pid,Motor4.Target ,Motor4.angle));
//	else
//		Motorx_Ctrl(4,0);
//	
//  if(Motor5.Angle_Safe_Flag) 
//		Motorx_Ctrl(5,(int16_t)Right_InvKine.angle_M2_dv[folwd_cnt]+ (int16_t)Motor_x_control_pid_PWM(&Motor5pid,Motor5.Target,Motor5.angle));
//	else
//		Motorx_Ctrl(5,0);
//	
//	if(Motor6.Angle_Safe_Flag) 
//	  Motorx_Ctrl(6,(int16_t)-Right_InvKine.angle_M3_dv[folwd_cnt]+ (int16_t)Motor_x_control_pid_PWM(&Motor6pid,-Motor6.Target,Motor6.angle));
//	else
//		Motorx_Ctrl(6,0);
	
//	printf("motor1:%d,motor3:%d\n\n",TIM12->CCR1,TIM13->CCR1);
//	printf("motor4:%d,motor6:%d\n\n",TIM14->CCR1,TIM15->CCR2);
	
	time = time+0.001f;

	/////******    电机驱动pd控制      *****/ ////////////////

	
}

///////////////////******PID控制  End    **************************//////


//让电机角度回到设置好的零位
//注意此函数使用需要切断所有驱动函数
uint8_t angle_Clear(void)
{
	float min_value = 0.2;
	uint8_t M1,M2,M3,M4,M5,M6;
	int PWM1,PWM2,PWM3,PWM4,PWM5,PWM6;
	//设置PID参数
	Pid_Set(&Motor1pid, 300, 10, 0);
	Pid_Set(&Motor2pid, 300, 10, 0);
	Pid_Set(&Motor3pid, 300, 10, 0);
	Pid_Set(&Motor4pid, 200, 10, 0);
	Pid_Set(&Motor5pid, 300, 10, 0);
	Pid_Set(&Motor6pid, 300, 10, 0);
	PWM1 = (int16_t)Motor_x_control_pid_PWM(&Motor1pid, 0, Motor1.angle);
	PWM2 = (int16_t)Motor_x_control_pid_PWM(&Motor2pid, 0, Motor2.angle);
	PWM3 = (int16_t)Motor_x_control_pid_PWM(&Motor3pid, 0, Motor3.angle);
	PWM4 = (int16_t)Motor_x_control_pid_PWM(&Motor4pid, 0, Motor4.angle);
	PWM5 = (int16_t)Motor_x_control_pid_PWM(&Motor5pid, 0, Motor5.angle);
	PWM6 = (int16_t)Motor_x_control_pid_PWM(&Motor6pid, 0, Motor6.angle);
	if(PWM1 > 3000) PWM1 = 3000;
	if(PWM1 < -3000) PWM1 = -3000;
	if(PWM2 > 3000) PWM2 = 3000;
	if(PWM2 < -3000) PWM2 = -3000;
	if(PWM3 > 3000) PWM3 = 3000;
	if(PWM3 < -3000) PWM3 = -3000;
	if(PWM4 > 3000) PWM4 = 3000;
	if(PWM4 < -3000) PWM4 = -3000;
	if(PWM5 > 3000) PWM5 = 3000;
	if(PWM5 < -3000) PWM5 = -3000;
	if(PWM6 > 3000) PWM6 = 3000;
	if(PWM6 < -3000) PWM6 = -3000;
	Motorx_Ctrl(1, PWM1);
	Motorx_Ctrl(2, PWM2);
	Motorx_Ctrl(3, PWM3);
	Motorx_Ctrl(4, PWM4);
	Motorx_Ctrl(5, PWM5);
	Motorx_Ctrl(6, PWM6);
	//归零判断
	if((Motor1.angle < min_value)&&(Motor1.angle > -min_value)) {M1 = 1;} else {M1 = 0;}
	if((Motor2.angle < min_value)&&(Motor2.angle > -min_value)) {M2 = 1;} else {M2 = 0;}
	if((Motor3.angle < min_value)&&(Motor3.angle > -min_value)) {M3 = 1;} else {M3 = 0;}
	if((Motor4.angle < min_value)&&(Motor4.angle > -min_value)) {M4 = 1;} else {M4 = 0;}
	if((Motor5.angle < min_value)&&(Motor5.angle > -min_value)) {M5 = 1;} else {M5 = 0;}
	if((Motor6.angle < min_value)&&(Motor6.angle > -min_value)) {M6 = 1;} else {M6 = 0;}
		
	return M1 && M2&& M1&& M4 && M5 && M6;
	
}

