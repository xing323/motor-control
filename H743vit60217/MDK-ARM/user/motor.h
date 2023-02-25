#ifndef _MOTOR_H
#define _MOTOR_H

typedef struct{
	float curent;           //电流
	float angle;            //角度
	float speed;	          //速度
	float Target;           //目标
	float err;              //误差
	int32_t Pulse;          //编码器脉冲
	uint8_t Angle_Safe_Flag;//角度安全标志
	uint8_t Enable_Flag;    //电机使能
	int U;                  //电机控制PWM
	uint16_t I_lim_PWM;     //电机电流限幅值下的PWM
	uint16_t PWM;           //电机正常运转PWM状态
}Motor;	

extern Motor Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;

void Motor_Init(void);

uint8_t Motor1_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor2_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor3_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor4_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor5_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor6_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);

void Angle_and_Speed_caculate(TIM_TypeDef * TIMx,Motor *Motorx);	//输入电机和用的时钟，速度 角度 写入电机结构体
void Motorx_Ctrl(uint8_t motorx,int16_t U_From_PC);

#endif

