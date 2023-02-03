#ifndef _MOTOR_H
#define _MOTOR_H

typedef struct{
	float curent;           //电流
	float angle;            //角度
	float speed;	          //速度
	float Target;           //一个周期5000个点
	float err;
	int32_t Pulse;
	uint8_t Angle_Safe_Flag;
	uint8_t Enable_Flag;
}Motor;	

extern Motor Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;

void Motor_Init(void);

uint8_t Motor1_driver_ctrl(uint16_t PWM_Value ,int8_t mode);
uint8_t Motor2_driver_ctrl(uint16_t PWM_Value ,int8_t mode);
uint8_t Motor3_driver_ctrl(uint16_t PWM_Value ,int8_t mode);
uint8_t Motor4_driver_ctrl(uint16_t PWM_Value ,int8_t mode);
uint8_t Motor5_driver_ctrl(uint16_t PWM_Value ,int8_t mode);
uint8_t Motor6_driver_ctrl(uint16_t PWM_Value ,int8_t mode);

void Angle_and_Speed_caculate(TIM_TypeDef * TIMx,Motor *Motorx);	//输入电机和用的时钟，速度 角度 写入电机结构体
void Motorx_Ctrl(uint8_t motorx,int16_t U_From_PC);

#endif

