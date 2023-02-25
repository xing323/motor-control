#ifndef _MOTOR_H
#define _MOTOR_H

typedef struct{
	float curent;           //����
	float angle;            //�Ƕ�
	float speed;	          //�ٶ�
	float Target;           //Ŀ��
	float err;              //���
	int32_t Pulse;          //����������
	uint8_t Angle_Safe_Flag;//�ǶȰ�ȫ��־
	uint8_t Enable_Flag;    //���ʹ��
	int U;                  //�������PWM
	uint16_t I_lim_PWM;     //��������޷�ֵ�µ�PWM
	uint16_t PWM;           //���������תPWM״̬
}Motor;	

extern Motor Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;

void Motor_Init(void);

uint8_t Motor1_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor2_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor3_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor4_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor5_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);
uint8_t Motor6_driver_ctrl(uint16_t PWM_Value ,int8_t mode,uint16_t I_Limit_PWM);

void Angle_and_Speed_caculate(TIM_TypeDef * TIMx,Motor *Motorx);	//���������õ�ʱ�ӣ��ٶ� �Ƕ� д�����ṹ��
void Motorx_Ctrl(uint8_t motorx,int16_t U_From_PC);

#endif

