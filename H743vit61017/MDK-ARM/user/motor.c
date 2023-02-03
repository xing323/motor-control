#include "main.h"
#include "motor.h"
#include "tim.h"

#define Limit(a,up,down)  ((a>up?up:a)<down)?down:(a>up?up:a)
/////////////////////
Motor Motor1, Motor2, Motor3, Motor4, Motor5, Motor6;    //����6���ṹ��

///////
void Motor_Init(void)
{
	Motor1.angle=0;
	Motor2.angle=0;
	Motor3.angle=0;    //�Ƕȳ�ʼ��
	Motor4.angle=0;
	Motor5.angle=0;
	Motor6.angle=0;    //�Ƕȳ�ʼ��
	
	Motor1.Pulse=0;
	Motor2.Pulse=0;
	Motor3.Pulse=0;     //�����ʼ��
	Motor4.Pulse=0;
	Motor5.Pulse=0;
	Motor6.Pulse=0;     //�����ʼ��
	
}
//////////////////

/////////////

uint8_t Motor1_driver_ctrl(uint16_t PWM_Value ,int8_t mode)
{
	//�������������βδ��ݵ��������Ϲ淶������ȷ���Ǵ������ˣ��ǾͲ��ܶ���
	if((mode!=0)&&(mode!=1)&&(mode!=-1)) return 0;
	if(((uint16_t)PWM_Value>11999))return 0;
	
	switch (mode)
	{
		case 0:       //Stop
			HAL_GPIO_WritePin(M1_In1_GPIO_Port,M1_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M1_In2_GPIO_Port,M1_In2_Pin, GPIO_PIN_SET);
		  TIM12->CCR1 = 0;
			break;
		case 1:      //+
			HAL_GPIO_WritePin(M1_In1_GPIO_Port,M1_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M1_In2_GPIO_Port,M1_In2_Pin, GPIO_PIN_RESET);	
      TIM12->CCR1 = (uint16_t)PWM_Value;		
			break;
		case -1:     //-
			HAL_GPIO_WritePin(M1_In1_GPIO_Port,M1_In1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M1_In2_GPIO_Port,M1_In2_Pin, GPIO_PIN_SET);
			TIM12->CCR1 = (uint16_t)PWM_Value;
			break;
	}
	return 1;
	
}


/////////
uint8_t Motor2_driver_ctrl(uint16_t PWM_Value ,int8_t mode)
{
	//�������������βδ��ݵ��������Ϲ淶������ȷ���Ǵ������ˣ��ǾͲ��ܶ���
	if((mode!=0)&&(mode!=1)&&(mode!=-1)) return 0;
	if(((uint16_t)PWM_Value>11999))return 0;
	switch (mode)
	{
		case 0:       //Stop
			HAL_GPIO_WritePin(M2_In1_GPIO_Port,M2_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M2_In2_GPIO_Port,M2_In2_Pin, GPIO_PIN_SET);
		  TIM12->CCR2 = 0;
			break;
		case 1:      //+
			HAL_GPIO_WritePin(M2_In1_GPIO_Port,M2_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M2_In2_GPIO_Port,M2_In2_Pin, GPIO_PIN_RESET);	
      TIM12->CCR2 = (uint16_t)PWM_Value;		
			break;
		case -1:     //-
			HAL_GPIO_WritePin(M2_In1_GPIO_Port,M2_In1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M2_In2_GPIO_Port,M2_In2_Pin, GPIO_PIN_SET);
		  TIM12->CCR2 = (uint16_t)PWM_Value;
			break;
	}
	return 1;
}

//////////
////////////////////////////////////////////////////////////////////////////
uint8_t Motor3_driver_ctrl(uint16_t PWM_Value ,int8_t mode)
{
	//�������������βδ��ݵ��������Ϲ淶������ȷ���Ǵ������ˣ��ǾͲ��ܶ���
	if((mode!=0)&&(mode!=1)&&(mode!=-1)) return 0;
	if(((uint16_t)PWM_Value>11999))return 0;
	switch (mode)
	{
		case 0:       //Stop
			HAL_GPIO_WritePin(M3_In1_GPIO_Port,M3_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M3_In2_GPIO_Port,M3_In2_Pin, GPIO_PIN_SET);
		  TIM13->CCR1 = 0;
			break;
		case 1:      //+
			HAL_GPIO_WritePin(M3_In1_GPIO_Port,M3_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M3_In2_GPIO_Port,M3_In2_Pin, GPIO_PIN_RESET);	
      TIM13->CCR1 = (uint16_t)PWM_Value;		
			break;
		case -1:     //-
			HAL_GPIO_WritePin(M3_In1_GPIO_Port,M3_In1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M3_In2_GPIO_Port,M3_In2_Pin, GPIO_PIN_SET);
		  TIM13->CCR1 = (uint16_t)PWM_Value;
			break;
	}
	return 1;
}

/////////////////////////
/////////////////////
uint8_t Motor4_driver_ctrl(uint16_t PWM_Value ,int8_t mode)
{
	//�������������βδ��ݵ��������Ϲ淶������ȷ���Ǵ������ˣ��ǾͲ��ܶ���
	if((mode!=0)&&(mode!=1)&&(mode!=-1)) return 0;
	if(((uint16_t)PWM_Value>11999))return 0;
	switch (mode)
	{
		case 0:       //Stop
			HAL_GPIO_WritePin(M4_In1_GPIO_Port,M4_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M4_In2_GPIO_Port,M4_In2_Pin, GPIO_PIN_SET);
		  TIM14->CCR1 = 0;
			break;
		case 1:      //+
			HAL_GPIO_WritePin(M4_In1_GPIO_Port,M4_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M4_In2_GPIO_Port,M4_In2_Pin, GPIO_PIN_RESET);	
      TIM14->CCR1 = (uint16_t)PWM_Value;		
			break;
		case -1:     //-
			HAL_GPIO_WritePin(M4_In1_GPIO_Port,M4_In1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M4_In2_GPIO_Port,M4_In2_Pin, GPIO_PIN_SET);
		  TIM14->CCR1 = (uint16_t)PWM_Value;
			break;
	}
	return 1;
}
/////////////////////////////

/////////////////////
uint8_t Motor5_driver_ctrl(uint16_t PWM_Value ,int8_t mode)
{
	//�������������βδ��ݵ��������Ϲ淶������ȷ���Ǵ������ˣ��ǾͲ��ܶ���
	if((mode!=0)&&(mode!=1)&&(mode!=-1)) return 0;
	if(((uint16_t)PWM_Value>11999))return 0;
	switch (mode)
	{
		case 0:       //Stop
			HAL_GPIO_WritePin(M5_In1_GPIO_Port,M5_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M5_In2_GPIO_Port,M5_In2_Pin, GPIO_PIN_SET);
		  TIM15->CCR1 = 0;
			break;
		case 1:      //+
			HAL_GPIO_WritePin(M5_In1_GPIO_Port,M5_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M5_In2_GPIO_Port,M5_In2_Pin, GPIO_PIN_RESET);	
      TIM15->CCR1 = (uint16_t)PWM_Value;		
			break;
		case -1:     //-
			HAL_GPIO_WritePin(M5_In1_GPIO_Port,M5_In1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M5_In2_GPIO_Port,M5_In2_Pin, GPIO_PIN_SET);
		  TIM15->CCR1 = (uint16_t)PWM_Value;
			break;
	}
	return 1;
}
///////////
uint8_t Motor6_driver_ctrl(uint16_t PWM_Value ,int8_t mode)
{
	//�������������βδ��ݵ��������Ϲ淶������ȷ���Ǵ������ˣ��ǾͲ��ܶ���
	if((mode!=0)&&(mode!=1)&&(mode!=-1)) return 0;
	if(((uint16_t)PWM_Value>11999))return 0;
	
	switch (mode)
	{
		case 0:       //Stop
			HAL_GPIO_WritePin(M6_In1_GPIO_Port,M6_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M6_In2_GPIO_Port,M6_In2_Pin, GPIO_PIN_SET);
		  TIM15->CCR2 = 0;
			break;
		case 1:      //+
			HAL_GPIO_WritePin(M6_In1_GPIO_Port,M6_In1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M6_In2_GPIO_Port,M6_In2_Pin, GPIO_PIN_RESET);	
      TIM15->CCR2 = (uint16_t)PWM_Value;		
			break;
		case -1:     //-
			HAL_GPIO_WritePin(M6_In1_GPIO_Port,M6_In1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M6_In2_GPIO_Port,M6_In2_Pin, GPIO_PIN_SET);
		  TIM15->CCR2 = (uint16_t)PWM_Value;
			break;
	}
	return 1;
}

////////////////////
void Angle_and_Speed_caculate(TIM_TypeDef * TIMx,Motor *Motorx)   //���������õ�ʱ�ӣ��Ƕ�д�����ṹ��
{
	float angle,speed;
	static int16_t Pulse = 0;                       //���32768
	Pulse = getTIMx_DetaCnt(TIMx);
	Motorx->Pulse += Pulse;
	
//	Motorx->angle=Motorx->Pulse*0.0035;       //���˲�
	angle= Motorx->Pulse*0.00353f;              //float����λС��
	Motorx->angle *= 0.7f;
	Motorx->angle += angle*0.3f;	
	
	speed=Pulse*200*0.000588f;                  //������ٶȼ���
	Motorx->speed *= 0.7f;
	Motorx->speed += speed*0.3f;	
}

//��������
void Motorx_Ctrl(uint8_t motorx,int16_t PWM)
{
	uint8_t motor=0;
	if((motorx>6)|| (motorx<1)) return;
	motor=motorx;
	switch (motor)
	{
		case 1:
			if(PWM>0)
				Motor1_driver_ctrl(PWM,1);
		else 
			Motor1_driver_ctrl(-PWM,-1);
			break;
		case 2:
			if(PWM>0)
				Motor2_driver_ctrl(PWM,1);
		else 
			Motor2_driver_ctrl(-PWM,-1);
			break;
		case 3:
			if(PWM>0)
				Motor3_driver_ctrl(PWM,1);
		else 
			Motor3_driver_ctrl(-PWM,-1);
			break;
		case 4:
			if(PWM>0)
				Motor4_driver_ctrl(PWM,1);
		else 
			Motor4_driver_ctrl(-PWM,-1);
			break;
		case 5:
			if(PWM>0)
				Motor5_driver_ctrl(PWM,1);
		else 
			Motor5_driver_ctrl(-PWM,-1);
			break;
		case 6:
			if(PWM>0)
				Motor6_driver_ctrl(PWM,1);
		else 
			Motor6_driver_ctrl(-PWM,-1);
			break;
  }	
}






