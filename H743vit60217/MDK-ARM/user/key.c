#include "main.h"
#include "key.h"
#include "motor.h"
extern uint8_t Position_zeroed_Flag;   //���������ɱ�־
//�ֶ����Ƶĵ���ٶ�
uint16_t Key_PWM_val = 2000;
void Delay_10ms(void)
{
	uint16_t _cnt = 5000;
	while(_cnt--);	
}



//����ɨ��
uint8_t Key_Scan(void)
{
	static uint8_t key_state=1;
	static uint8_t close_count=0;
	switch (key_state)
	{
		case 1:
			key_state= Motor1_zeroed();
			break;
		case 2:
			key_state= Motor2_zeroed();
			break;
		case 3:
			key_state= Motor3_zeroed();
			break;
		case 4:
			key_state= Motor4_zeroed();
			break;
		case 5:
			key_state= Motor5_zeroed();
			break;
		case 6:
			key_state= Motor6_zeroed();
			break;
		default:
			break;
	}
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET))
	{
		Delay_10ms();
		if(close_count++>100)
		{
			close_count=0;                //����λ�ù���֮ǰ�Ѽ�������
			return 1;
		}
		else
		{
		  return 0;
		}
	}
	else
	{
		close_count=0;                //���м��м�ϣ���Ϊ��
		return 0;
	}
}
//���㺯��

//���1
uint8_t Motor1_zeroed(void)
{
	uint8_t key_up, key_down,key_last ,key_next;
	static uint8_t key_move_flag=0;
	if((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)        key_up   = 1;//������־��1
	if((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)    key_down = 1;
	if((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)      key_last = 1;
	if((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)      key_next = 1;
	
	Delay_10ms(); //��������
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)&&(key_up==1))           //��ת
	{
		Motor1_driver_ctrl(Key_PWM_val,1,11999);
	  HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)&&(key_down==1)) //��ת
	{
		Motor1_driver_ctrl(Key_PWM_val,-1,11999);
	  HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else
	{
		key_up   = 0;
		key_down = 0;
		Motor1_driver_ctrl(0,0,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
	
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&(key_last==1)&&(key_move_flag==1))           //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		key_move_flag=0;
		Motor1_driver_ctrl(0,0,11999);
		return 6;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)&&(key_next==1)&&(key_move_flag==1))      //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		key_move_flag=0;
		Motor1_driver_ctrl(0,0,11999);
		return 2;
	}
	else
	{
		key_last=0;
		key_next=0;
		return 1;
	}
}
////////////////
//���2
uint8_t Motor2_zeroed(void)
{
	uint8_t key_up, key_down,key_last ,key_next;
	static uint8_t key_move_flag=0;                                                      //��ֹδ����ֱ�ӽ�����һ�����
	if((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)        key_up   = 1;//������־��1
	if((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)    key_down = 1;
	if((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)      key_last = 1;
	if((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)      key_next = 1;
	
	Delay_10ms(); //��������
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)&&(key_up==1))           //��ת
	{
		Motor2_driver_ctrl(Key_PWM_val,1 ,11999);
	  HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)&&(key_down==1)) //��ת
	{
		Motor2_driver_ctrl(Key_PWM_val,-1 ,11999);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else
	{
		key_up   = 0;
		key_down = 0;
		Motor2_driver_ctrl(0,0 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
	
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&(key_last==1)&&(key_move_flag==1))           //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		key_move_flag=0;
		Motor2_driver_ctrl(0,0 ,11999);
		return 1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)&&(key_next==1)&&(key_move_flag==1))      //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		key_move_flag=0;
		Motor2_driver_ctrl(0,0 ,11999);
		return 3;
	}
	else
	{
		key_last=0;
		key_next=0;
		return 2;
	}
}
////////////////
//���3
uint8_t Motor3_zeroed(void)
{
	uint8_t key_up, key_down,key_last ,key_next;
	static uint8_t key_move_flag=0; 
	if((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)        key_up   = 1;//������־��1
	if((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)    key_down = 1;
	if((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)      key_last = 1;
	if((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)      key_next = 1;
	
	Delay_10ms(); //��������
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)&&(key_up==1))           //��ת
	{
		Motor3_driver_ctrl(Key_PWM_val,1 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)&&(key_down==1)) //��ת
	{
		Motor3_driver_ctrl(Key_PWM_val,-1 ,11999);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else
	{
		key_up   = 0;
		key_down = 0;
		Motor3_driver_ctrl(0,0 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
	
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&(key_last==1)&&(key_move_flag==1))           //��һ�����
	{
		
		key_move_flag=0;
		Motor3_driver_ctrl(0,0 ,11999);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		return 2;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)&&(key_next==1)&&(key_move_flag==1))      //��һ�����
	{
		key_move_flag=0;
		Motor3_driver_ctrl(0,0 ,11999);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		return 4;
	}
	else
	{
		key_last=0;
		key_next=0;
		return 3;
	}
}
////////////////
//���4
uint8_t Motor4_zeroed(void)
{
	uint8_t key_up, key_down,key_last ,key_next;
	static uint8_t key_move_flag=0; 
	if((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)        key_up   = 1;//������־��1
	if((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)    key_down = 1;
	if((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)      key_last = 1;
	if((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)      key_next = 1;
	
	Delay_10ms(); //��������
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)&&(key_up==1))           //��ת
	{
		Motor4_driver_ctrl(Key_PWM_val,1 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)&&(key_down==1)) //��ת
	{
		Motor4_driver_ctrl(Key_PWM_val,-1 ,11999);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else
	{
		key_up   = 0;
		key_down = 0;
		Motor4_driver_ctrl(0,0 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
	
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&(key_last==1)&&(key_move_flag==1))           //��һ�����
	{
		key_move_flag=0;
		Motor4_driver_ctrl(0,0 ,11999);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		return 3;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)&&(key_next==1)&&(key_move_flag==1))      //��һ�����
	{
		key_move_flag=0;
		Motor4_driver_ctrl(0,0 ,11999);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		return 5;
	}
	else
	{
		key_last=0;
		key_next=0;
		return 4;
	}
}
////////////////
//���5
uint8_t Motor5_zeroed(void)
{
	uint8_t key_up, key_down,key_last ,key_next;
	static uint8_t key_move_flag=0; 
	if((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)        key_up   = 1;//������־��1
	if((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)    key_down = 1;
	if((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)      key_last = 1;
	if((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)      key_next = 1;
	
	Delay_10ms(); //��������
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)&&(key_up==1))           //��ת
	{
		Motor5_driver_ctrl(Key_PWM_val,1 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)&&(key_down==1)) //��ת
	{
		Motor5_driver_ctrl(Key_PWM_val,-1 ,11999);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else
	{
		key_up   = 0;
		key_down = 0;
		Motor5_driver_ctrl(0,0 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
	
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&(key_last==1)&&(key_move_flag==1))           //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		Motor5_driver_ctrl(0,0 ,11999);
		key_move_flag=0;
		return 4;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)&&(key_next==1)&&(key_move_flag==1))      //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		Motor5_driver_ctrl(0,0 ,11999);
		key_move_flag=0;
		return 6;
	}
	else
	{
		return 5;
	}
}
////////////////
//���6
uint8_t Motor6_zeroed(void)
{
	uint8_t key_up, key_down,key_last ,key_next;
	static uint8_t key_move_flag=0; 
	if((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)        key_up   = 1;//������־��1
	if((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)    key_down = 1;
	if((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)      key_last = 1;
	if((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)      key_next = 1;
	
	Delay_10ms(); //��������
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET)&&(key_up==1))           //��ת
	{
		Motor6_driver_ctrl(Key_PWM_val,1 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)&&(key_down==1)) //��ת
	{
		Motor6_driver_ctrl(Key_PWM_val,-1 ,11999);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		key_move_flag=1;
	}
	else
	{
		key_up   = 0;
		key_down = 0;
		Motor6_driver_ctrl(0,0 ,11999);
		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
	
	if(((HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin))==GPIO_PIN_SET)&&(key_last==1)&&(key_move_flag==1))           //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		key_move_flag=0;
		Motor6_driver_ctrl(0,0 ,11999);
		return 5;
	}
	else if(((HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin))==GPIO_PIN_SET)&&(key_next==1)&&(key_move_flag==1))      //��һ�����
	{
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		key_move_flag=0;
		Motor6_driver_ctrl(0,0 ,11999);
		return 1;
	}
	else
	{
		return 6;
	}
}


//************  �ֶ�����/��չ���ճ���  **************//
void ShouDongQuShenZhanShou_Function(int PWM)
{
	int8_t KeyUp_value, KeyDown_value, KeyLast_value, KeyNext_value;
	int Motor3_TongBuXiuBu_Value = 0;
	int Motor6_TongBuXiuBu_Value = 0;
	//����PWM���ˣ������淶ֱ�Ӳ���
  //if((PWM>5000)||(PWM<0))  PWM=0;
	
	//��ȡ����ֵ
	KeyUp_value = HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin);
	KeyDown_value = HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin);
  KeyLast_value = HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin);
	KeyNext_value = HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin);
	
	Delay_10ms();
	
	//���ڲ�ͬ������Ҫ����ͬ���޲�ֵ����ͬ���޲�(��ת�Ƕ��෴Ϊͬ��
	
	Motor3_TongBuXiuBu_Value = 200 * (Motor1.angle + Motor3.angle);
	Motor6_TongBuXiuBu_Value = 200 * (Motor4.angle + Motor6.angle);
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET))           //��ת
		{
			Motor1_driver_ctrl(PWM,                            (int)Motor1.Enable_Flag, Motor1.I_lim_PWM);
			Motor2_driver_ctrl(PWM,                            (int)Motor2.Enable_Flag, Motor2.I_lim_PWM);
			Motor3_driver_ctrl(PWM + Motor3_TongBuXiuBu_Value,-(int)Motor3.Enable_Flag, Motor3.I_lim_PWM);
			Motor4_driver_ctrl(PWM,                            (int)Motor4.Enable_Flag, Motor4.I_lim_PWM);
			Motor5_driver_ctrl(PWM,                            (int)Motor5.Enable_Flag, Motor5.I_lim_PWM);
			Motor6_driver_ctrl(PWM + Motor6_TongBuXiuBu_Value,-(int)Motor6.Enable_Flag, Motor6.I_lim_PWM);
			HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		}
		else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)) //��ת
		{
			Motor1_driver_ctrl(PWM,                           -(int)Motor1.Enable_Flag, Motor1.I_lim_PWM);
			Motor2_driver_ctrl(PWM,                           -(int)Motor2.Enable_Flag, Motor2.I_lim_PWM);
			Motor3_driver_ctrl(PWM - Motor3_TongBuXiuBu_Value, (int)Motor3.Enable_Flag, Motor3.I_lim_PWM);
			Motor4_driver_ctrl(PWM,                           -(int)Motor4.Enable_Flag, Motor4.I_lim_PWM);
			Motor5_driver_ctrl(PWM,                           -(int)Motor5.Enable_Flag, Motor5.I_lim_PWM);
			Motor6_driver_ctrl(PWM - Motor6_TongBuXiuBu_Value, (int)Motor6.Enable_Flag, Motor6.I_lim_PWM);
			HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		}
		else
		{
			Motor1_driver_ctrl(0,0 ,11999);
			Motor2_driver_ctrl(0,0 ,11999);
			Motor3_driver_ctrl(0,0 ,11999);
			Motor4_driver_ctrl(0,0 ,11999);
			Motor5_driver_ctrl(0,0 ,11999);
			Motor6_driver_ctrl(0,0 ,11999);
			HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
		}
}

//************  �ֶ�����������  **************//
void ShouDongNeiWaiXuan_Function(int PWM)
{
	int8_t KeyUp_value, KeyDown_value, KeyLast_value, KeyNext_value;
	
	//����PWM���ˣ������淶ֱ�Ӳ���
  //if((PWM>5000)||(PWM<0))  PWM=0;
	
	//��ȡ����ֵ
	KeyUp_value = HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin);
	KeyDown_value = HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin);
  KeyLast_value = HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin);
	KeyNext_value = HAL_GPIO_ReadPin(KEY_Next_GPIO_Port,KEY_Next_Pin);
	
	Delay_10ms();
	
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET))           //����
		{
			Motor1_driver_ctrl(PWM,    (int)Motor1.Enable_Flag, Motor1.I_lim_PWM);
			Motor2_driver_ctrl(PWM/10,-(int)Motor2.Enable_Flag, Motor2.I_lim_PWM);
			Motor3_driver_ctrl(PWM,    (int)Motor3.Enable_Flag, Motor3.I_lim_PWM);
			Motor4_driver_ctrl(PWM,    (int)Motor4.Enable_Flag, Motor4.I_lim_PWM);
			Motor5_driver_ctrl(PWM/10,-(int)Motor5.Enable_Flag, Motor5.I_lim_PWM);
			Motor6_driver_ctrl(PWM,    (int)Motor6.Enable_Flag, Motor6.I_lim_PWM);
			HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		}
		else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)) //����
		{
			Motor1_driver_ctrl(PWM,   -(int)Motor1.Enable_Flag, Motor1.I_lim_PWM);
			Motor2_driver_ctrl(PWM/10, (int)Motor2.Enable_Flag, Motor2.I_lim_PWM);
			Motor3_driver_ctrl(PWM,   -(int)Motor3.Enable_Flag, Motor3.I_lim_PWM);
			Motor4_driver_ctrl(PWM,   -(int)Motor4.Enable_Flag, Motor4.I_lim_PWM);
			Motor5_driver_ctrl(PWM/10, (int)Motor5.Enable_Flag, Motor5.I_lim_PWM);
			Motor6_driver_ctrl(PWM,   -(int)Motor6.Enable_Flag, Motor6.I_lim_PWM);
			HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		}
		else
		{
			Motor1_driver_ctrl(0,0 ,11999);
			Motor2_driver_ctrl(0,0 ,11999);
			Motor3_driver_ctrl(0,0 ,11999);
			Motor4_driver_ctrl(0,0 ,11999);
			Motor5_driver_ctrl(0,0 ,11999);
			Motor6_driver_ctrl(0,0 ,11999);
			HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
		}
}
/////////////////
//�л��Զ����ֶ�
uint8_t  Key_switch(uint8_t flag)
{
	static uint8_t Key_count = 0;//������������
	static uint8_t a =0;
	static uint8_t  KeyLast_value;
	static uint8_t  switch_intval = 0;//�л��������100ms
	
	//��ȡ����
  KeyLast_value = HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin);
	
	if(switch_intval==0)
	{
		//����c��������50ms����
		if(KeyLast_value == GPIO_PIN_RESET)
		{
			//��ȡ����
			KeyLast_value = HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin);
		}
		else
		{
			if(Key_count++ >25) //��������
			{
				KeyLast_value = HAL_GPIO_ReadPin(KEY_Last_GPIO_Port,KEY_Last_Pin);
				if(KeyLast_value == GPIO_PIN_SET)
				{
					switch_intval = 50;
					if(flag == 0)
					{		
						a= 1;			
					}
					if(flag == 1)
					{			
						a= 0;
					}
				}
			}	
		}
	}
	
	if(switch_intval > 0 )
	{
		switch_intval--;//�л�ʱ��	
	}
	return a;
}



////��λ���ӿں���
/* ���ݽ��յ����ݵ���ʹ�ܲ������������*/
void Motor_AdustedFlag_from_PC(uint8_t data)
{
	//��ʼ��ʧ��
	Motor1.Enable_Flag = 0;
	Motor2.Enable_Flag = 0;
	Motor3.Enable_Flag = 0;
	Motor4.Enable_Flag = 0;
	Motor5.Enable_Flag = 0;
	Motor6.Enable_Flag = 0;
	//���ݽ�������ʹ��
	switch(data)
	{
		case 0x01://motor1
		{
			Motor1.Enable_Flag = 1;	break;	
		}
		case 0x02://motor2
		{
			Motor2.Enable_Flag = 1;break;			
		}
		case 0x04://motor3
		{
			Motor3.Enable_Flag = 1;	break;		
		}
		case 0x08://motor4
		{
			Motor4.Enable_Flag = 1;	break;		
		}
		case 0x10://motor5
		{
			Motor5.Enable_Flag = 1;	break;		
		}
		case 0x20://motor6
		{
			Motor6.Enable_Flag = 1;	break;		
		}
		break;
	}
}

//�����������
void Motor_Adusted_from_PC(void)
{
	uint16_t PWM = 2000;
	if(((HAL_GPIO_ReadPin(KEY_Up_GPIO_Port,KEY_Up_Pin))==GPIO_PIN_SET))           //��ת
		{
			Motor1_driver_ctrl(PWM,(int)Motor1.Enable_Flag ,Motor1.I_lim_PWM);
			Motor2_driver_ctrl(PWM,(int)Motor2.Enable_Flag ,Motor2.I_lim_PWM);
			Motor3_driver_ctrl(PWM,(int)Motor3.Enable_Flag ,Motor3.I_lim_PWM);
			Motor4_driver_ctrl(PWM,(int)Motor4.Enable_Flag ,Motor4.I_lim_PWM);
			Motor5_driver_ctrl(PWM,(int)Motor5.Enable_Flag ,Motor5.I_lim_PWM);
			Motor6_driver_ctrl(PWM,(int)Motor6.Enable_Flag ,Motor6.I_lim_PWM);
			HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_RESET);
		}
	else if(((HAL_GPIO_ReadPin(KEY_Down_GPIO_Port,KEY_Down_Pin))==GPIO_PIN_SET)) //��ת
		{
			Motor1_driver_ctrl(PWM,-(int)Motor1.Enable_Flag ,Motor1.I_lim_PWM);
			Motor2_driver_ctrl(PWM,-(int)Motor2.Enable_Flag ,Motor2.I_lim_PWM);
			Motor3_driver_ctrl(PWM,-(int)Motor3.Enable_Flag ,Motor3.I_lim_PWM);
			Motor4_driver_ctrl(PWM,-(int)Motor4.Enable_Flag ,Motor4.I_lim_PWM);
			Motor5_driver_ctrl(PWM,-(int)Motor5.Enable_Flag ,Motor5.I_lim_PWM);
			Motor6_driver_ctrl(PWM,-(int)Motor6.Enable_Flag ,Motor6.I_lim_PWM);
			HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_RESET);
		}
		else
		{
			//�Է���һ�����ƶ�
			Motor1_driver_ctrl(0,0 ,11999);
			Motor2_driver_ctrl(0,0 ,11999);
			Motor3_driver_ctrl(0,0 ,11999);
			Motor4_driver_ctrl(0,0 ,11999);
			Motor5_driver_ctrl(0,0 ,11999);
			Motor6_driver_ctrl(0,0 ,11999);
			HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin,GPIO_PIN_SET);
	}
}
