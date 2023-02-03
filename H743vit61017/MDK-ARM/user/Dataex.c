#include "main.h"
#include "Dataex.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"
#include "control.h"
#include "Ctrl_schedule.h"
#include "key.h"
#include "FC_PID.h"
#include "Inv_Kine.h"

JY901_angle Left_Leg,Right_Leg;
dt_flag_t f;                               //���ͽṹ��
uint8_t data_to_send[255];                 //���ͻ�������    
uint8_t data_Received[255];                //���մ�������
extern uint32_t ADC1_Data[6];
extern uint32_t ADC3_Data[2];
extern uint8_t Position_zeroed_Flag;       //λ�ó�ʼ����־λ
extern uint8_t target_followed_flag;       //Ŀ����ٱ�־
uint8_t angle_Clear_Flag = 0;                  //�Ƕ�����ʹ��
uint8_t angle_Clear_Finished_Flag = 0;         //�Ƕ�������ɱ�־

//�ܳ���ִ�нṹ
void Data_Exchange(void)
{
	static uint8_t cnt = 0;                        //���� ������λ
	static uint8_t Curent_cnt 	    = 5;          //���͵��PWM����
	static uint8_t Encoder_cnt       = 10;
	static uint8_t Control_Inc_cnt   = 20;
	static uint8_t send2PC_cnt 	      = 100;        //���͵���Ƕ�
//send aotumatically	
	if((cnt % send2PC_cnt) == (send2PC_cnt-1))
		f.send_2PC = 1;	
	
	if((cnt % Curent_cnt) == (Curent_cnt-1))
		f.Curent = 1;	
	
	if((cnt % Control_Inc_cnt) == (Control_Inc_cnt-1))
	  f.Control_Inc = 1;
	
	if((cnt % Encoder_cnt) == (Encoder_cnt-1))
	  f.Encoder = 1	;
	
	  cnt++;
/////////////////////////////////////////////////////////////////////////////////////

	
	  if(f.send_2PC)
		{
			f.send_2PC = 0;
			Send_Data2PC();	       //����������
		}	
		
		if(f.Curent)
		{
			f.Curent = 0;	
			Current_updata();
//	uint8_t _cnt=0;
//	float f_temp;
//  uint8_t sum = 0;
//	uint8_t i;
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xF2;    //�����֣�F1Ϊ�û�����
//	data_to_send[_cnt++]=0;
//		
//	f_temp = Motor1.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	
//	f_temp = Motor2.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor3.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor4.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor5.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor6.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	data_to_send[3] = _cnt-4;
//		
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
			
		}	
		
		if(f.Control_Inc)
		{
			f.Control_Inc = 0;
			//λ�ù��� �����Զ�ģʽ ��λ�����Ϳ����ź�
			if(Position_zeroed_Flag != 1)
			{
				//��ȡ��λ��״̬��
			}
			else
			{
						//ɨ�谴���л�ģʽ
					target_followed_flag = Key_switch(target_followed_flag);
					//�����Զ�ģʽ
					if(target_followed_flag == 1)
					{
						if(angle_Clear()==0)
						{
							Control_Inc_updata();		
						}							
						HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin,GPIO_PIN_RESET);
					}//�����ֶ�ģʽ
					else if(target_followed_flag == 0)
					{					
						//�ֶ��ź�
						HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin,GPIO_PIN_SET);
						//�ֶ�ģʽ�»س�ʼλ��
						if(angle_Clear_Flag == 1)
						{
							if(angle_Clear_Finished_Flag)  angle_Clear_Flag=0;           //���Ƕ����㣬����Ƕ�����ʹ���ź�
							angle_Clear_Finished_Flag = angle_Clear();                   //���Ƕ����㣬�Ƕ�������ɱ�־��1				
						}
						else
						{
							//��������ת��
							Position_Demo(3000);
						}
					}
			}
			
		}	
		if(f.Encoder)
		{
			f.Encoder = 0;
      Encoder_value_updata();	  //�������ź�
		}
		
// ��ȡPID����
		if(f.send_pid1)
		{
			f.send_pid1 = 0;
			Send_PID1_to_PC();
		}	
		
		if(f.send_pid2)
		{
			f.send_pid2 = 0;
			Send_PID2_to_PC();
		}	
}


//���͵���ĽǶ�
void Send_Data2PC(void)     
{
	uint8_t _cnt=0;
	float f_temp;
  uint8_t sum = 0;
	uint8_t i;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;    //�����֣�F1Ϊ�û�����
	data_to_send[_cnt++]=0;
		
	f_temp = Motor1.angle;    //���͵���Ƕ�
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor2.angle;    //���͵���Ƕ�
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor3.angle;    //���͵���Ƕ�
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
//	

	f_temp = Motor4.angle;    //���͵���Ƕ�
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor5.angle;    //���͵���Ƕ�
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor6.angle;    //���͵���Ƕ�
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
//	 f_temp = Motor1pid.err;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor2pid.err;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);

//	
//	f_temp = Motor3pid.err;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);


//	f_temp = Motor1.Target;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);

//	
//	f_temp = Motor2.Target;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor3.Target;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
	
//	f_temp = Motor4.Target;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);

//	
//	f_temp = Motor5.Target;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor6.Target;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
	
//	_cnt = 0;
//	sum = 0;
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xF2;    //�����֣�F1Ϊ�û�����
//	data_to_send[_cnt++]=0;
//		
//	f_temp = Motor1.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	
//	f_temp = Motor2.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor3.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor4.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor5.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor6.curent;    //���͵���Ƕ�
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	data_to_send[3] = _cnt-4;
//		
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++]=sum;
//	
//	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
}


//������λ����ȡ���123����
void Send_PID1_to_PC(void)
{
	uint8_t _cnt=0;
	int16_t _temp;
  uint8_t sum = 0;
	uint8_t i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;    //�����֣�10ΪPID1
	data_to_send[_cnt++]=18;

  _temp = (int)Motor1pid.Kp;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor1pid.Ki;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor1pid.Kd;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)Motor2pid.Kp;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp = (int)Motor2pid.Ki;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor2pid.Kd;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)Motor3pid.Kp;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor3pid.Ki;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor3pid.Kd;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
}

// ������λ����ȡ���456��PID����
void Send_PID2_to_PC(void)
{
	uint8_t _cnt=0;
	int16_t _temp;
  uint8_t sum = 0;
	uint8_t i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;    //�����֣�10ΪPID1
	data_to_send[_cnt++]=18;

	 _temp = (int)Motor4pid.Kp;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp = (int)Motor4pid.Ki;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor4pid.Kd;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)Motor5pid.Kp;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp = (int)Motor5pid.Ki;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor5pid.Kd;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int)Motor6pid.Kp;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	 _temp = (int)Motor6pid.Ki;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)Motor6pid.Kd;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
}


/*****************************************              �������źŸ���                       ***************************/
/////////////////////
void Encoder_value_updata(void)
{
  //���1�ٶȽǶȸ���
  Angle_and_Speed_caculate(TIM1,&Motor1);
	
   //���2�ٶȽǶȸ���
  Angle_and_Speed_caculate(TIM2,&Motor2);
	
   //���3�ٶȽǶȸ���
  Angle_and_Speed_caculate(TIM3,&Motor3);
	
  //���4�ٶȽǶȸ���
  Angle_and_Speed_caculate(TIM4,&Motor4);
	
	//���5�ٶȽǶȸ���
  Angle_and_Speed_caculate(TIM5,&Motor5);
	
	//���6�ٶȽǶȸ���
  Angle_and_Speed_caculate(TIM8,&Motor6);
	
}

/*****************************************              �������źŸ���                       ***************************/

/*****************************************              �����źŸ���                        ***************************/

void Current_updata(void)
{
	uint32_t Data[8];
	float Vrefint = 0;
	SCB_InvalidateDCache_by_Addr((uint32_t *)ADC1_Data,sizeof(ADC1_Data));   /* ���ڴ�0x38000000�ڴ�ı������浽Cache */
	SCB_InvalidateDCache_by_Addr((uint32_t *)ADC3_Data,sizeof(ADC3_Data));   /* ���ڴ�0x38000000�ڴ�ı������浽Cache */	
	/* ���ڴ�0x38000000��ȫ�ֱ����Ƶ�DTCM�ڴ� */
	Data[0] = ADC1_Data[0];
	Data[1] = ADC1_Data[1];
	Data[2] = ADC1_Data[2];
	Data[3] = ADC1_Data[3];
	Data[4] = ADC1_Data[4];
	Data[5] = ADC1_Data[5];
	Data[6] = ADC3_Data[0];
	Data[7] = ADC3_Data[1];
	Vrefint = ((float)Data[7] / 65536) * 3.3f;
	Motor1.curent = Motor1.curent*0.8 + (((float)Data[0] / 65536) * Vrefint + 0.001 - 0.5 *Vrefint)*0.2 *1.7857f;
	Motor2.curent = Motor2.curent*0.8 + (((float)Data[1] / 65536) * Vrefint + 0.003 - 0.5 *Vrefint)*0.2 *1.7857f;
	Motor3.curent = Motor3.curent*0.8 + (((float)Data[2] / 65536) * Vrefint         - 0.5 *Vrefint)*0.2 *1.7857f;
	Motor4.curent = Motor4.curent*0.8 + (((float)Data[3] / 65536) * Vrefint + 0.002 - 0.5 *Vrefint)*0.2 *1.7857f;
	Motor5.curent = Motor5.curent*0.8 + (((float)Data[4] / 65536) * Vrefint         - 0.5 *Vrefint)*0.2 *1.7857f;
	Motor6.curent = Motor6.curent*0.8 + (((float)Data[5] / 65536) * Vrefint + 0.002 - 0.5 *Vrefint)*0.2 *1.7857f;

}

/*****************************************              �����źŸ���                        ***************************/

/*****************************************              ������λ��ͨѶ                       ***************************/

///�������ݳ�������
void Received_data_check(uint8_t Receive_data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&Receive_data==0xAA)
	{
		f.u_Tran_End_Flag = 0; 
		state=1;
		RxBuffer[0]=Receive_data;
	}
	else if(state==1&&Receive_data==0xAF)
	{
		state=2;
		RxBuffer[1]=Receive_data;
	}
	else if(state==2&&Receive_data<0xF1)//������
	{
		state=3;
		RxBuffer[2]=Receive_data;
	}
	else if(state==3&&Receive_data<50)//���ݳ���
	{
		state = 4;
		RxBuffer[3]=Receive_data;
		_data_len = Receive_data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)  //��������
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=Receive_data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)               //У���
	{
		state = 0;
		RxBuffer[4+_data_cnt]=Receive_data;
		Received_data_progress(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

//����ָ��ľ��崦��
void Received_data_progress(uint8_t *data_buf, uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
	
	if(*(data_buf+2)==0x10)								//PID1
    {
        Motor1pid.Kp  = 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        Motor1pid.Ki  = 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        Motor1pid.Kd  = 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        Motor2pid.Kp = 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        Motor2pid.Ki = 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        Motor2pid.Kd = 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
        Motor3pid.Kp 	= 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
        Motor3pid.Ki 	= 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
        Motor3pid.Kd 	= 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
    }
		if(*(data_buf+2)==0x11)								//PID2
    {
			  Motor4pid.Kp  = 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        Motor4pid.Ki  = 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        Motor4pid.Kd  = 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        Motor5pid.Kp = 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        Motor5pid.Ki = 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        Motor5pid.Kd = 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
        Motor6pid.Kp 	= 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
        Motor6pid.Ki 	= 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
        Motor6pid.Kd 	= 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
		if(*(data_buf+2)==0x22)								//��λ������0x22
    {
        Motor1pid.U_From_Pc  = ( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        Motor2pid.U_From_Pc  = ( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        Motor3pid.U_From_Pc  = ( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        Motor4pid.U_From_Pc = ( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        Motor5pid.U_From_Pc = ( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        Motor6pid.U_From_Pc = ( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
    }
		
		//����λ������PID
		if(*(data_buf+2)==0X02)
		{
			if(*(data_buf+4)==0X01)  //���Ͷ�ȡPID��ָ��
				{
					f.send_pid1 = 1;
					f.send_pid2 = 1;
				}
				if(*(data_buf+4)==0XA1)		//�ָ�Ĭ�ϲ������õ��ֹͣ
				 {
							Pid_Set(&Motor1pid,0,0,0);
							Pid_Set(&Motor2pid,0,0,0);
							Pid_Set(&Motor3pid,0,0,0);
							Pid_Set(&Motor4pid,0,0,0);
							Pid_Set(&Motor5pid,0,0,0);
							Pid_Set(&Motor6pid,0,0,0);
					}
				if(*(data_buf+4)==0XA2)		//����Ƕ�����
					{
						angle_Clear_Flag = 1; //�����Ƕ�����
					}
				if(*(data_buf+4)==0XA3)		//���ֹͣ
				{
//						Motor1_driver_ctrl(2000,0);
//						Motor3_driver_ctrl(2000,0);
//						Motor4_driver_ctrl(2000,0);
//						Motor6_driver_ctrl(2000,0);
				}
				if(*(data_buf+4)==0XA4)		//�ֶ�����ģʽ
				{
					  uint8_t Instruction = 0;
					  Instruction = *(data_buf+5);     //����ָ������0x01-0x20
					  Motor_AdustedFlag_from_PC(Instruction);  //ʹ�ܺ���
				}
				if(*(data_buf+4)==0XA5)		//��ʼ�����ģʽ
				{
					Position_zeroed_Flag = 1;
					target_followed_flag = 0;
					//��ʼ������ֵΪ0������������
					TIM1->CNT = 0x7fff;
					TIM2->CNT = 0x7fff;
					TIM3->CNT = 0x7fff;
					TIM4->CNT = 0x7fff;
					TIM5->CNT = 0x7fff;
					TIM8->CNT = 0x7fff;
				}
				if(*(data_buf+4)==0XA6)		//�Զ�����ģʽ
				{
					target_followed_flag = 1;
				}
				
	 }
}


//PID����У��λ
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, 7 ,10);
}

/*****************************************              ������λ��ͨѶ                       ***************************/

/*****************************************              ���ᴫ����ͨѶ                       ***************************/



//CopeSerialDataΪ����3�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void Receive_Data_From_JY901_left(uint8_t ucData)
{
	static uint8_t ucRxBuffer[250];
	static uint8_t ucRxCnt = 0;
	static float Roll1_x ,Pitch1_y ,Yaw1_z ;
  uint8_t Roll_L,Roll_H,Pitch_L,Pitch_H,Yaw_L,Yaw_H;
	unsigned char Sum=0,i;
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
  if(ucRxCnt<11){return;}                      //һ����ʮһ�����ݣ������ǣ��򷵻�}
	else
	{
		for(i=0;i<10;i++)
		{
			Sum+=ucRxBuffer[i];
		}
		if(Sum!=ucRxBuffer[10]){ucRxCnt=0;return;}
		if(ucRxBuffer[1]==0x53)
		{
			  Roll_L=ucRxBuffer[2];
			  Roll_H=ucRxBuffer[3];
			  Pitch_L=ucRxBuffer[4];
			  Pitch_H=ucRxBuffer[5];
			  Yaw_L=ucRxBuffer[6];
			  Yaw_H=ucRxBuffer[7];
		}
		//��ת˳��Ϊz-y-x��Ҳ����gamma-alpha-beta������ŷ����ԭ�򣬽Ƕȷ�Χ���Ϊ��+-90��*��+-60��*��+-90��٤�����90����Ӱ��x
		Roll1_x=(((short)Roll_H<<8)|Roll_L)*0.0055f;
		Pitch1_y = (((short)Pitch_H<<8)|Pitch_L)*0.0055f;
		Yaw1_z = (((short)Yaw_H<<8)|Yaw_L)*0.0055f;
		
		Right_Leg.Roll_x = Right_Leg.Roll_x*0.7f+Roll1_x*0.3f;
		Right_Leg.Pitch_y = Right_Leg.Pitch_y*0.7f+Pitch1_y*0.3f;
		Right_Leg.Yaw_z = Right_Leg.Yaw_z*0.7f+Yaw1_z*0.3;

		ucRxCnt=0;//��ջ�����
}
}


/*****************************************              ���ᴫ����ͨѶ                       ***************************/


