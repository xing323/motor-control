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
dt_flag_t f;                                   //发送结构体
UI_Flag UI;
extern uint32_t ADC1_Data[6];
extern uint32_t ADC3_Data[2];
extern uint8_t Position_zeroed_Flag;           //位置初始化标志位
extern uint8_t target_followed_flag;           //目标跟踪标志
uint8_t data_to_send[255];                     //发送缓存数组    
uint8_t data_Received[255];                    //接收处理数组
uint8_t angle_Clear_Flag = 0;                  //角度清零使能
uint8_t angle_Clear_Finished_Flag = 0;         //角度清零完成标志
uint8_t Send_Data_mode = 0;                    //数据发送模式，=0为匿名上位机，=1为自己上位机
uint8_t ShouDongKangFu_mode;                   //手动康复模式标志
uint8_t AutoKangFu_mode;                       //自动康复模式标志
uint8_t LeftLeg_NeiWaiXuan;                    //左腿内外旋标志
uint8_t RightLeg_NeiWaiXuan;                   //右腿内外旋标志
int ShouDong_Speed;                            //手动康复速度
int AutoKangFu_CiShu;                          //自动康复次数
uint8_t receive_buff_tiaoshi[20];

//总程序执行结构
void Data_Exchange(void)
{
	static uint16_t cnt               = 0;           //基础 计数单位
	static uint8_t  Curent_cnt 	      = 5;           //电机电流更新
	static uint8_t  Encoder_cnt       = 10;          //编码器状态更新
	static uint8_t  Control_Inc_cnt   = 20;          //控制信号更新
	static uint8_t  send2NiMing_cnt   = 200;         //数据采集时间（发送到匿名上位机）
	static uint16_t  send2PC_cnt 	    = 500;         //发送电机角度给上位机

	
  //send aotumatically	

	if((cnt % Curent_cnt) == (Curent_cnt-1))
		f.Curent = 1;	
	
	if((cnt % Encoder_cnt) == (Encoder_cnt-1))
	  f.Encoder = 1	;
	
	if((cnt % Control_Inc_cnt) == (Control_Inc_cnt-1))
	  f.Control_Inc = 1;
	
	if((cnt % send2NiMing_cnt) == (send2NiMing_cnt-1))
	  f.send2NiMing = 1	;
		
	if((cnt % send2PC_cnt) == (send2PC_cnt-1))
		f.send_2PC = 1;		
	

	  
	 if(cnt<65500)cnt++;else cnt = 0;  //应该选择上面数的公倍数
	
	//任何时候的回零操作
	if((angle_Clear_Flag == 1) || (UI.angle_Clear_Flag == 1))
	{
		//自动手动模式归零
		AutoKangFu_mode     = 0;UI.AutoKangFu_mode     = 0;
		ShouDongKangFu_mode = 0;UI.ShouDongKangFu_mode = 0;
		//电机失能
		Motor_Enable(0);
		angle_Clear_Finished_Flag = angle_Clear();  
		if(angle_Clear_Finished_Flag)  {angle_Clear_Flag=0;UI.angle_Clear_Flag = 0;}           //若角度清零，清除角度清零使能信号
		                 //若角度清零，角度清零完成标志置1				
	}
	
	
//*************************************************//

		if(f.Curent)
		{
			f.Curent = 0;	
			Current_updata();		
		}	
		
		if(f.Encoder)
		{
			f.Encoder = 0;
      Encoder_value_updata();	  //编码器信号
		}
		
		if(f.Control_Inc)
		{
			f.Control_Inc = 0;
			//位置归零 进入自动模式 上位机发送控制信号
			if(Position_zeroed_Flag != 1)
			{
				//读取上位机状态量
			}
			else
			{
					//扫描按键切换模式
					target_followed_flag = Key_switch(target_followed_flag);
				
				  //自动训练模式
				  if(target_followed_flag)
            {
						    AutoKangFu_mode = 1;
							  UI.AutoKangFu_CiShu = 1;
						}
				
				  if(AutoKangFu_mode){
						if(UI.AutoKangFu_CiShu)
						{
							if(!UI.Curve_Generation_Flag)
							{
						    //Control_Inc_updata();          //控制系统输入输出更新
								Control_FC_update();
							}
						}
						
					HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin,GPIO_PIN_RESET);
					}
					
					else if(ShouDongKangFu_mode)
					{					
						
						//手动模式下回初始位置
							//按键控制转动
							if(LeftLeg_NeiWaiXuan || RightLeg_NeiWaiXuan)
							{
								ShouDongNeiWaiXuan_Function(ShouDong_Speed);
								//手动信号
						    HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin,GPIO_PIN_SET);
							}
							else
							{
								ShouDongQuShenZhanShou_Function(ShouDong_Speed);
								//手动信号
						    HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin,GPIO_PIN_SET);
							}
					}
			}	
		}
		
		if(f.send_2PC)
		{
			f.send_2PC = 0;
			if(!f.send_2PC)
			{
				Send_Data2PC();	           //发送数据到自己上位机（显示）
				Check_Back();
			}
		}	
		
		if(f.send2NiMing)
		{
			f.send2NiMing = 0;
			if(f.send2NiMing)
			{
				Send_Data2NiMing();	       //发送数据到匿名（数据采集储存）
			}
			
		}	

// 读取PID参数
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




//发送到匿名进行数据记录
void Send_Data2NiMing(void)
{
	uint8_t _cnt=0;
	float f_temp;
  uint8_t sum = 0;
	uint8_t i;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;    //功能字，F1为用户数据
	data_to_send[_cnt++]=0;
		
	//***发送电机角度***//
	f_temp = Motor1.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor2.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor3.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);

	f_temp = Motor4.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor5.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor6.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	//***发送电机跟踪误差***//
	f_temp = Motor1pid.err;    //发送电机跟踪误差
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor2pid.err;    //发送电机跟踪误差
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);

	f_temp = Motor3pid.err;    //发送电机跟踪误差
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor4pid.err;    //发送电机跟踪误差
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor5pid.err;    //发送电机跟踪误差
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);

	f_temp = Motor6pid.err;    //发送电机跟踪误差
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);

//	//***发送电机目标角度***//
//	f_temp = Motor1.Target;    //发送电机目标角度
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);

//	
//	f_temp = Motor2.Target;    //发送电机目标角度
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor3.Target;    //发送电机目标角度
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor4.Target;    //发送电机目标角度
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);

//	
//	f_temp = Motor5.Target;    //发送电机目标角度
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
//	
//	f_temp = Motor6.Target;    //发送电机目标角度
//	data_to_send[_cnt++]=BYTE3(f_temp);
//	data_to_send[_cnt++]=BYTE2(f_temp);
//	data_to_send[_cnt++]=BYTE1(f_temp);
//	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
	
	//***发送电机电流***//
	_cnt = 0;sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF2;    //功能字，F2为用户数据
	data_to_send[_cnt++]=0;
		
	f_temp = Motor1.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor2.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor3.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor4.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor5.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor6.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	data_to_send[3] = _cnt-4;
		
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
}

//发送给上位机信息
void Send_Data2PC(void)     
{
	uint8_t _cnt=0;
	float f_temp;
  uint8_t sum = 0;
	uint8_t i;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;    //功能字，F1为用户数据
	data_to_send[_cnt++]=0;
		
	f_temp = Motor1.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor2.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor3.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
//	

	f_temp = Motor4.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor5.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	
	f_temp = Motor6.angle;    //发送电机角度
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);		
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
	

	//***发送电机电流***//
	_cnt = 0;sum = 0;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF2;    //功能字，F2为电流数据
	data_to_send[_cnt++]=0;
		
	f_temp = Motor1.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor2.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor3.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor4.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor5.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	f_temp = Motor6.curent;    //发送电机电流
	data_to_send[_cnt++]=BYTE3(f_temp);
	data_to_send[_cnt++]=BYTE2(f_temp);
	data_to_send[_cnt++]=BYTE1(f_temp);
	data_to_send[_cnt++]=BYTE0(f_temp);
	
	data_to_send[3] = _cnt-4;
		
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
}


//匿名上位机读取电机123参数
void Send_PID1_to_PC(void)
{
	uint8_t _cnt=0;
	int16_t _temp;
  uint8_t sum = 0;
	uint8_t i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;    //功能字，10为PID1
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

// 匿名上位机读取电机456的PID参数
void Send_PID2_to_PC(void)
{
	uint8_t _cnt=0;
	int16_t _temp;
  uint8_t sum = 0;
	uint8_t i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;    //功能字，10为PID1
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


/*****************************************              编码器信号更新                       ***************************/
/////////////////////
void Encoder_value_updata(void)
{
  //电机1速度角度更新
  Angle_and_Speed_caculate(TIM1,&Motor1);
	
   //电机2速度角度更新
  Angle_and_Speed_caculate(TIM2,&Motor2);
	
   //电机3速度角度更新
  Angle_and_Speed_caculate(TIM3,&Motor3);
	
  //电机4速度角度更新
  Angle_and_Speed_caculate(TIM4,&Motor4);
	
	//电机5速度角度更新
  Angle_and_Speed_caculate(TIM5,&Motor5);
	
	//电机6速度角度更新
  Angle_and_Speed_caculate(TIM8,&Motor6);
	
}

/*****************************************              编码器信号更新                       ***************************/

/*****************************************              电流信号更新                        ***************************/
void Current_updata(void)
{
	uint32_t Data[8];
	float Vrefint = 0;
	SCB_InvalidateDCache_by_Addr((uint32_t *)ADC1_Data,sizeof(ADC1_Data));   /* 将内存0x38000000内存的变量缓存到Cache */
	SCB_InvalidateDCache_by_Addr((uint32_t *)ADC3_Data,sizeof(ADC3_Data));   /* 将内存0x38000000内存的变量缓存到Cache */	
	/* 将内存0x38000000的全局变量移到DTCM内存 */
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
	
	//记录电流限幅时刻的PWM（最大电流为2.9A）
	if((Motor1.curent > 2.8)||(Motor1.curent < -2.8)) Motor1.I_lim_PWM =  Motor1.PWM;  else Motor1.I_lim_PWM = 11999;
	if((Motor2.curent > 2.8)||(Motor2.curent < -2.8)) Motor2.I_lim_PWM =  Motor2.PWM;  else Motor2.I_lim_PWM = 11999;
	if((Motor3.curent > 2.8)||(Motor3.curent < -2.8)) Motor3.I_lim_PWM =  Motor3.PWM;  else Motor3.I_lim_PWM = 11999;
	if((Motor4.curent > 2.8)||(Motor4.curent < -2.8)) Motor4.I_lim_PWM =  Motor4.PWM;  else Motor4.I_lim_PWM = 11999;
	if((Motor5.curent > 2.8)||(Motor5.curent < -2.8)) Motor5.I_lim_PWM =  Motor5.PWM;  else Motor5.I_lim_PWM = 11999;
	if((Motor6.curent > 2.8)||(Motor6.curent < -2.8)) Motor6.I_lim_PWM =  Motor6.PWM;  else Motor6.I_lim_PWM = 11999;

}

/*****************************************              电流信号更新                        ***************************/

/*****************************************              匿名上位机通讯                       ***************************/

///接收数据初步处理
void Received_data_check(uint8_t Receive_data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	static uint8_t cnt_tiaoshi = 0;
	receive_buff_tiaoshi[cnt_tiaoshi++] = Receive_data;
	if(cnt_tiaoshi>5) cnt_tiaoshi= 0;
	
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
	else if(state==2&&Receive_data<0xF1)//功能码
	{
		state=3;
		RxBuffer[2]=Receive_data;
	}
	else if(state==3&&Receive_data<50)//数据长度
	{
		state = 4;
		RxBuffer[3]=Receive_data;
		_data_len = Receive_data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)  //接收数据
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=Receive_data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)               //校验和
	{
		state = 0;
		RxBuffer[4+_data_cnt]=Receive_data;
		Received_data_progress(RxBuffer,_data_cnt+5);
		
	}
	else
		state = 0;
}

//接收指令的具体处理
void Received_data_progress(uint8_t *data_buf, uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	

	
	
	//************  PID参数  ***********//
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
		
		if(*(data_buf+2)==0x22)								//上位机控制0x22
    {
        Motor1pid.U_From_Pc  = ( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
        Motor2pid.U_From_Pc  = ( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
        Motor3pid.U_From_Pc  = ( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
        Motor4pid.U_From_Pc = ( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
        Motor5pid.U_From_Pc = ( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
        Motor6pid.U_From_Pc = ( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
    }
		//往上位机发送PID
		if(*(data_buf+2)==0X02)
		{
			if(*(data_buf+4)==0X01)  //发送读取PID的指令
			{
				f.send_pid1 = 1;
				f.send_pid2 = 1;
			}
		}
		//************  PID参数  ***********//
		
		//*********   上位机界面指令   ************//
		if(*(data_buf+2)==0X01)
		{			
			if(*(data_buf+4)==0XA1)		                    //初始化时位置调整
		  {
					uint8_t Instruction = 0;
				  //接收指令数据0x01-0x20
					Instruction = *(data_buf+5);  
				  //电机使能
				  if(Position_zeroed_Flag == 0)//开始时才可以调整
					Motor_AdustedFlag_from_PC(Instruction);
          Check_Back();				
			}		 
			
			if(*(data_buf+4)==0XA2)		                    //电机角度清零指令
			{
				//启动角度清零
				angle_Clear_Flag = 1;
        UI.angle_Clear_Flag	= 1;
        Check_Back();					
			}
				
			if(*(data_buf+4)==0XA3)		                     //初始化完成指令
			{
				Position_zeroed_Flag = 1;
				target_followed_flag = 0;
				//初始化计数值为0······
				TIM1->CNT = 0x7fff;
				TIM2->CNT = 0x7fff;
				TIM3->CNT = 0x7fff;
				TIM4->CNT = 0x7fff;
				TIM5->CNT = 0x7fff;
				TIM8->CNT = 0x7fff;
				Check_Back();	
			}
			if(*(data_buf+4)==0XA4)		                      //手动康复模式开始（自动康复结束）
			{
				AutoKangFu_mode        = 0;		
        UI.AutoKangFu_mode     = 0;
			  ShouDongKangFu_mode    = 1;
				UI.ShouDongKangFu_mode = 1;
				Check_Back();	
			}
			if(*(data_buf+4)==0XA5)		                      //自动康复模式开始
			{
				ShouDongKangFu_mode    = 0;
				UI.ShouDongKangFu_mode = 0;
				AutoKangFu_mode        = 1;		
        UI.AutoKangFu_mode     = 1;
			  Check_Back();		
			}
			if(*(data_buf+4)==0XA6)		                      //康复腿及康复运动选择
			{
				uint8_t Instruction = 0;
				//接收指令数据0x01-0x09
				Instruction = *(data_buf+5);  
				//电机使能
				Motor_Enable(Instruction);
				UI.Para_Send_Flag      = Instruction;
				AutoKangFu_mode        = 0;		
        UI.AutoKangFu_mode     = 0;
			  ShouDongKangFu_mode    = 0;
				UI.ShouDongKangFu_mode = 0;
        Check_Back();					
			}
			if(*(data_buf+4)==0XA7)		                     //手动康复速度
			{
				uint8_t Instruction = 0;
				//接收指令数据0x01-0x09
				Instruction = *(data_buf+5);  
				switch(Instruction)
				{
					case 0x01:
						//低速
						ShouDong_Speed = 1500;
					  UI.ShouDongKangFu_SpeedSet_Flag = 0;
						break;
					case 0x02:
						//中速
						ShouDong_Speed = 2500;
					  UI.ShouDongKangFu_SpeedSet_Flag = 1;
						break;
					case 0x03:
						//高速
						ShouDong_Speed = 3500;
					  UI.ShouDongKangFu_SpeedSet_Flag = 2;
						break;
					case 0x04:
						//电流限幅测试
						ShouDong_Speed = 10000;
					  UI.ShouDongKangFu_SpeedSet_Flag = 3;
						break;
				}
			  ShouDongKangFu_mode    = 0;
				UI.ShouDongKangFu_mode = 0;				
				Check_Back();	
			}
			if(*(data_buf+4)==0XA8)		                     //手动康复结束
			{
				angle_Clear_Flag    = 1;
				UI.angle_Clear_Flag = 1;
				//电机失能
				Motor_Enable(0);
				//手动康复模式关闭
				ShouDongKangFu_mode    = 0;
				UI.ShouDongKangFu_mode = 0;
				//速度失能
				ShouDong_Speed = 0;
				Check_Back();	
	    }
			
			if(*(data_buf+4)==0XA9)		                     //自动康复结束
			{
				angle_Clear_Flag    = 1; 
				UI.angle_Clear_Flag = 1;
				//电机失能
				Motor_Enable(0);
				//手动康复模式关闭
				AutoKangFu_mode    = 0;
				UI.AutoKangFu_mode = 0;
				//自动康复次数
				AutoKangFu_CiShu    = 0;
				UI.AutoKangFu_CiShu = 0;
				Check_Back();	
	    }
			if(*(data_buf+4)==0XAA)		                     //自动康复次数
			{
				//接收指令数据0x01-0x09
				AutoKangFu_CiShu    = *(data_buf+5);
        UI.AutoKangFu_CiShu = *(data_buf+5);
				AutoKangFu_mode        = 0;		
        UI.AutoKangFu_mode     = 0;
				Check_Back();	
	    }
			if(*(data_buf+4)==0XAB)		                     //自动康复曲线类型
			{						
				if((UI.Curve_Type - *(data_buf+5)))
				{
					//接收指令数据0x00为sin，0x01为步态
					UI.Curve_Type = *(data_buf+5);
					//若曲线不一致，需要重新进行逆运动学解算生成曲线
					UI.Curve_Generation_Flag = 1;        					
				}
				AutoKangFu_mode        = 0;		
        UI.AutoKangFu_mode     = 0;
				Check_Back();	
	    }
			
	 }
}
//把状态反馈回上位机
void Check_Back(void)
{
	//单片机接收到数据之后状态发送成功标志
	uint8_t _cnt = 0;
  uint8_t sum  = 0;
	uint8_t i;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF3;    //功能字，发送之后返回校验
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++] = Position_zeroed_Flag;
	data_to_send[_cnt++] = UI.angle_Clear_Flag ;
	data_to_send[_cnt++] = UI.ShouDongKangFu_mode;
	data_to_send[_cnt++] = UI.AutoKangFu_mode;
	
	data_to_send[_cnt++] = UI.ShouDongKangFu_SpeedSet_Flag;
	data_to_send[_cnt++] = UI.AutoKangFu_CiShu;
	data_to_send[_cnt++] = UI.Curve_Type;
	data_to_send[_cnt++] = UI.Para_Send_Flag;
	
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	
	data_to_send[_cnt++] = Motor1.Enable_Flag;
	data_to_send[_cnt++] = Motor2.Enable_Flag;
	data_to_send[_cnt++] = Motor3.Enable_Flag;
	data_to_send[_cnt++] = Motor4.Enable_Flag;
	
	data_to_send[_cnt++] = Motor5.Enable_Flag;
	data_to_send[_cnt++] = Motor6.Enable_Flag;
	data_to_send[_cnt++] = LeftLeg_NeiWaiXuan ;
	data_to_send[_cnt++] = RightLeg_NeiWaiXuan;
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);
	
}

//电机使能
void Motor_Enable(uint8_t data)
{
	//初始化失能
	Motor1.Enable_Flag = 0;
	Motor2.Enable_Flag = 0;
	Motor3.Enable_Flag = 0;
	Motor4.Enable_Flag = 0;
	Motor5.Enable_Flag = 0;
	Motor6.Enable_Flag = 0;
	//根据接收数据使能
	switch(data)
	{
		case 0x01://motor1
		{
			Motor1.Enable_Flag = 1;	
			Motor3.Enable_Flag = 1;	
			LeftLeg_NeiWaiXuan = 0;
			break;	
		}
		case 0x02://motor2
		{
			Motor2.Enable_Flag = 1;
			LeftLeg_NeiWaiXuan = 0;
			break;			
		}
		case 0x03://motor3
		{
			Motor1.Enable_Flag = 1;	
			Motor2.Enable_Flag = 1;	
			Motor3.Enable_Flag = 1;	
			LeftLeg_NeiWaiXuan = 1;
			break;		
		}
		case 0x04://motor4
		{
			Motor4.Enable_Flag = 1;	
			Motor6.Enable_Flag = 1;
			RightLeg_NeiWaiXuan = 0;
			break;		
		}
		case 0x05://motor5
		{
			Motor5.Enable_Flag = 1;
			RightLeg_NeiWaiXuan = 0;
			break;		
		}
		case 0x06://motor6
		{
			Motor4.Enable_Flag = 1;	
			Motor5.Enable_Flag = 1;
			Motor6.Enable_Flag = 1;
			RightLeg_NeiWaiXuan = 1;
			break;		
		}
		case 0x07://motor6
		{
			Motor1.Enable_Flag = 1;	
			Motor3.Enable_Flag = 1;
			Motor4.Enable_Flag = 1;
			Motor6.Enable_Flag = 1;
			LeftLeg_NeiWaiXuan = 0;
			RightLeg_NeiWaiXuan = 0;
			break;		
		}
		case 0x08://motor6
		{
			Motor2.Enable_Flag = 1;
			Motor5.Enable_Flag = 1;
			LeftLeg_NeiWaiXuan = 0;
			RightLeg_NeiWaiXuan = 0;
			break;		
		}
		case 0x09://motor6
		{
			Motor1.Enable_Flag = 1;
			Motor2.Enable_Flag = 1;
			Motor3.Enable_Flag = 1;
			Motor4.Enable_Flag = 1;
			Motor5.Enable_Flag = 1;
			Motor6.Enable_Flag = 1;
			LeftLeg_NeiWaiXuan = 1;
			RightLeg_NeiWaiXuan = 1;
			break;		
		}
		break;
	}
}

//PID发送校验位
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

/*****************************************              匿名上位机通讯                       ***************************/

/*****************************************              九轴传感器通讯                       ***************************/



//CopeSerialData为串口3中断调用函数，串口每收到一个数据，调用一次这个函数。
void Receive_Data_From_JY901_left(uint8_t ucData)
{
	static uint8_t ucRxBuffer[250];
	static uint8_t ucRxCnt = 0;
	static float Roll1_x ,Pitch1_y ,Yaw1_z ;
  uint8_t Roll_L,Roll_H,Pitch_L,Pitch_H,Yaw_L,Yaw_H;
	unsigned char Sum=0,i;
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
  if(ucRxCnt<11){return;}                      //一组是十一个数据，若不是，则返回}
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
		//旋转顺序为z-y-x，也就是gamma-alpha-beta，由于欧拉角原因，角度范围最好为（+-90）*（+-60）*（+-90）伽马角在90附近影响x
		Roll1_x=(((short)Roll_H<<8)|Roll_L)*0.0055f;
		Pitch1_y = (((short)Pitch_H<<8)|Pitch_L)*0.0055f;
		Yaw1_z = (((short)Yaw_H<<8)|Yaw_L)*0.0055f;
		
		Right_Leg.Roll_x = Right_Leg.Roll_x*0.7f+Roll1_x*0.3f;
		Right_Leg.Pitch_y = Right_Leg.Pitch_y*0.7f+Pitch1_y*0.3f;
		Right_Leg.Yaw_z = Right_Leg.Yaw_z*0.7f+Yaw1_z*0.3;

		ucRxCnt=0;//清空缓存区
}
}


/*****************************************              九轴传感器通讯                       ***************************/


