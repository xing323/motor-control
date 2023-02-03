//调试包含头文件
#include "main.h"
#include "Dataex.h"
#include "usart.h"
//调试包含头文件
#include  "math.h"
#include "Inv_Kine.h"
#include "target.h"

#define PI 3.141593

extern uint8_t data_to_send[255];  

/**********  生成幅值为Amp 周期为w的正弦轨迹  **********/
float curve_Sin( float Amp , float w,float t)
{
	float target = 0.0174533f*Amp*sin(2*PI*w*t);           //0.0174533为Π/180的值，化为弧度
	return target;
}

/**********  人腿的轨迹  **********/
float target_Progress(float t1)
{
		float a0 = 8361;
		float a1 = 12070;
		float b1 = -3541;
		float a2 = -2248;
		float b2 = -192.7;
		float a3 = -208;
		float b3 = 984.3;
		float a4 = -373.9;
		float b4 = 227.1;
		float a5 =  -54.2;
		float b5 =  285.7;
		float phy = 3.4; 
    float q1=0;	
		q1=(float)0.001f*(a0+a1*cos(6.283f*t1+phy)+b1*sin(6.283f*t1+phy)+a2*cos(6.283f*2*t1+phy)+b2*sin(6.283f*2*t1+phy)+a3*cos(6.283f*3*t1+phy)+b3*sin(6.283f*3*t1+phy)+a4*cos(6.283f*4*t1+phy)+b4*sin(6.283f*4*t1+phy)+a5*cos(6.283f*5*t1+phy)+b5*sin(6.283f*5*t1+phy));
    return q1;
}


/**********  实验数据轨迹生成  **********/

void motor_Angle_d_Caculate(float time,Inv_Kine* leg, int flag)
{
	unsigned int i;
	float alpha,beta,gamma;
	float t;
	switch(flag)
	{
		case 0:
			for(i=0;i<time*1000;i++)
			{
				t = 0.001*i;
				alpha = curve_Sin(30, 1, t);
				beta =  0;
				gamma = 0;
				Inv_Kine_Caculator(alpha, beta, gamma, leg);
				leg->angle_M1_d[i] = 57.2958f* 20* leg->theta1;
				leg->angle_M2_d[i] = 57.2958f* 20* leg->theta2;
				leg->angle_M3_d[i] = 57.2958f* 20* leg->theta3;
			}
			break;
		case 1:
			for(i=0;i<time*1000;i++)
			{
				t = 0.001*i;
				alpha = 0;
				beta =  curve_Sin(30, 1, t);
				gamma = 0;
				Inv_Kine_Caculator(alpha, beta, gamma, leg);
				leg->angle_M1_d[i] = 57.2958f* 20* leg->theta1;
				leg->angle_M2_d[i] = 57.2958f* 20* leg->theta2;
				leg->angle_M3_d[i] = 57.2958f* 20* leg->theta3;
			}
			break;
		case 2:
			for(i=0;i<time*1000;i++)
			{
				t = 0.001*i;
				alpha = 0;
				beta =  0;
				gamma = curve_Sin(30, 1, t);
				Inv_Kine_Caculator(alpha, beta, gamma, leg);
				leg->angle_M1_d[i] = 57.2958f* 20* leg->theta1;
				leg->angle_M2_d[i] = 57.2958f* 20* leg->theta2;
				leg->angle_M3_d[i] = 57.2958f* 20* leg->theta3;
			}
			break;
		case 3:
			for(i=0;i<time*1000;i++)
			{
				t = 0.001*i;
				alpha = curve_Sin(10, 1, t);
				beta =  curve_Sin(10, 1, t);
				gamma = curve_Sin(10, 1, t);
				Inv_Kine_Caculator(alpha, beta, gamma, leg);
				leg->angle_M1_d[i] = 57.2958f* 20* leg->theta1;
				leg->angle_M2_d[i] = 57.2958f* 20* leg->theta2;
				leg->angle_M3_d[i] = 57.2958f* 20* leg->theta3;
			}
			break;
	}
	//最后一个值赋值为0
	leg->angle_M1_d[(uint16_t)(time*1000-1)] = 0;
	leg->angle_M2_d[(uint16_t)(time*1000-1)] = 0;
	leg->angle_M3_d[(uint16_t)(time*1000-1)] = 0;
}

//传输数据到电脑上看数据生成对不对,数据生成之后调用这个函数把数据发电脑上分析
void inv_To_PC(void)
{
	uint8_t _cnt=0;
	float f_temp;
  uint8_t sum = 0;
	uint8_t i;
	static uint16_t data_cnt = 0;

		data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0xF1;    //功能字，F1为用户数据
		data_to_send[_cnt++]=0;
		
		f_temp = Left_InvKine.angle_M1_d[data_cnt];    //发送电机角度
		data_to_send[_cnt++]=BYTE3(f_temp);
		data_to_send[_cnt++]=BYTE2(f_temp);
		data_to_send[_cnt++]=BYTE1(f_temp);
		data_to_send[_cnt++]=BYTE0(f_temp);
		

		
		f_temp = Left_InvKine.angle_M2_d[data_cnt];;    //发送电机角度
		data_to_send[_cnt++]=BYTE3(f_temp);
		data_to_send[_cnt++]=BYTE2(f_temp);
		data_to_send[_cnt++]=BYTE1(f_temp);
		data_to_send[_cnt++]=BYTE0(f_temp);
		
		
		f_temp = Left_InvKine.angle_M3_d[data_cnt];;    //发送电机角度
		data_to_send[_cnt++]=BYTE3(f_temp);
		data_to_send[_cnt++]=BYTE2(f_temp);
		data_to_send[_cnt++]=BYTE1(f_temp);
		data_to_send[_cnt++]=BYTE0(f_temp);
	//	

		f_temp = Right_InvKine.angle_M1_d[data_cnt];    //发送电机角度
		data_to_send[_cnt++]=BYTE3(f_temp);
		data_to_send[_cnt++]=BYTE2(f_temp);
		data_to_send[_cnt++]=BYTE1(f_temp);
		data_to_send[_cnt++]=BYTE0(f_temp);
		
		
		f_temp = Right_InvKine.angle_M2_d[data_cnt];    //发送电机角度
		data_to_send[_cnt++]=BYTE3(f_temp);
		data_to_send[_cnt++]=BYTE2(f_temp);
		data_to_send[_cnt++]=BYTE1(f_temp);
		data_to_send[_cnt++]=BYTE0(f_temp);
		
		
		f_temp = Right_InvKine.angle_M3_d[data_cnt++];    //发送电机角度
		data_to_send[_cnt++]=BYTE3(f_temp);
		data_to_send[_cnt++]=BYTE2(f_temp);
		data_to_send[_cnt++]=BYTE1(f_temp);
		data_to_send[_cnt++]=BYTE0(f_temp);
		
		if(data_cnt>2999) data_cnt = 0;
		
		data_to_send[3] = _cnt-4;
		
		
		for(i=0;i<_cnt;i++)
			sum += data_to_send[i];
		data_to_send[_cnt++]=sum;
		
		HAL_UART_Transmit(&huart1, (uint8_t *)&data_to_send, _cnt ,10);

}


