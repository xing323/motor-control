#ifndef DATAEX_H
#define DATAEX_H

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef struct 
{
		uint8_t send_2PC;
		uint8_t Curent;
		uint8_t send_pid1;
		uint8_t send_pid2;
		uint8_t Control_Inc;
	  uint8_t Encoder;
	  uint8_t u_Tran_End_Flag;
	  uint8_t Inv_updata;
	  uint8_t Position_zeroed_Flag;

}dt_flag_t;
extern dt_flag_t f;
//九轴传感器结构体
typedef struct 
{
	float  Roll_x;
	float  Pitch_y;
	float  Yaw_z;

}JY901_angle;

extern JY901_angle Left_Leg,Right_Leg;

void Data_Exchange(void);
void Send_Data2PC(void);     //腿的角度
void Send_PWM1(void);     //腿的角度
void Send_PID1_to_PC(void);
void Send_PID2_to_PC(void);
void Encoder_value_updata(void);
void Usart1_user_send(uint16_t send_target_count);   //发送target值
void Received_data_check(uint8_t Receive_data);
void Received_data_progress(uint8_t *data_buf, uint8_t num);
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum);
void Receive_Data_From_JY901_left(uint8_t ucData);
void Current_updata(void);

#endif
