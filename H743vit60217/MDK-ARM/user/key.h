#ifndef __KEY_H
#define __KEY_H

uint8_t Key_Scan(void);
uint8_t Motor1_zeroed(void);
uint8_t Motor2_zeroed(void);
uint8_t Motor3_zeroed(void);
uint8_t Motor4_zeroed(void);
uint8_t Motor5_zeroed(void);
uint8_t Motor6_zeroed(void);

void ShouDongQuShenZhanShou_Function(int PWM);
void ShouDongNeiWaiXuan_Function(int PWM);
uint8_t  Key_switch(uint8_t flag);

void Delay_10ms(void);

//电机动作函数
void Motor_Adusted_from_PC(void);
////上位机接口函数
/* 根据接收的数据调整使能并调整电机动作*/
void Motor_AdustedFlag_from_PC(uint8_t data);


#endif
