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

//�����������
void Motor_Adusted_from_PC(void);
////��λ���ӿں���
/* ���ݽ��յ����ݵ���ʹ�ܲ������������*/
void Motor_AdustedFlag_from_PC(uint8_t data);


#endif
