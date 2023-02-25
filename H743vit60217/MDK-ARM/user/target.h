#ifndef __TARGET_H
#define __TARGET_H

#include "Inv_Kine.h"

float curve_Sin( float Amp , float w,float t);
float target_Progress(float t1);

void motor_Angle_d_Caculate(float time,Inv_Kine* leg, int flag);
void inv_To_PC(void);




#endif




