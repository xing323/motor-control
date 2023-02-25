#ifndef INV_KINE_H
#define INV_KINE_H

#include "math.h"


typedef struct{
	float alpha;           
	float beta;           
	float gamma;	        
	float theta1;     
	float theta2;
  float theta3;
	float angle_M1_d[3000];
	float angle_M2_d[3000];
	float angle_M3_d[3000];
	float angle_M1_dv[3000];
	float angle_M2_dv[3000];
	float angle_M3_dv[3000];
}Inv_Kine;	

extern Inv_Kine Left_InvKine,Right_InvKine;




void Inv_Kine_Caculator(float alpha,float beta,float gamma,Inv_Kine* A);









#endif
