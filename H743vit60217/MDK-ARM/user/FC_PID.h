#ifndef FC_PID_H
#define FC_PID_H

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

//模糊控制结构体
typedef struct{
	float e;     //误差
	float e_pre; //上一次的误差
	float de;    //误差的变化率
	float emax;  //误差基本论域上限
	float demax; //误差变化率基本论域的上限
	float umax;  //输出的上限
	float Ke;    //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	float Kde;   //Ke=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
	float Ku;    //Ke=umax/n,量化论域为[-3,-2,-1,0,1,2,3]
	float *e_mf_paras; //误差的隶属度函数的参数
	float *de_mf_paras;//误差的偏差隶属度函数的参数
	float *KP_mf_paras; //输出KP的隶属度函数的参数
	float *KI_mf_paras; //输出KI的隶属度函数的参数
	float *KD_mf_paras; //输出KD的隶属度函数的参数
	int rule_KP[7][7];
	int rule_KI[7][7];
	int rule_KD[7][7];

}FC_controller;	

extern FC_controller FC_M1,FC_M3,FC_M4,FC_M6;

void fuzzy_controler_Init(FC_controller* FC_M);
void Fuzzy_controller_setRule(int rule[][7],int rulelist[][7]);
float Fuzzy_controller_trimf(float x,float a,float b,float c);
void Fuzzy_controller_setMf(FC_controller* FC_e,float *mf);
float Fuzzy_controller_realize(int rule[][7],float e,float de); 
float limit_Float(float a, float up, float down);
void Control_FC_update(uint8_t period_num );
void FC_PD_Target_update(uint16_t cnt,float time);
void FC_qiankui_Caculate(void);


float plant_Test(float u);





//之前的模糊规则
//int ruleMatrix_KP[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
//												{PB,PB,PM,PS,PS,ZO,NS},
//												{PM,PM,PM,PS,ZO,NS,NS},
//												{PM,PM,PS,ZO,NS,NM,NM},
//												{PS,PS,ZO,NS,NS,NM,NM},
//												{PS,ZO,NS,NM,NM,NM,NB},
//												{ZO,ZO,NM,NM,NM,NB,NB}};;//模糊规则表
//int ruleMatrix_KI[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
//												{NB,NB,NM,NS,NS,ZO,PS},
//												{NM,NM,NM,NS,ZO,PS,PS},
//												{NM,NM,NS,ZO,PS,PM,PM},
//												{NS,NS,ZO,PS,PS,PM,PM},
//												{NS,ZO,PS,PM,PM,PM,PB},
//												{ZO,ZO,PM,PM,PM,PB,PB}};//模糊规则表
//int ruleMatrix_KD[7][7]={{PS,NS,NB,NB,NB,NM,PS},
//												{PS,NS,NB,NM,NM,NS,ZO},
//												{ZO,NS,NM,NM,NS,NS,ZO},
//												{ZO,NS,NS,NS,NS,NS,ZO},
//												{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
//												{PB,NS,PS,PS,PS,PS,PB},
//												{PB,PM,PM,PM,PS,PS,PB}};//模糊规则表




#endif
