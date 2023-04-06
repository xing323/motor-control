#include "main.h"
#include "FC_PID.h"
#include "Motor.h"
#include "Dataex.h"
#include "control.h"
#include "target.h"
#include "qiankui.h"
#include "usart.h"
///////////////////******PID模糊控制     Start  **************************//////

extern Motor Motor1,Motor2,Motor3,Motor4,Motor5,Motor6;
FC_controller FC_M1, FC_M3, FC_M4, FC_M6;

float e_mf_paras[21]  = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; //误差e的隶属度函数参数，这里隶属度函数为三角型，所以3个数据为一组
float de_mf_paras[21] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; //误差变化率de的模糊隶属度函数参数
float KP_mf_paras[21] = {-3, -3, -2, -3, -2, -1, -2, -1, 0, -1, 0, 1, 0, 1, 2, 1, 2, 3, 2, 3, 3}; //输出量u的隶属度函数参数


int ruleMatrix_KP[7][7] =
{
    {PB, PB, PM, PM, PS, PM, PB},
    {PB, PB, PM, PM, PS, PM, PM},
    {PM, PM, PS, PS, ZO, PS, PM},
    {NS, NS, NS, NS, NS, NS, NS},
    {PM, PS, ZO, PS, PS, PM, PM},
    {PM, PM, PS, PM, PM, PB, PB},
    {PB, PM, PS, PM, PM, PB, PB}
};//模糊规则表
int ruleMatrix_KD[7][7] =
{
    {PB, PM, PS, PM, PS, ZO, NB},
    {PB, PB, PM, PS, ZO, NS, NB},
    {PB, PM, PM, PS, ZO, NM, NB},
    {ZO, ZO, PS, ZO, PS, ZO, ZO},
    {NB, NM, ZO, PS, PM, PM, PB},
    {NB, NS, ZO, PS, PM, PB, PB},
    {NB, ZO, PS, PM, PS, PM, PB}
};//模糊规则表


void Control_FC_update(uint8_t period_num )
{
    static float    time = 0.0;
    static uint16_t folwd_cnt = 0;
	  static int      cnt = 0;                   //用来控制周期
	  static int      Angle_Safe_Value = 700;
	
    int PWM1=0, PWM2=0, PWM3=0, PWM4=0, PWM5=0, PWM6=0;
    
	  //目标值更新
	  FC_PD_Target_update(folwd_cnt, time);
    //前馈计算
	  FC_qiankui_Caculate();
	
	
    if(cnt++ % period_num == 0)folwd_cnt++;
    //3000个点为训练1次
    if(folwd_cnt > 2999)
    {
        folwd_cnt = 0;
        UI.AutoKangFu_CiShu--;
        if(!UI.AutoKangFu_CiShu) UI.angle_Clear_Flag = 1;
    }

    FC_M1.e = limit_Float(Motor1pid.err, 3, -3);     //e，和ec输入限幅
    FC_M1.de = limit_Float(Motor1pid.d_err, 3, -3);

    FC_M3.e = limit_Float(Motor3pid.err, 3, -3);     //e，和ec输入限幅
    FC_M3.de = limit_Float(Motor3pid.d_err, 3, -3);

    FC_M4.e = limit_Float(Motor4pid.err, 3, -3);     //e，和ec输入限幅
    FC_M4.de = limit_Float(Motor4pid.d_err, 3, -3);

    FC_M6.e = limit_Float(Motor6pid.err, 3, -3);     //e，和ec输入限幅
    FC_M6.de = limit_Float(Motor6pid.d_err, 3, -3);

    //输出值为-+3
    Motor1pid.Kp = 100.0f * Fuzzy_controller_realize(FC_M1.rule_KP, FC_M1.e, FC_M1.de) + 300;
    Motor1pid.Ki = 0;
    Motor1pid.Kd = 100.0f * Fuzzy_controller_realize(FC_M1.rule_KD, FC_M1.e, FC_M1.de) + 400;

    Motor3pid.Kp = 100.0f * Fuzzy_controller_realize(FC_M3.rule_KP, FC_M3.e, FC_M3.de) + 300;
    Motor3pid.Ki = 0;
    Motor3pid.Kd = 100.0f * Fuzzy_controller_realize(FC_M3.rule_KD, FC_M3.e, FC_M3.de) + 400;

    Motor4pid.Kp = 150.0f * Fuzzy_controller_realize(FC_M4.rule_KP, FC_M4.e, FC_M4.de)*0 + 60;
    Motor4pid.Ki = 0;
    Motor4pid.Kd = 100.0f * Fuzzy_controller_realize(FC_M4.rule_KD, FC_M4.e, FC_M4.de)*0 + 40;

    Motor6pid.Kp = 150.0f * Fuzzy_controller_realize(FC_M6.rule_KP, FC_M6.e, FC_M6.de) + 600;
    Motor6pid.Ki = 0;
    Motor6pid.Kd = 100.0f * Fuzzy_controller_realize(FC_M6.rule_KD, FC_M6.e, FC_M6.de)*0 + 400*0;

    //误差更

    //安全标志位的设立
    if((Motor1.angle <= Angle_Safe_Value) && (Motor1.angle >= -Angle_Safe_Value))
        Motor1.Angle_Safe_Flag = 1;
    else
        Motor1.Angle_Safe_Flag = 0;
    if((Motor2.angle <= Angle_Safe_Value) && (Motor2.angle >= -Angle_Safe_Value))
        Motor2.Angle_Safe_Flag = 1;
    else
        Motor2.Angle_Safe_Flag = 0;
    if((Motor3.angle <= Angle_Safe_Value) && (Motor3.angle >= -Angle_Safe_Value))
        Motor3.Angle_Safe_Flag = 1;
    else
        Motor3.Angle_Safe_Flag = 0;
    if((Motor4.angle <= Angle_Safe_Value) && (Motor4.angle >= -Angle_Safe_Value))
        Motor4.Angle_Safe_Flag = 1;
    else
        Motor4.Angle_Safe_Flag = 0;
    if((Motor5.angle <= Angle_Safe_Value) && (Motor5.angle >= -Angle_Safe_Value))
        Motor5.Angle_Safe_Flag = 1;
    else
        Motor5.Angle_Safe_Flag = 0;
    if((Motor6.angle <= Angle_Safe_Value) && (Motor6.angle >= -Angle_Safe_Value))
        Motor6.Angle_Safe_Flag = 1;
    else
        Motor6.Angle_Safe_Flag = 0;


    Motor1.U = (int16_t)(QK_M1.U_k + Motor_x_control_pid_PWM(&Motor1pid, Motor1.Target, Motor1.angle))* 0.2 + Motor1.U * 0.8;
    Motor2.U = (int16_t)(QK_M2.U_k + Motor_x_control_pid_PWM(&Motor2pid, Motor2.Target, Motor2.angle))* 0.2 + Motor2.U * 0.8;
    Motor3.U = (int16_t)(QK_M3.U_k + Motor_x_control_pid_PWM(&Motor3pid, Motor3.Target, Motor3.angle))* 0.2 + Motor3.U * 0.8;
    Motor4.U = (int16_t)(QK_M4.U_k + Motor_x_control_pid_PWM(&Motor4pid, Motor4.Target, Motor4.angle))* 0.2 + Motor4.U * 0.8;
		Motor5.U = (int16_t)(QK_M5.U_k + Motor_x_control_pid_PWM(&Motor5pid, Motor5.Target, Motor5.angle))* 0.2 + Motor5.U * 0.8;
    Motor6.U = (int16_t)(QK_M6.U_k + Motor_x_control_pid_PWM(&Motor6pid, Motor6.Target, Motor6.angle))* 0.2 + Motor6.U * 0.8;

    PWM1 = Motor1.U * Motor1.Enable_Flag * Motor1.Angle_Safe_Flag ;
    PWM2 = Motor2.U * Motor2.Enable_Flag * Motor2.Angle_Safe_Flag ;
    PWM3 = Motor3.U * Motor3.Enable_Flag * Motor3.Angle_Safe_Flag ;
    PWM4 = Motor4.U * Motor4.Enable_Flag * Motor4.Angle_Safe_Flag ;
    PWM5 = Motor5.U * Motor5.Enable_Flag * Motor5.Angle_Safe_Flag ;
    PWM6 = Motor6.U * Motor6.Enable_Flag * Motor6.Angle_Safe_Flag ;

    Motorx_Ctrl(1, PWM1);
    Motorx_Ctrl(2, PWM2);
    Motorx_Ctrl(3, PWM3);
    Motorx_Ctrl(4, PWM4);
    Motorx_Ctrl(5, PWM5);
    Motorx_Ctrl(6, PWM6);
		
		for(int j=30000;j>0;j--);

	
    //输出监控
    //	printf("motor1:%d,motor3:%d\n\n",TIM12->CCR1,TIM13->CCR1);
    //	printf("motor4:%d,motor6:%d\n\n",TIM14->CCR1,TIM15->CCR2);

    //printf("motor4KP:%d,motor6KP:%d\n\n",(int)(Motor4pid.Kp*10),(int)(Motor6pid.Kp*10));

   //printf("QK_M4.U_k:%d\n\n",(int)(QK_M4.U_k));
   time = time + 0.005f;
}

void fuzzy_controler_Init(FC_controller *FC_M)
{
    FC_M->emax = 9;
    FC_M->demax = 9;
    FC_M->umax = 9;
    FC_M->Ke = 3;            //量化系数
    FC_M->Kde = 3;
    FC_M->Ku = 3;
    Fuzzy_controller_setRule(FC_M->rule_KP, ruleMatrix_KP);   //模糊规则
    //	Fuzzy_controller_setRule(FC_M->rule_KI,ruleMatrix_KI);
    Fuzzy_controller_setRule(FC_M->rule_KD, ruleMatrix_KD);
    Fuzzy_controller_setMf(FC_M, e_mf_paras);
}


//三角隶属度函数
float Fuzzy_controller_trimf(float x, float a, float b, float c)
{
    float u;
    if(x >= a && x < b)
        u = (x - a) / (b - a);
    else if(x > b && x <= c)
        u = (c - x) / (c - b);
    else if(x == b)
        u = 1.0;
    else
        u = 0.0;
    return u;
}


//设置模糊规则   5*5
void Fuzzy_controller_setRule(int rule[][7], int rulelist[][7])
{
    for(int i = 0; i < 7; i++)
    {
        for(int j = 0; j < 7; j++)
        {
            rule[i][j] = rulelist[i][j];
        }
    }
}

//设置模糊隶属度函数的类型和参数
void Fuzzy_controller_setMf(FC_controller *FC_e, float *mf)
{
    for(int i = 0; i < 7 * 3; i++)
    {
        FC_e->e_mf_paras[i] = mf[i];
        FC_e->de_mf_paras[i] = mf[i];
        FC_e->KP_mf_paras[i] = mf[i];
        FC_e->KI_mf_paras[i] = mf[i];
        FC_e->KD_mf_paras[i] = mf[i];
    }
}


//实现模糊控制
float Fuzzy_controller_realize(int rule[][7], float e, float de)
{
    float u_e[7], u_de[7];
    int u_e_index[3], u_de_index[3];  //假设一个输入最多激活3个模糊子集
    float u;
    int M = 3;
    int j = 0;
    for(int i = 0; i < 7; i++)
    {
        u_e[i] = Fuzzy_controller_trimf(e, e_mf_paras[i * M], e_mf_paras[i * M + 1], e_mf_paras[i * M + 2]); //e模糊化，计算它的隶属度
        if(u_e[i] != 0)
        {
            u_e_index[j++] = i;                                            //存储被激活的模糊子集的下标，可以减小计算量
        }

    }
    for(; j < 3; j++)u_e_index[j] = 0;


    j = 0;
    for(int i = 0; i < 7; i++)
    {
        u_de[i] = Fuzzy_controller_trimf(de, de_mf_paras[i * M], de_mf_paras[i * M + 1], de_mf_paras[i * M + 2]); //de模糊化，计算它的隶属度
        if(u_de[i] != 0)
        {
            u_de_index[j++] = i;                                                  //存储被激活的模糊子集的下标，可以减小计算量
        }
    }

    for(; j < 3; j++)u_de_index[j] = 0;

    float den = 0, num = 0;
    for(int m = 0; m < 3; m++)
    {
        for(int n = 0; n < 3; n++)
        {
            num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * rule[u_e_index[m]][u_de_index[n]]; //加权平均法计算输出，输出隶属度相当于输入的积
            den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
        }
    }
    u = num / den;
    return u;
}

/***********************************************************
*@fuction	:FC_PD_Target_update
*@brief		:目标值更新
*@param		:uint16_t cnt,float time
*@return	:void
*@author	:xing
*@date		:2023-04-03
***********************************************************/

void FC_PD_Target_update(uint16_t cnt,float time)
{
	if(time <= 2)
    {
        Motor1.Target = +Left_InvKine.angle_M1_d[cnt]  * 0.5 * (1 + sin(3.1416 * (time * 0.5 - 0.5)));
        Motor2.Target =  Left_InvKine.angle_M2_d[cnt]  * 0.5 * (1 + sin(3.1416 * (time * 0.5 - 0.5)));
        Motor3.Target = -Left_InvKine.angle_M3_d[cnt]  * 0.5 * (1 + sin(3.1416 * (time * 0.5 - 0.5)));
        Motor4.Target = +Right_InvKine.angle_M1_d[cnt] * 0.5 * (1 + sin(3.1416 * (time * 0.5 - 0.5)));
        Motor5.Target =  Right_InvKine.angle_M2_d[cnt] * 0.5 * (1 + sin(3.1416 * (time * 0.5 - 0.5)));
        Motor6.Target = -Right_InvKine.angle_M3_d[cnt] * 0.5 * (1 + sin(3.1416 * (time * 0.5 - 0.5)));
    }
    else
    {
        Motor1.Target = Left_InvKine.angle_M1_d[cnt];
        Motor2.Target = Left_InvKine.angle_M2_d[cnt];
        Motor3.Target = Left_InvKine.angle_M3_d[cnt];
        Motor4.Target = Right_InvKine.angle_M1_d[cnt];
        Motor5.Target = Right_InvKine.angle_M2_d[cnt];
        Motor6.Target = Right_InvKine.angle_M3_d[cnt];
    }
}

void FC_qiankui_Caculate(void)
{
	    //前馈计算
    QK_M1.U_k = Qiankui_Caculate(&QK_M1);
    Qiankui_ParaUpdate(&QK_M1,  Motor1.Target);

    QK_M2.U_k = Qiankui_Caculate(&QK_M2);
    Qiankui_ParaUpdate(&QK_M2,  Motor2.Target);

    QK_M3.U_k = Qiankui_Caculate(&QK_M3);
    Qiankui_ParaUpdate(&QK_M3,  Motor3.Target);

    QK_M4.U_k = Qiankui_Caculate(&QK_M4);
    Qiankui_ParaUpdate(&QK_M4,  Motor4.Target);

    QK_M5.U_k = Qiankui_Caculate(&QK_M5);
    Qiankui_ParaUpdate(&QK_M5,  Motor5.Target);

    QK_M6.U_k = Qiankui_Caculate(&QK_M6);
    Qiankui_ParaUpdate(&QK_M6,  Motor6.Target);
}
//////////
float limit_Float(float a, float up, float down)
{
    if (up < down ) return 0;
    if(a > up)a = up;
    if(a < down)a = down;
    return a;
}


float plant_Test(float u)
{
    float y1 = 0;
    y1 = u * 5;
    return y1;
}



///////////////////******PID模糊控制        End         **************************//////

