#include "main.h"
#include "qiankui.h"


/***********************************************************
*@fuction	:Qiankui_init
*@brief		:前馈结构体参数初始化为0
*@param		:Qiankui* Motr
*@return	:uint8_t
*@author	:xing
*@date		:2023-04-01
***********************************************************/

uint8_t Qiankui_init(Qiankui *Motr)
{
    Motr->U_k       = 0;
    Motr->U_k_1     = 0;
    Motr->U_k_2     = 0;
    Motr->U_k_3     = 0;
    Motr->Theta_k   = 0;
    Motr->Theta_k_1 = 0;
    Motr->Theta_k_2 = 0;
    Motr->Theta_k_3 = 0;
    return 1;
}

/***********************************************************
*@fuction	:Qiankui_ParaUpdate
*@brief		:参数更新接口
*@param		:Qiankui *Motr, float U_k, float Theta_k
*@return	:void
*@author	:xing
*@date		:2023-04-01
***********************************************************/

uint8_t Qiankui_ParaUpdate(Qiankui *Motr, float Theta_k)
{
    //    Motr->U_k_3     = Motr->U_k_2;
    //    Motr->U_k_2     = Motr->U_k_1;
    Motr->U_k_1     = Motr->U_k;
    Motr->Theta_k_3 = Motr->Theta_k_2;
    Motr->Theta_k_2 = Motr->Theta_k_1;
    Motr->Theta_k_1 = Motr->Theta_k;
    Motr->Theta_k   = Theta_k;
    return 1;
}


float Qiankui_Caculate(Qiankui *Motr)
{
    //传递函数表达式
    //U(k) = K1*θ(k)+ K2*θ(k-1)+ K3*θ(k-2)+ K4*θ(k-3)+ K6*U(k-1)+ K7*U(k-2)+ K8*U(n-3)
    //	float K_para[2][4] = {0.004953,0.01486,0.01486,0.004953,0,-0.4687,0.9109,0.5578};    //0.1
    //	//float K_para[2][4] = {0.0000175,0.00005275,0.00005275,0.00001758,0,-2.157,-1.529,0.372};   //0.001
    //	float U_k = K_para[0][0]*Motr->Theta_k + K_para[0][1]*Motr->Theta_k_1+ K_para[0][2]*Motr->Theta_k_2+ K_para[0][3]*Motr->Theta_k_3
    //		                                     + K_para[1][1]*Motr->U_k_1    + K_para[1][2]*Motr->U_k_2    + K_para[1][3]*Motr->U_k_3;
    float U_k = 2*1520 * 0.333 * (Motr->Theta_k - Motr->Theta_k_3) + 2*22.4 * 0.5 * (Motr->Theta_k - 2 * Motr->Theta_k_2 + Motr->Theta_k_3);
	  U_k = U_k*0.2 + 0.8*Motr->U_k;
    return U_k;
}
