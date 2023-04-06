#ifndef QIANKUI_H
#define QIANKUI_H



typedef struct
{
    float  U_k;            // U(k)
    float  U_k_1;          // U(k-1)
    float  U_k_2;          // U(k-2)
    float  U_k_3;          // U(k-3)
    float  Theta_k;        // ¦È(k)
    float  Theta_k_1;      // ¦È(k-1)
    float  Theta_k_2;      // ¦È(k-2)
    float  Theta_k_3;      // ¦È(k-3)

}Qiankui;

extern Qiankui QK_M1,QK_M2,QK_M3,QK_M4,QK_M5,QK_M6;

uint8_t Qiankui_init(Qiankui *Motr);
uint8_t Qiankui_ParaUpdate(Qiankui *Motr, float Theta_k);
float Qiankui_Caculate(Qiankui *Motr);


#endif
