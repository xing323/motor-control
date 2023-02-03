#include "main.h"
#include "Inv_Kine.h"

//左右腿的逆运动学机构体
Inv_Kine Left_InvKine,Right_InvKine;


void Inv_Kine_Caculator(float alpha,float beta,float gamma,Inv_Kine* A)
{
	//基本参数，输入输出都为弧度，调用时需注意
	float x0=0;
	float y0=-0.165;
	float z0=0.130;
	float c=0.135;
	float r=0.05;    //原来r的一半
	float h=0.323;
	float phi1=atan((c-1.73205*r)/h);
	
	float sin_Alpha= sin(alpha);
	float cos_Alpha= cos(alpha);
	float sin_Beta= sin(beta);
	float cos_Beta= cos(beta);
	float sin_Gamma= sin(gamma);
	float cos_Gamma= cos(gamma);
	
	float para1 = 1.73205*r-c-y0;
	float para2 = c+y0-2*r;

	float a1=y0+(r-x0)*cos_Beta*sin_Gamma+para1*(cos_Alpha*cos_Gamma+sin_Alpha*sin_Beta*sin_Gamma)-(h-z0)*(sin_Alpha*cos_Gamma-cos_Alpha*sin_Beta*sin_Gamma);
  float b1=z0-(r-x0)*sin_Beta+para1*sin_Alpha*cos_Beta+(h-z0)*cos_Alpha*cos_Beta;
  float a2=x0-x0*cos_Beta*cos_Gamma+para2*(cos_Alpha*sin_Gamma-sin_Alpha*sin_Beta*cos_Gamma)+(h-z0)*(sin_Alpha*sin_Gamma+cos_Alpha*sin_Beta*cos_Gamma);
  float b2=z0+x0*sin_Beta-para2*sin_Alpha*cos_Beta+(h-z0)*cos_Alpha*cos_Beta;
  float a3=y0+(-r-x0)*cos_Beta*sin_Gamma+para1*(cos_Alpha*cos_Gamma+sin_Alpha*sin_Beta*sin_Gamma)-(h-z0)*(sin_Alpha*cos_Gamma-cos_Alpha*sin_Beta*sin_Gamma);
  float b3=z0+(r+x0)*sin_Beta+para1*sin_Alpha*cos_Beta+(h-z0)*cos_Alpha*cos_Beta;
	
	float M1=-a1/b1;
  float M2=a2/b2;
  float M3=-a3/b3;
	
	A->theta1=atan(M1)-phi1;
  A->theta2=atan(M2);
  A->theta3=atan(M3)-phi1;
	
}




































