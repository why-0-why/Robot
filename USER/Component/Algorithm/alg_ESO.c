//
// Created by Administrator on 25-8-7.
//
/* 包含头文件 ----------------------------------------------------------------*/
#include "alg_ESO.h"
#include "mdl_Gimbal.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
ESO_t ESO;
float e,z1,z2,z3,last_z1,last_z2,last_z3,u;
float deta1,deta2,deta3;
/* 扩展变量 ------------------------------------------------------------------*/
extern GimbalHandle_t gimbal_handle;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void ESO_init(void)
{
	e = 0,z1 = 0,z2 = 0,z3 = 0,last_z1 = 0,last_z2 = 0,last_z3 = 0,u = 0;
	ESO.y = 0;
	ESO.last_y = 0;
	ESO.h = 0.002;						//控制周期				s
	ESO.w = 5;								//扰动带宽
	ESO.K = 0.001;						//电压系数	
	ESO.Km = 0.741;						//转矩常数				N*M/A
	ESO.Ke = 13.33/60;				//转速常数				rps/V
	ESO.J = 0.3;							//转动惯量				kg*m^2
	ESO.R = 1.8;							//电机电阻				R
	ESO.b = 2;								//系统阶数
	deta1 = 3 * ESO.w;
	deta2 = 3 * pow(ESO.w,2);
	deta3 = 3 * pow(ESO.w,3);
}

int32_t ESO_calc(void *argc)
{
		//ESO补偿
		e = last_z1 - ESO.y;
		z1 = last_z1 + ESO.h * (last_z2 - deta1 * e);
		z2 = last_z2 + ESO.h * (last_z3 - deta2 * e + (ESO.K * ESO.Km)/(ESO.J * ESO.R) * u - (ESO.Km * ESO.Ke)/(ESO.J * ESO.R) * last_z2);
		z3 = last_z3 - ESO.h * deta3 * e;
		u = (gimbal_handle.pitch_motor.current_set - last_z3) / ESO.b;
		last_z1 = z1;
		last_z2 = z2;
		last_z3 = z3;
		return 0;
}

int sgn(float x)
{
	int y;
	if (x > 0)
		y = 1;
	else if (x < 0)
		y = -1;
	else
		y = 0;
	return y;
}

float fhan(float x1, float x2, float r, float h)
{
	float d, d0, y, a0, a, fst;
	d = r * h;
	d0 = d * h;
	y = x1 + h * x2;
	a0 = sqrt((d * d + 8 * r * ABS(y)));
	if (ABS(y) > d0)
		a = x2 + (a0 - d) / 2 * sgn(y);
	else
		a = x2 + y / h;
	if (ABS(a) > d)
		fst = -(r * sgn(a));
	else
		fst = -(r * a / d);
	return fst;
}

float TD(float v,float r,float h,float h0)		//v 跟踪目标	r 跟踪速度（越大跟踪速度越快）	h 采样周期	h0 滤波因子（一般0.001-0.1，越小滤波效果越好）
{
	float v1,v2,last_v1,last_v2;
	v1 = last_v1 + h * last_v2;
	v2 = last_v2 + h * fhan(last_v1 - v, last_v2, r, h0);
	last_v1 = v1;
	last_v2 = v2;
	return v2;
}

