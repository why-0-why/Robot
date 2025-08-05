#ifndef ALG_ADRC_H
#define ALG_ADRC_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define timeradio 1000.f
#define zepi_sign(x) ((x)>0?1:((x)<0?-1:1))
#define zepi_max(a /*Generic*/,b /*Generic*/) ((a)>(b)?(a):(b))
#define zepi_min(a /*Generic*/,b /*Generic*/) ((a)<(b)?(a):(b))
#define zepi_clamp(n /*Generic*/,a /*Generic*/,b /*Generic*/) (zepi_min(zepi_max(n,a),b))

/*----------------------------------结构体定义-----------------------------------------------------*/
typedef struct _ADRC_InitNLC_TypeDef
{
	float B;
	float Alpha_p;  // *非线性混合律积分项的指数
	float Alpha_n;  // *非线性混合律控制量的指数
	float Alpha_f;  // *非线性混合率预测项的指数
	float Beta_p;   // *非线性混合律积分项的权重
	float Beta_n;   // *非线性混合律控制量的权重
	float Beta_f;   // *非线性混合率预测项的权重
	float Delta;    // *饱和函数的临界值           5*Dtime =< Delta <=10*Dtime
	float MaxOutputClamp;  // *最大输出范围
}ADRC_InitNLC;

typedef struct _ADRC_InitESO_TypeDef
{
	float Alpha1;   // *观测误差的指数
	float Alpha2;   // *观测误差速度的指数
	float Alpha3;   // *观测误差加速度与系统扰动的指数
	float Omega;    // *系统观测带宽
	float k;
	float Delta;    // *饱和函数的临界值
}ADRC_InitESO;

typedef struct _ADRC_InitOther_TypeDef
{
	float DeadBand;
}ADRC_InitOther;

 typedef struct _ADRC_Init_TypeDef
 {
 	struct
 	{
 		float R;
 	}TD;             // 跟踪微分器

 	ADRC_InitNLC NLC;  // *非线性混合律
 	ADRC_InitESO ESO;  // 扩张状态观测器
 	ADRC_InitOther Other;
 }ADRC_Init;

typedef struct _ADRC_TypeDef
{
	float Target; //目标值
	float Lastnot0_target;  // 最后一次非零的目标值

	float Measure; //// *测量值

	struct _TD
	{
		float R1;  // 跟踪微分器当前跟踪值
		float R2;  // 跟踪微分器当前跟踪速度
		float V1;  // 跟踪微分器预估的跟踪速度
		float V2;  // 跟踪微分器预估的跟踪加速度

		float R;   // *跟踪微分器最大跟踪速度
	}TD;  // 跟踪微分器子结构

	struct _ESO
	{
		float Z1;       // 扩张状态观测器观测的当前状态值
		float Z2;       // 扩张状态观测器观测的当前变化速度
		float Z3;       // 扩张状态观测器观测的当前总系统扰动

		float Alpha1;   // *观测误差的指数
		float Alpha2;   // *观测误差速度的指数
		float Alpha3;   // *观测误差加速度与系统扰动的指数
		float Omega;    // *系统观测带宽
		float k;
		float Delta;    // *饱和函数的临界值
	}ESO;

	struct _NLC
	{
		float E0;  // 当前预测误差
		float E1;  // 当前预测误差的变化速度
		float E2;  // 当前预测误差的变化加速度
		float U0;  // 非线性混合律的原始控制量
		float U;   // 考虑扰动得出的最终控制量

		float B;        // *非线性混合律建模量与实际控制量的映射系数 逐渐增大b0
		float Alpha_p;  // *非线性混合律积分项的指数 i
		float Alpha_n;  // *非线性混合律控制量的指数 p
		float Alpha_f;  // *非线性混合率预测项的指数 d
		float Beta_p;   // *非线性混合律积分项的权重
		float Beta_n;   // *非线性混合律控制量的权重
		float Beta_f;   // *非线性混合率预测项的权重
		float Delta;    // *饱和函数的临界值 线性区间的宽度
		float MaxOutputClamp;  // *最大输出范围
	}NLC;

	struct _Time
	{
		uint32_t Time_p;  // 上次控制的时刻
		uint32_t Time_n;  // 本次控制的时刻
		float Dtime;      // 两次控制之间的时间
	}Time;

	float Output;         // 本次最终输出
	float Last_output;    // 最后一次的输出
	float MaxErr_recoder; // 最大误差记录

	struct _Other
	{
		float DeadBand;   // *控制死区范围
	}Other;

	void (*TD_cal)(struct _ADRC_TypeDef * adrc);  // 计算跟踪微分器部分
	void (*NLC_cal)(struct _ADRC_TypeDef * adrc); // 计算非线性混合律部分
	void (*ESO_cal)(struct _ADRC_TypeDef * adrc); // 计算扩张状态观测器部分
	void (*fullCal)(struct _ADRC_TypeDef * adrc); // 计算整个自抗扰控制器
	void (*getTimeStamp)(struct _ADRC_TypeDef * adrc); // 记录当前的时间戳
  void (*param_init)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init);            // 初始化参数
	void (*inputStatus)(struct _ADRC_TypeDef * adrc, float target, float feedback);  // 输入当前状态
	float (*getlastOutput)(struct _ADRC_TypeDef * adrc);                             // 获取最后一次的输出
	float (*getOutput)(struct _ADRC_TypeDef * adrc, float target, float feedback);   // 根据当前状态给出输出
	void (*reset_TD)(struct _ADRC_TypeDef * adrc, float new_R);                      // 重设跟踪微分器参数
	void (*reset_NLC)(struct _ADRC_TypeDef * adrc, ADRC_InitNLC new_NLC);            // 重设非线性混合律参数
	void (*reset_ESO)(struct _ADRC_TypeDef * adrc, ADRC_InitESO new_ESO);            // 重设扩张状态观测器参数
	void (*reset_Other)(struct _ADRC_TypeDef * adrc, float new_DeadBand);            // 重设其他参数
	void (*reset_allParam)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init);        // 重设所有参数
	void (*restart)(struct _ADRC_TypeDef * adrc);                                    // 重启自抗扰控制器
	void (*clean_recoder)(struct _ADRC_TypeDef * adrc);                              // 清除误差记录
}ADRC;

extern const ADRC_Init Default_ADRC;
extern const ADRC_Init Default_3508_speed;
extern const ADRC_Init Default_6020_speed;
extern const ADRC_Init Default_6020_pos;

/*---------------------------------函数声明------------------------------------------------*/
ADRC zepi_create_ADRC(void);
float zepi_fst(float x1, float x2, float r, float h);
static inline float zepi_fal(float x, float alpha, float delta);
static void _adrc_TD_cal(ADRC * adrc);  //使用zepi_fst;
static void _adrc_NLC_cal(ADRC * adrc);
static void _adrc_ESO_cal(ADRC * adrc);
static inline void _adrc_fullCal(ADRC * adrc);
static void _adrc_getTimeStamp(ADRC * adrc);
static void _adrc_param_init(ADRC * adrc, ADRC_Init adrc_init);
static void _adrc_inputStatus(ADRC * adrc, float target, float feedback);
static float _adrc_getLastOutput(ADRC * adrc);
float _adrc_getOutput(ADRC * adrc, float target, float feedback);
static void _adrc_reset_TD(ADRC * adrc, float R);
static void _adrc_reset_NLC(ADRC * adrc, ADRC_InitNLC new_NLC);
static void _adrc_reset_ESO(ADRC * adrc, ADRC_InitESO new_ESO);
static void _adrc_reset_Other(ADRC * adrc, float new_DeadBand);
void _adrc_reset_allParam(ADRC * adrc, ADRC_Init adrc_init);
static void _adrc_restart(ADRC * adrc);
static void _adrc_clean_recoder(ADRC * adrc);
ADRC zepi_create_ADRC(void);

#endif //ALG_ADRC_H
