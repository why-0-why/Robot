/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-11-11 18:36:13
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-11-19 19:25:57
 * @FilePath: \MDK-ARMd:\RM_work\Hero_GroupWork\Hero2025_V1.3\Components\drvices\DM_Motor\bsp_dm4310.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef DVC_DM4310_H
#define DVC_DM4310_H


/* 包含头文件 ----------------------------------------------------------------*/
#include "drv_can.h"
#include "arm_math.h" // 引入数学库

/* 类型定义 ------------------------------------------------------------------*/
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
#define Master_ID 0x11          //4310反馈帧id


// 电动机信息结构体
typedef __packed struct
{
 int id;     //控制器的 ID， 取 CAN_ID 的低 8 位
 int state;  //故障类型
 int p_int;
 int v_int;
 int t_int;
 int kp_int;
 int kd_int;
 float pos;
 float pos_deg;   // 位置 (角度制)
  float vel;       // 速度 (弧度/秒)
  float vel_rpm;   // 速度 (转速/分钟)
 float toq;
 float Kp;
 float Kd;
 float Tmos;
 float Tcoil;
 float offset_pos; // 位置偏移量
 float max_relative_angle; // 最大相对角度
 float min_relative_angle; // 最小相对角度
 float dir;        // 电机安装方向
}Motor_Inf;

// 电动机MIT数据结构体
typedef __packed struct
{
 float p_int;
 float v_int;
 float kp_int;
 float kd_int;
    float t_int;
}Motor_MIT_Data_t;


/* 宏定义 --------------------------------------------------------------------*/
#define CAN_ID    0x01         //4310控制帧id   0x000MIT偏移 0x100位置速度偏移 0x200速度偏移
#define Master_ID 0x11          //4310反馈帧id

#define PITCH_DMDIR     (-1.0f) //pitch电机安装方向
#define YAW_DMDIR       (-1.0f)  //yaw电机安装方向
#define PITCH_DMRATIO       (1.0f)  //pitch大疆减速比
#define YAW_DMRATIO         (1.0f)  //yaw减速比
/* 扩展变量 ------------------------------------------------------------------*/
extern Motor_Inf chassis_dmmotor[4];
extern Motor_Inf gimbal_dmmotor_yaw;
extern Motor_Inf gimbal_dmmotor_pitch;
extern Motor_Inf friction_wheel_dmmotor[2];
extern Motor_Inf pluck_dmmotor;
extern Motor_Inf cover_dmmotor;
extern Motor_Inf mtr;
/* 函数声明 ------------------------------------------------------------------*/
// 将浮点数转换为无符号整数
int float_to_uint(float x, float x_min, float x_max, int bits);

// 将无符号整数转换为浮点数
float uint_to_float(int x_int, float x_min, float x_max, int bits);

// 计算电机位相对位置
float DMMotor_RelativePosition(float rad, float center_offset);

// 计算电机绝对位置
float DMMotor_CalculateAbsolutePosition(float tmp, float center_offset);

// 解析DM4310数据
void DMMotor_DataParse(Motor_Inf* dmmotor,uint8_t data[]);

void DMMotor_Init(Motor_Inf* ptr);

// 控制电机函数,MIT模式
void ctrl_motor(CAN_Object_t *obj,uint16_t id, Motor_MIT_Data_t* _dm43_mit_t); //MIT模式

// 控制电机的第二个函数，位置模式
void ctrl_motor2(CAN_Object_t *obj,uint16_t id, float _pos, float _vel) ;//位置模式

// 控制电机的第三个函数, 速度模式
void ctrl_motor3(CAN_HandleTypeDef* hcan,uint16_t id, float _vel);

// 启动电机
void start_motor(CAN_Object_t *obj,uint16_t id);

// 停止电机
void lock_motor(CAN_Object_t *obj,uint16_t id);

// CAN滤波器初始化函数
void can_filter_init(void);

Motor_MIT_Data_t* Get_MIT_Pointer();


#endif /* DVC_DM4310_H */
