/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-11-11 18:36:13
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-11-21 21:41:35
 * @FilePath: \MDK-ARMd:\RM_work\Hero_GroupWork\Hero2025_V1.3\Components\drvices\Motor\motor.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef MOTOR_H
#define MOTOR_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "drv_can.h"
#include "../Device/dvc_DJmotor.h"
#include "../Device/dvc_DMmotor.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "alg_pid.h"
/* 类型定义 ------------------------------------------------------------------*/


typedef enum
{
   DM_MOTOR = 0,        //达妙电机
   DJ_MOTOR = 1,        //大疆电机
}MotorType_t;

typedef struct
{
    MotorType_t motor_type;           //电机类型

    float relative_angle;          //360角度制
    float last_relative_angle;     //上一次的相对角度
    float total_angle;           //累计角度  摩擦轮用

    int16_t speed_rpm;
    float given_value;     //给定值，不同电机单位不同，可为位置值或速度值或电流值、电压值
    uint8_t temperature;           //电机温度

    float offset_value;   //复位值，单位可为编码值或弧度值

    int16_t ratio;           //减速比

    float min_relative_angle; //最小相对角度
    float max_relative_angle; //最大相对角度

} MotorInfo_t;               //电机总接口信息



/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
extern MotorInfo_t chassis_motor[4];
extern MotorInfo_t gimbal_motor_yaw;
extern MotorInfo_t gimbal_motor_pitch;
extern MotorInfo_t friction_wheel_motor[2];
extern MotorInfo_t pluck_motor;
extern MotorInfo_t cover_motor;

/* 函数声明 ------------------------------------------------------------------*/

void Motor_Update(MotorInfo_t *motor,MotorType_t type,DJMotorInfo_t* djmotor,Motor_Inf* dmmotor);
void Motor_DataParse(MotorInfo_t *motor,MotorType_t type,uint8_t *data,DJMotorInfo_t* djmotor,Motor_Inf* dmmotor);
MotorInfo_t* ChassisMotor_Pointer(uint8_t i);
MotorInfo_t* GimbalMotorYaw_Pointer(void);
MotorInfo_t* GimbalMotorPitch_Pointer(void);
MotorInfo_t* FrictionWheelMotor_1_Pointer(void);
MotorInfo_t* FrictionWheelMotor_2_Pointer(void);
MotorInfo_t* PluckMotor_Pointer(void);
MotorInfo_t* MagazineMotor_Pointer(void);


#endif  // MOTOR_H

