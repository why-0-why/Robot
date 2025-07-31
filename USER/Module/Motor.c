/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-11-11 18:36:13
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-11-21 20:21:20
 * @FilePath: \MDK-ARMd:\RM_work\Hero_GroupWork\Hero2025_V1.3\Components\drvices\Motor\motor.c
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
/* 包含头文件 ----------------------------------------------------------------*/
#include "Motor.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
MotorInfo_t chassis_motor[4];
MotorInfo_t gimbal_motor_yaw;
MotorInfo_t gimbal_motor_pitch;
MotorInfo_t friction_wheel_motor[2];
MotorInfo_t pluck_motor;
MotorInfo_t cover_motor;

extern   Motor_MIT_Data_t MIT;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/

//电机总接口信息更新，回调后及时调用该函数
void Motor_Update(MotorInfo_t *motor,MotorType_t type,DJMotorInfo_t* djmotor,Motor_Inf* dmmotor)
{
    if(type == DJ_MOTOR)
    {
        motor->last_relative_angle = motor->relative_angle;  //更新上一次的相对角度
        motor->relative_angle = djmotor->ecd_ratio* (float)DJMotor_RelativePosition(djmotor->ecd, djmotor->offset_ecd);
        motor->speed_rpm = djmotor->speed_rpm;
        motor->temperature = djmotor->temperature;
        motor->total_angle = djmotor->total_ecd;//摩擦轮用

    }
    else if(type == DM_MOTOR)
    {
        motor->last_relative_angle = motor->relative_angle;  //更新上一次的相对角度
        motor->relative_angle =  dmmotor->dir*(float)DMMotor_RelativePosition(dmmotor->pos, dmmotor->offset_pos)*180.0f/PI;
        motor->speed_rpm = dmmotor->vel_rpm;
        motor->temperature = dmmotor->Tmos;
        motor->max_relative_angle = dmmotor->max_relative_angle;
        motor->min_relative_angle = dmmotor->min_relative_angle;
        motor->offset_value = dmmotor->offset_pos;


    }


}

/***************************************************************
 * 函数名称: Motor_DataParse
 * 功能描述: 解析电机数据，根据电机类型调用相应的数据解析函数，
 *           并更新电机信息。
 * 输入参数:
 *   MotorInfo_t *motor: 电机信息结构体指针
 *   MotorType_t type: 电机类型
 *   uint8_t *data: 数据指针
 *   DJMotorInfo_t* djmotor: DJ电机信息结构体指针
 *   Motor_Inf* dmmotor: DM电机信息结构体指针
 * 返回值: 无
 ***************************************************************/
void Motor_DataParse(MotorInfo_t *motor,MotorType_t type,uint8_t *data,DJMotorInfo_t* djmotor,Motor_Inf* dmmotor)
{
    motor->motor_type = type;
    if(motor->motor_type == DJ_MOTOR)
        {
        DJMotor_DataParse(djmotor,data);
        Motor_Update(motor,type,djmotor,NULL);
        }
else if (motor->motor_type == DM_MOTOR)
        {
        DMMotor_DataParse(dmmotor,data);
        Motor_Update(motor,type,NULL,dmmotor);
        }
}

MotorInfo_t* ChassisMotor_Pointer(uint8_t i)
{
    return &chassis_motor[i];
}

MotorInfo_t* GimbalMotorYaw_Pointer(void)
{
    return &gimbal_motor_yaw;
}

MotorInfo_t* GimbalMotorPitch_Pointer(void)
{
    return &gimbal_motor_pitch;
}

MotorInfo_t* FrictionWheelMotor_1_Pointer(void)
{
    return &friction_wheel_motor[0];
}

MotorInfo_t* FrictionWheelMotor_2_Pointer(void)
{
    return &friction_wheel_motor[1];
}

MotorInfo_t* PluckMotor_Pointer(void)
{
    return &pluck_motor;
}

MotorInfo_t* MagazineMotor_Pointer(void)
{
    return &cover_motor;
}

