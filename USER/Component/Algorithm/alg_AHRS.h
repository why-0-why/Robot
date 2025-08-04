
#ifndef ALG_AHRS_H
#define ALG_AHRS_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"

//重新定义数据类型
typedef unsigned char bool_t;

//定义 NULL
#ifndef NULL
#define NULL 0
#endif

//定义PI 值
#ifndef PI
#define PI 3.14159265358979f
#endif

//定义 角度(度)转换到 弧度的比例
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//定义 弧度 转换到 角度的比例
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(float *high);
extern void AHRS_get_latitude(float *latitude);
extern float AHRS_invSqrt(float num);
extern float AHRS_sinf(float angle);
extern float AHRS_cosf(float angle);
extern float AHRS_tanf(float angle);
extern float AHRS_asinf(float sin);
extern float AHRS_acosf(float cos);
extern float AHRS_atan2f(float y, float x);

/*------------------------------------链接AHRS.lib文件中的函数---------------------------*/
/**
  * @brief          根据加速度的数据，磁力计的数据进行四元数初始化
  * @param[in]      需要初始化的四元数数组
  * @param[in]      用于初始化的加速度计,(x,y,z)不为空 单位 m/s2
  * @param[in]      用于初始化的磁力计计,(x,y,z)不为空 单位 uT
  * @retval         返回空
  */
extern void AHRS_init(float quat[4], const float accel[3], const float mag[3]);

/**
  * @brief          根据陀螺仪的数据，加速度的数据，磁力计的数据进行四元数更新
  * @param[in]      需要更新的四元数数组
  * @param[in]      更新定时时间，固定定时调用，例如1000Hz，传入的数据为0.001f,
  * @param[in]      用于更新的陀螺仪数据,数组顺序(x,y,z) 单位 rad
  * @param[in]      用于初始化的加速度数据,数组顺序(x,y,z) 单位 m/s2
  * @param[in]      用于初始化的磁力计数据,数组顺序(x,y,z) 单位 uT
  * @retval         1:更新成功, 0:更新失败
  */
extern bool_t AHRS_update(float quat[4], const float timing_time, const float gyro[3], const float accel[3], const float mag[3]);

/**
  * @brief          根据四元数大小计算对应的欧拉角偏航yaw
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的偏航角yaw 单位 rad
  */
extern float get_yaw(const float quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角俯仰角 pitch
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的俯仰角 pitch 单位 rad
  */
extern float get_pitch(const float quat[4]);
/**
  * @brief          根据四元数大小计算对应的欧拉角横滚角 roll
  * @param[in]      四元数数组，不为NULL
  * @retval         返回的横滚角 roll 单位 rad
  */
extern float get_roll(const float quat[4]);

/**
  * @brief          根据四元数大小计算对应的欧拉角yaw，pitch，roll
  * @param[in]      四元数数组，不为NULL
  * @param[in]      返回的偏航角yaw 单位 rad
  * @param[in]      返回的俯仰角pitch  单位 rad
  * @param[in]      返回的横滚角roll 单位 rad
  */
extern void get_angle(const float quat[4], float *yaw, float *pitch, float *roll);
/**
  * @brief          返回当前的重力加速度
  * @param[in]      空
  * @retval         返回重力加速度 单位 m/s2
  */
extern float get_carrier_gravity(void);


#endif


