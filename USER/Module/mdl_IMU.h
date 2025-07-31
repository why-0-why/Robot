//
// Created by Administrator on 25-7-31.
//

#ifndef MDL_IMU_H
#define MDL_IMU_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "dvc_BMI088.h"
#include "dvc_IST8310.h"
#include "alg_AHRS.h"
#include "cmsis_os.h"
#include "drv_timer.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    float gyro[3];       /* 角速度 */
    float accel[3];      /* 加速度 */
    float mag[3];        /* 磁力计 */
    float temp;          /* 温度 */
    float temp_tar;      /* 目标温度 */
    float quat[4];       /* 四元数 */

    /* 欧拉角 单位 rad */
    struct
    {
        float pitch;
        float roll;
        float yaw;
    } euler;

    /* 姿态角 */
    struct
    {
        float pitch;
        float roll;
        float yaw;
    } attitude;
    struct
    {
        float i_pitch;
        float i_roll;
        float i_yaw;
    } i_attitude;
} IMU_Data_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/

void IMU_Init(void);
void IMU_Update(const float period_time);
IMU_Data_t* IMU_GetDataPointer(void);
void IMU_CalibrateGyro(float cali_offset[3]);
void IMU_SetGyroOffset(float cali_offset[3]);


#endif //MDL_IMU_H
