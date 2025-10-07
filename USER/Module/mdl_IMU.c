/* 包含头文件 ----------------------------------------------------------------*/
#include "alg_AHRS.h"
#include "mdl_IMU.h"
#include "dvc_BMI088.h"
#include "dvc_IST8310.h"
#include "cmsis_os.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define BMI088_BOARD_INSTALL_SPIN_MATRIX     \
    { 0.0f, -1.0f, 0.0f},                     \
    {1.0f, 0.0f, 0.0f},                     \
    { 0.0f, 0.0f, 1.0f}                      \

#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

/* 私有变量 ------------------------------------------------------------------*/
/* IMU数据结构 */
IMU_Data_t m_imu;
bmi088_real_data_t bmi088_data;
ist8310_real_data_t ist8310_data;
float k1 = -0.2501;   //matlb拟合的系数，用手扶住头不动，然后录yaw的零漂数据集  开遥控器
float k2 = -0.1573;
/* 校准使用数据 */
float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float gyro_offset[3];
float gyro_cali_offset[3];

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_offset[3];

float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float mag_offset[3];

/* 加速度计使用的低通滤波 */
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void IMU_CaliSlove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);
static void IMU_Accel_LowPassFilter(float accel[3]);

/* 函数体 --------------------------------------------------------------------*/
/**
  * @brief  根据安装方向旋转陀螺仪，加速度计和磁力计，并计算零漂
  */
static void IMU_CaliSlove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
  * @brief  accel 低通滤波
  */
static void IMU_Accel_LowPassFilter(float accel[3])
{
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + accel[2] * fliter_num[2];
}


float off_yaw,on_yaw;
void IMU_Init(void)
{
    while(BMI088_init())
    {
        osDelay(100);
    }
    while(IST8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
    IMU_CaliSlove(m_imu.gyro, m_imu.accel, m_imu.mag, &bmi088_data, &ist8310_data);

    AHRS_init(m_imu.quat, m_imu.accel, m_imu.mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = m_imu.accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = m_imu.accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = m_imu.accel[2];

    get_angle(m_imu.quat, &(m_imu.euler.yaw), &(m_imu.euler.pitch), &(m_imu.euler.roll));
	m_imu.i_attitude.i_pitch=m_imu.euler.roll * RAD_TO_ANGLE;//记录未初始化前roll
	m_imu.i_attitude.i_yaw=m_imu.euler.yaw * RAD_TO_ANGLE;//记录未初始化前yaw
 	m_imu.i_attitude.i_roll=m_imu.euler.pitch * RAD_TO_ANGLE;//记录未初始化前pitch
}

int i1=0;
float t1=0,t2=0;
void IMU_Update(const float period_time)
{
    BMI088_read(bmi088_data.gyro, bmi088_data.accel, &bmi088_data.temp);
    IMU_CaliSlove(m_imu.gyro, m_imu.accel, m_imu.mag, &bmi088_data, &ist8310_data);
    m_imu.temp = bmi088_data.temp;

    IMU_Accel_LowPassFilter(m_imu.accel);
    /*根据周期和陀螺仪数据、加入低通滤波更新四元数*/
    AHRS_update(m_imu.quat, period_time, m_imu.gyro, accel_fliter_3, m_imu.mag);
    /*使用四元数计算欧拉角*/
    get_angle(m_imu.quat, &m_imu.euler.yaw, &m_imu.euler.pitch, &m_imu.euler.roll);

    m_imu.attitude.pitch =  m_imu.euler.pitch * RAD_TO_ANGLE;                      //实际的pitch和roll反了
		m_imu.attitude.yaw = m_imu.euler.yaw * RAD_TO_ANGLE- k1 *BSP_GetTime_ms() /1000.0f;
//+ 0.2605 *BSP_GetTime_ms() /1000.0f ; //matlab数据拟合得出的参数
    m_imu.attitude.roll =   m_imu.euler.roll * RAD_TO_ANGLE;

}

IMU_Data_t* IMU_GetDataPointer(void)
{
    return &m_imu;
}

void IMU_GyroOffsetCalc(float gyro_offset[3], float gyro[3])
{
    if (gyro_offset == NULL || gyro == NULL)
    {
        return;
    }

    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
}

/*陀螺仪零漂计算*/
void IMU_CalibrateGyro(float cali_offset[3])
{
    IMU_GyroOffsetCalc(gyro_offset, m_imu.gyro);

    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
}

void IMU_SetGyroOffset(float cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}


