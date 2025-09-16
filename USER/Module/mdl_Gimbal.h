#ifndef MDL_GIMBAL_H
#define MDL_GIMBAL_H

#include "mdl_Console.h"
#include "mdl_IMU.h"
#include "Motor.h"
#include "task_Detect.h"
#include "alg_AHRS.h"
#include "alg_ESO.h"
#include "alg_ramp.h"


#define ABS(x) (((x) > 0) ? (x) : (-(x)))

#define BACK_CENTER_TIME 2500
#define KALMAN_FILTER_ANGLE 0
#define KALMAN_FILTER_SPEED 1
#define KALMAN_FILTER_ACCEL 2

#define vision_comps_yaw 0
#define vision_comps_pitch 1

extern ramp_v0_t yaw_ramp;
extern ramp_v0_t pitch_ramp;

typedef enum
{
    GIMBAL_RELAX = 0,
    GIMBAL_INIT,
    GIMBAL_GYRO,
    GIMBAL_RELATIVE,
    GIMBAL_NORMAL,
    GIMBAL_VISION,
    GIMBAL_ZARO,
} GimbalCtrlMode_e;



#pragma pack(push, 1)
typedef struct
{
    uint8_t data_head;          // 默认 0xAA
    uint8_t pitch[4];
    uint8_t yaw[4];
    uint8_t palstance[4];
    uint8_t can_shoot[2];
    uint8_t enemy_move_state[2];
    uint8_t state[2];
    uint8_t data_tail;
} Comm_VisionInfo_t;

typedef struct
{
    uint8_t data_head;
    uint8_t enemy_color;
    float   yaw_relative_angle;
    float   pitch_relative_angle;
    uint8_t robot_level;
} Comm_RobotInfo_t;
#pragma pack(pop)

typedef enum
{
    RAW_VALUE_MODE = 0,
    GYRO_MODE,
    ENCONDE_MODE,
} GimbalMotorMode_e;

typedef struct
{
    float            relative_angle; /* unit: degree */
    float            gyro_angle;
    float            palstance;      /* uint: degree/s */
} GimbalSensor_t;

typedef struct
{
    pid_t           outer_pid;
    pid_t           inter_pid;
    float            angle_ref;
    float            angle_fdb;
    float            speed_ref;
    float            speed_fdb;
} Gimbal_PID_t;

typedef struct
{
    MotorInfo_t*    motor_info;
    uint16_t        offset_ecd;
    float            ecd_ratio;
    float            max_relative_angle;
    float            min_relative_angle;

    GimbalMotorMode_e mode;
    GimbalMotorMode_e last_mode;
    float            given_value;         //输入期望值
    float            last_given_value;    //上一次输入期望值

    GimbalSensor_t  sensor;             //传感器数据（包含编码器相对角度、陀螺仪角度、角速度）
    Gimbal_PID_t    pid;

    int16_t         current_set;
} GimbalMotor_t;

typedef struct
{
    Console_t*      console;
    IMU_Data_t*     imu;              //
    CAN_Object_t*   gimbal_can;    //

    Comm_RobotInfo_t*    vision_tx_data;
    Comm_VisionInfo_t*   vision_rx_data;

    Gimbal_CMD_e last_cmd;
    GimbalCtrlMode_e ctrl_mode;  //
    GimbalCtrlMode_e last_ctrl_mode;  //
    GimbalCtrlMode_e last_moment_ctrl_mode;  //

    GimbalMotor_t   yaw_motor;
    GimbalMotor_t   pitch_motor;

    float       infrared;
} GimbalHandle_t;



typedef enum
{
    AIM_NO = 0,
    AIM_RIGHT,
} VisionAim_e;

typedef enum
{
    RELAX = 0,
    FOLLOW,
    FIRST_AIMING,
    COMPLETE_AIMING,
} AimContorl_e;

typedef struct
{
    float    pitch;
    float    yaw;
    float    palstance;
    float    accelerated_speed;
    float    last_pitch;
    float    last_yaw;
    float    last_palstance;
    float    last_accelerated_speed;
    float    rpm;
    int      can_shoot;
    int      switch_armor;
    int      last_switch_armor;
    int      enemy_move_state;
    int      state;     //VisionState_e  0x01->Comm_Successed
    VisionAim_e yaw_success;
    VisionAim_e pitch_success;
} VisionDatabase_t;

typedef struct
{
    AimContorl_e aim_mode;
    uint32_t pid_SAtime;
    uint32_t aiming_time;
    uint32_t stay_time;
    uint32_t systeam_time;
    float tol_angle ;                      //人为规定消抖角度范围(-tol_angle,tol_angle)
    uint32_t tol_time ;                    //the aiming-done continueous time that user set 人为规定
    uint32_t Sstart_time,Astart_time;      //消抖持续时间,单次自瞄持续时间
    int first_aim ;                        //初入消抖范围标志
    int aim_flag ;                         //新的单次自瞄标志
    float Ap_parm;
    float Sp_parm;
    int enable_paramSA;
} AutoAim_t;

typedef struct
{
    float angle[2];  //目标角度
    float gv[2];     //角速度
    float acc[2];    //角加速度
    float out;
    uint32_t times;
} AimCalcData_t;

typedef struct
{
    float yaw_now;
    float yaw_last;
    float pitch_now;
    float pitch_last;
    float distance;
} AutoAimError_t;

typedef struct
{
    float yaw_lpc;   //移动预测系数   38；35；30
    float pitch_lpc;
    float auto_err_yaw;   //角度误差
    float auto_err_pitch;
    float kalman_filter_delay;   //预测开启延时
    float kalman_filter_yaw_speed_min;
    float kalman_filter_pitch_speed_min;
    float kalman_filter_yaw_speed_max;
    float kalman_filter_pitch_speed_max;
    float kalman_filter_yaw_amplification;   //预测增幅    130；125；115；135
    float kalman_filter_pitch_amplification;
} AutoAimCoefficient_t;

typedef struct
{
    uint32_t delay_cnt;
    uint32_t freq;
    uint32_t last_time;
    float last_position;
    float speed;
    float last_speed;
    float processed_speed;
} Speed_Calc_Data_t;

typedef struct
{
    uint32_t auto_aim_shoot_l;
    uint32_t auto_aim_shoot_r;
    uint32_t auto_aim_shoot_stop;
} Aim_Shootdelay_t;

void GimbalSensorUpdata(void);
void GimbalCtrlModeSwitch(void);
void GimbalRampUpdate(void);
void GimbalRampInit(void);
void GimbalMotorSendCurrent(int16_t yaw, int16_t pitch);
void GimbalMotorSendPos(float _pos, float _vel);

void GimbalInitMode(void);
void GimbalGyroAngleMode(void);
void GimbalRelativeAngleMode(void);
void GimbalNormalMode(void);
void GimbalVisionAimMode(void);
void GimbalClippedMode(void);

void VisionDataUpdate(void);
void pid_paramSA(AutoAim_t* Aim);
void Aim_contorl(AutoAim_t* Aim);
VisionDatabase_t* VisionData_Pointer(void);
float AngleTransform(float target_angle, float gyro_angle);
void AimContorlModeSwitch(AutoAim_t* Aim);

float Gimbal_PID_Calc(Gimbal_PID_t* pid, float angle_ref, float angle_fdb, float speed_fdb);
void Gimbal_PID_Clear(Gimbal_PID_t* pid);
void GimbalMotorChangeProtect(GimbalMotor_t* motor);
void GimbalMotorControl(GimbalMotor_t* motor);

void AimReset(AimCalcData_t* data, float angle_raf);
void AimCalc(AimCalcData_t* data, float angle, float time, float angle_raf);

float Target_Speed_Calc(Speed_Calc_Data_t *Speed, uint32_t time, float position);
float Auto_Aim_RAMP(float final, float now, float ramp);

#endif /* MDL_GIMBAL_H */