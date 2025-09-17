#ifndef MDL_CHASSIS_H
#define MDL_CHASSIS_H

#include "alg_pid.h"

#include "mdl_Console.h"
#include "mdl_IMU.h"
#include "Motor.h"
#include "robot_info.h"
#include "task_Detect.h"
#include "user_lib.h"
#include "dvc_Referee_system.h"
#include "alg_ADRC.h"
#include "dvc_Supercapacity.h"
#include "task_SoftwareTimer_Check.h"




typedef enum
{
    CHASSIS_RELAX = 0,          //安全模式
    CHASSIS_STOP,               //底盘停止
    CHASSIS_FOLLOW_GIMBAL,      //底盘跟随云台
    CHASSIS_SEPARATE_GIMBAL,    //底盘云台分离
    CHASSIS_SPIN,                //底盘旋转
    CHASSIS_ISLAND,
} ChassisCtrlMode_e;



typedef struct
{
	float wheel_perimeter; /* the perimeter(mm) of wheel */
	float wheeltrack;      /* wheel track distance(mm) */
	float wheelbase;       /* wheelbase distance(mm) */
	float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
	float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
} MechanicalStructure_t;

typedef struct
{
	MotorInfo_t*    motor_info;
	pid_t           pid;
	float            given_speed;
	int16_t         current_set;
} ChassisMotor_t;

typedef struct
{
    Console_t*    console;
    IMU_Data_t*   imu;                      //底盘陀螺仪指针
    CAN_Object_t* chassis_can;              //

    ChassisCtrlMode_e       ctrl_mode;             //底盘控制状态
    MechanicalStructure_t   structure;
    ChassisMotor_t          chassis_motor[4];
    pid_t                   chassis_follow_pid;        //follow angle PID.
    pid_t                  super_power_limit_pid;
	
    float vx;                      //
    float vy;                      //
    float vw;                      //
    float wheel_rpm[4];

    float Super_Power_ratio;         //超电充电比
    float Chassis_super_power;
    uint8_t	SuperPower_State;
	
    float gimbal_yaw_ecd_angle;
    float chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.
    float chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.
    float chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.
	
}	ChassisHandle_t;

typedef union
{
	uint8_t u[8];
	uint16_t f;
	uint8_t super_state;
	uint16_t f2;
	double ff;
}FormatTrans_t;

extern ChassisHandle_t chassis_handle;

void ChassisCtrlModeSwitch(void);
void ChassisSensorUpdata(void);

void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur);
void ChassisStopMode(void);
void ChassisFollowGimbalMode(void);
void ChassisSeparateGimbalMode(void);
void ChassisSpinMode(void);
void ChassisIslandMode(void);
void Chassis_ControlCalc(ChassisHandle_t* chassis_handle);
void Chassis_LimitPower(ChassisHandle_t* chassis_handle);

void ChassisAppConfig(void);

#endif //MDL_CHASSIS_H
