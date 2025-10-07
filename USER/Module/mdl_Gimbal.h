#ifndef MDL_GIMBAL_H
#define MDL_GIMBAL_H

#include "mdl_Console.h"// 控制台数据类型
#include "mdl_IMU.h" //IMU数据类型
#include "Motor.h"// pid和motor的数据类型
#include "mdl_comm.h"
#include "alg_pid.h"



#define ABS(x) (((x) > 0) ? (x) : (-(x)))

#define BACK_CENTER_TIME 2500
#define KALMAN_FILTER_ANGLE 0
#define KALMAN_FILTER_SPEED 1
#define KALMAN_FILTER_ACCEL 2

#define vision_comps_yaw 0
#define vision_comps_pitch 1

extern ramp_v0_t yaw_ramp;
extern ramp_v0_t pitch_ramp;


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