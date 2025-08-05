// //
// // Created by Administrator on 25-7-30.
// //
//
#ifndef GIMBAL_H
#define GIMBAL_H
// #include "alg_pid.h"
// #include "drv_can.h"
// #include "mdl_IMU.h"
// #include "Motor.h"
//
// typedef struct
// {
//     float      now_yaw;
//     float      last_yaw;
//     float      now_pitch;
//     float      last_pitch;
//     float      YAW_RATIO;
//     float      PITCH_RATIO;
// } AimMessage_t;
//
// typedef enum
// {
//     GIMBAL_RELAX = 0,          //安全模式
//     GIMBAL_INIT,
//     GIMBAL_GYRO,
//     GIMBAL_RELATIVE,
//     GIMBAL_NORMAL,
//     GIMBAL_VISION,
//     GIMBAL_CLIPPED,
//     GIMBAL_ADD,
//     GIMBAL_MOVE,
// } GimbalCtrlMode_e;
//
// typedef enum
// {
//     RAW_VALUE_MODE = 0,
//     GYRO_MODE,
//     ENCONDE_MODE,
// } GimbalMotorMode_e;
// typedef struct
// {
//     float               relative_angle; /* 单位: degree */
//     float               gyro_angle;
//     float               palstance;      /* 单位: degree/s */
// } GimbalSensor_t;
//
// typedef struct
// {
//     pid_t               outer_pid;
//     pid_t               inter_pid;
//     float               angle_ref;
//     float               angle_fdb;
//     float               speed_ref;
//     float               speed_fdb;
// } Gimbal_PID_t;
//
// typedef struct
// {
//     MotorInfo_t*        motor_info;
//     uint16_t            offset_ecd;
//     float               ecd_ratio;
//     float               max_relative_angle;
//     float               min_relative_angle;
//
//     GimbalMotorMode_e   mode;
//     GimbalMotorMode_e   last_mode;
//     float               given_value;
//     float               last_given_value;
//
//     GimbalSensor_t      sensor;
//     Gimbal_PID_t        pid;
//
//     int16_t             current_set;
// } GimbalMotor_t;
//
// typedef struct
// {
//     Console_t*          console;
//     IMU_Data_t*         imu;              //
//     CAN_Object_t*       gimbal_can;    //
//
//     Comm_RobotInfo_t*   vision_tx_data;
//
//     Comm_VisionInfo_t*  vision_rx_data;
//
//     Gimbal_CMD_e        last_cmd;
//     GimbalCtrlMode_e    ctrl_mode;  //
//     GimbalCtrlMode_e    last_ctrl_mode;  //
//
//     GimbalMotor_t       yaw_motor;
//     GimbalMotor_t       pitch_motor;
//
//     float               infrared;
//     float               last_yaw_angle;
//     float               last_pitch_angle;
// } GimbalHandle_t;
#endif //GIMBAL_H
