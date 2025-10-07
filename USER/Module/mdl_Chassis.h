#ifndef MDL_CHASSIS_H
#define MDL_CHASSIS_H


//#include "mdl_Console.h"// 控制台函数和数据类型
//#include "mdl_IMU.h"// 数据结构体
// 数据结构体
//#include "alg_pid.h"
//#include "Motor.h"
#include "mdl_comm.h"
//





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
