// /* 包含头文件 ----------------------------------------------------------------*/
// #include "task_Chassis_ctrl.h"
// #include "infantry_def.h"
// #include "cmsis_os.h"
// #include "chassis_function.h"
// #include "user_protocol.h"
// #include "dvc_Supercapacity.h"
// #include "dvc_Referee_system.h"
//
// /* 私有类型定义 --------------------------------------------------------------*/
// static uint16_t super_power_state = 0;
// /* 私有宏定义 ----------------------------------------------------------------*/
//
// /* 私有变量 ------------------------------------------------------------------*/
// float tff=0;//前馈扭矩
// /* 任务 */
// osThreadId ChassisTaskHandle;
// #if INCLUDE_uxTaskGetStackHighWaterMark
// static uint32_t chassis_task_stack = 0;
// #endif
//
// /* 扩展变量 ------------------------------------------------------------------*/
// extern ChassisHandle_t chassis_handle;
// extern ADRC adrc_3508[4];
// //extern ShootHandle_t shoot_handle;
// /* 私有函数原形 --------------------------------------------------------------*/
// static void ChassisCtrlModeSwitch(void);
// static void ChassisSensorUpdata(void);
//
// static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur);
// static void ChassisStopMode(void);
// static void ChassisFollowGimbalMode(void);
// static void ChassisSeparateGimbalMode(void);
// static void ChassisSpinMode(void);
// static void ChassisIslandMode(void);
//
// /* 函数体 --------------------------------------------------------------------*/
// void ChassisTask(void const *argument)
// {
//     for(;;)
//     {
//         ChassisSensorUpdata();
//         ChassisCtrlModeSwitch();
//         ChassisSuperCapTest();
//         switch (chassis_handle.ctrl_mode)
//         {
//             case CHASSIS_STOP:
//             {
//                 ChassisStopMode();
//             }break;
//             case CHASSIS_FOLLOW_GIMBAL:
//             {
//                 ChassisFollowGimbalMode();
//             }break;
//             case CHASSIS_SEPARATE_GIMBAL:
//             {
//                 ChassisSeparateGimbalMode();
//             }break;
//             case CHASSIS_SPIN:
//             {
//                 ChassisSpinMode();
//             }break;
//             case CHASSIS_ISLAND:
//             {
//                 ChassisIslandMode();
//             }break;
//             default:
//                 break;
//         }
//
//         Chassis_ControlCalc(&chassis_handle);
//
//         for (uint8_t i = 0; i < 4; i++)
//         {
//             chassis_handle.chassis_motor[i].given_speed = chassis_handle.wheel_rpm[i];
//
//             chassis_handle.chassis_motor[i].current_set = pid_calc(&chassis_handle.chassis_motor[i].pid,
//                                                                    chassis_handle.chassis_motor[i].motor_info->speed_rpm,
//                                                                    chassis_handle.chassis_motor[i].given_speed);
//
//         }
//
//       Chassis_LimitPower(&chassis_handle);
// //        Chassis_LimitCap(&chassis_handle);
//
//         if(chassis_handle.ctrl_mode == CHASSIS_RELAX)
//         {
//             for (uint8_t i = 0; i < 4; i++)
//             {
//                 chassis_handle.chassis_motor[i].current_set = 0;
//             }
//         }
//
//         SuperCap_SendMessage(&can1_obj);
//
//         ChassisMotorSendCurrent(chassis_handle.chassis_motor[0].current_set,
//                                 chassis_handle.chassis_motor[1].current_set,
//                                 chassis_handle.chassis_motor[2].current_set,
//                                 chassis_handle.chassis_motor[3].current_set);
//
//         osDelay(CHASSIS_CTRL_TASK_PERIOD);
// #if INCLUDE_uxTaskGetStackHighWaterMark                           //freertos标准格式
//         chassis_task_stack = uxTaskGetStackHighWaterMark(NULL);
// #endif
//
//     }
// }
//
// void ChassisTaskInit(void)
// {
//     osThreadDef(chassis_task, ChassisTask, osPriorityNormal, 0, 256);
//     ChassisTaskHandle = osThreadCreate(osThread(chassis_task), NULL);
// }
//
//
//
// static void ChassisSensorUpdata(void)
// {
//     Comm_GimbalInfo_t* gimbal_info = GimbalInfo_Pointer();
//     chassis_handle.gimbal_yaw_ecd_angle = gimbal_info->yaw_ecd_angle;
//     chassis_handle.chassis_pitch = chassis_handle.imu->attitude.pitch - gimbal_info->pitch_gyro_angle;
//     chassis_handle.chassis_roll = chassis_handle.imu->attitude.roll;
//     chassis_handle.chassis_yaw = chassis_handle.imu->attitude.yaw - gimbal_info->yaw_gyro_angle;
// }
//
// static void ChassisCtrlModeSwitch(void)
// {
//     if (chassis_handle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
//     {
//         chassis_handle.ctrl_mode = CHASSIS_RELAX;
//     }
//     else if (chassis_handle.console->chassis_cmd == CHASSIS_STOP_CMD)
//     {
//         chassis_handle.ctrl_mode = CHASSIS_STOP;
//     }
//     else if (chassis_handle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
//     {
//         chassis_handle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
//     }
//     else if (chassis_handle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
//     {
//         chassis_handle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
//     }
//     else if (chassis_handle.console->chassis_cmd == CHASSIS_SPIN_CMD)
//     {
//         chassis_handle.ctrl_mode = CHASSIS_SPIN;
//     }
//     else if (chassis_handle.console->chassis_cmd == CHASSIS_ISLAND_CMD)
//     {
//         chassis_handle.ctrl_mode = CHASSIS_ISLAND;
//     }
// }
//
// static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur)
// {
//     Motor_SendMessage(chassis_handle.chassis_can, MOTOR_1TO4_CONTROL_STD_ID, motor1_cur, motor2_cur, motor3_cur, motor4_cur);
// }
//
//
//
// static void ChassisStopMode(void)
// {
//     chassis_handle.vx = 0;
//     chassis_handle.vy = 0;
//     chassis_handle.vw = 0;
// }
//
// static void ChassisFollowGimbalMode(void)
// {
//     chassis_handle.vx = chassis_handle.console->chassis.vx;
//     chassis_handle.vy = chassis_handle.console->chassis.vy;
//     chassis_handle.vw = pid_calc(&chassis_handle.chassis_follow_pid,
//                                  -chassis_handle.gimbal_yaw_ecd_angle,
//                                  0);
// }
//
// static void ChassisSeparateGimbalMode(void)
// {
//     chassis_handle.vx = chassis_handle.console->chassis.vx;
//     chassis_handle.vw = -chassis_handle.console->chassis.vw;
// }
//
// static void ChassisSpinMode(void)
// {
// 	if(chassis_handle.console->supercap_cmd == SUPERCAP_OFF_CMD)
// 	{
//     chassis_handle.vx = chassis_handle.console->chassis.vx;
//     chassis_handle.vy = chassis_handle.console->chassis.vy;
//     chassis_handle.vw = 110;
// 		if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=55)
// 		{
// 			chassis_handle.vw = 100;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=60)
// 		{
// 			chassis_handle.vw = 110;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=65)
// 		{
// 			chassis_handle.vw = 120;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=70)
// 		{
// 			chassis_handle.vw = 130;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=75)
// 		{
// 			chassis_handle.vw = 140;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=80)
// 		{
// 			chassis_handle.vw = 150;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=85)
// 		{
// 			chassis_handle.vw = 160;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=90)
// 		{
// 			chassis_handle.vw = 170;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=100)
// 		{
// 			chassis_handle.vw = 190;
// 		}
// 		else if(RefereeSystem_RobotState_Pointer()->chassis_power_limit<=120)
// 		{
// 			chassis_handle.vw = 230;
// 		}
// 		//100 55w
// 		//110 60w
// 		//120 65w
// 		//130 70w
// 		//140 75w
// 		//150 80w
// 		//160 85w
// 		//170 90w
// 		//190 100w
// 		//230 120w
// 	}
// 	if(chassis_handle.console->supercap_cmd == SUPERCAP_ON_CMD)
// 	{
// 		chassis_handle.vx = chassis_handle.console->chassis.vx;
//     chassis_handle.vy = chassis_handle.console->chassis.vy;
//     chassis_handle.vw = 500;
// 	}
// }
//
// static void ChassisIslandMode(void)
// {
//     chassis_handle.vx = 1000;  //!!!!
//     chassis_handle.vy = 0;
//     chassis_handle.vw = 0;
// }
//
