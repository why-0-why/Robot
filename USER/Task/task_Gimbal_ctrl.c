// //
// // Created by Administrator on 25-8-1.
// //
// #include "task_Gimbal_ctrl.h"
// #include "mdl_Gimbal.h"
//
// /* 任务 */
// osThreadId GimbalTaskHandle;
// #if INCLUDE_uxTaskGetStackHighWaterMark
// static uint32_t gimbal_task_stack = 0;
// #endif
//
// void GimbalTask(void const *argument)
// {
//     for(;;)
//     {
//         GimbalSensorUpdata();
//         GimbalCtrlModeSwitch();
//         switch (gimbal_handle.ctrl_mode)
//         {
//             case GIMBAL_INIT:
//             {
//                 GimbalInitMode();           //云台初始化
//             }break;
//
//             case GIMBAL_GYRO:
//             {
//                 GimbalGyroAngleMode();     //云台陀螺仪模式
//             }break;
//
//             case GIMBAL_RELATIVE:           //云台分离模式
//             {
//                 GimbalRelativeAngleMode();
//             }break;
//
//             case GIMBAL_NORMAL:             //云台跟随模式
//             {
//                 GimbalNormalMode();
//             }break;
//             case GIMBAL_VISION:
//             {
//                 GimbalVisionAimMode();      //辅瞄模式
//             }break;
//             case GIMBAL_CLIPPED:             //吊射模式
//             {
//                 GimbalClippedMode();
//             }break;
// 			case GIMBAL_ADD:
//             {
//                 GimbalADDMode();
//             }break;
// 						case GIMBAL_MOVE:
// 						{
// 						GimbalMoveAimMode();
// 						}
//             default:
//                 break;
//         }
//
//         GimbalMotorControl(&gimbal_handle.yaw_motor);
//         GimbalMotorControl(&gimbal_handle.pitch_motor);
//
//         if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
//         {
//             pid_clear(&gimbal_handle.yaw_motor.pid.outer_pid);
//             pid_clear(&gimbal_handle.yaw_motor.pid.inter_pid);
//             pid_clear(&gimbal_handle.pitch_motor.pid.outer_pid);
//             pid_clear(&gimbal_handle.pitch_motor.pid.inter_pid);
//             gimbal_handle.yaw_motor.current_set = 0;
//             gimbal_handle.pitch_motor.current_set = 0;
//         }
//
//         GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
//                                (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);
//         osDelay(GIMBAL_CTRL_TASK_PERIOD);
//
// #if INCLUDE_uxTaskGetStackHighWaterMark
//         gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
// #endif
//     }
// }
//
// void GimbalTaskInit(void)
// {
//     ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
//     ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
//
//     KalmanCreate(&Yaw_Kalman_error, 1, 40);
//     KalmanCreate(&Pitch_Kalman_error, 1, 40);
//
//     KalmanCreate(&Distance_Kalman_error, 1, 2000);
//
//     mat_init(&yaw_kalman_filter.Q, 2, 2,yaw_kalman_filter_para.Q_data);
//     mat_init(&yaw_kalman_filter.R, 2, 2,yaw_kalman_filter_para.R_data);
//     kalman_filter_init(&yaw_kalman_filter,&yaw_kalman_filter_para);
//
//     mat_init(&pitch_kalman_filter.Q, 2, 2,pitch_kalman_filter_para.Q_data);
//     mat_init(&pitch_kalman_filter.R, 2, 2,pitch_kalman_filter_para.R_data);
//     kalman_filter_init(&pitch_kalman_filter,&pitch_kalman_filter_para);
//
//
//     AutoAimCoefficient.yaw_lpc = 45;
//     AutoAimCoefficient.pitch_lpc = 1.5;
//     AutoAimCoefficient.auto_err_yaw = 80;//120
//     AutoAimCoefficient.auto_err_pitch = 10;//150t7
//     AutoAimCoefficient.kalman_filter_delay = 80;
//     AutoAimCoefficient.kalman_filter_yaw_speed_min = 0.2;
//     AutoAimCoefficient.kalman_filter_pitch_speed_min = 0.15;
//     AutoAimCoefficient.kalman_filter_yaw_speed_max = 4;
//     AutoAimCoefficient.kalman_filter_yaw_amplification = 45;//200
//     AutoAimCoefficient.kalman_filter_pitch_amplification = 45;
//     Aim_Shootdelay.auto_aim_shoot_l = 0;
//     Aim_Shootdelay.auto_aim_shoot_r = 0;
//     Aim_Shootdelay.auto_aim_shoot_stop = 0;
//     osThreadDef(gimbal_task, GimbalTask, osPriorityNormal, 0, 256);
//     GimbalTaskHandle = osThreadCreate(osThread(gimbal_task), NULL);
// }