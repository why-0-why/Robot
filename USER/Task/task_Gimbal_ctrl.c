#include "task_Gimbal_ctrl.h"


GimbalHandle_t gimbal_handle;

/* 任务 */
osThreadId GimbalTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t gimbal_task_stack = 0;
#endif



void GimbalTask(void const *argument)
{
    for(;;)
    {
        GimbalSensorUpdata();
		VisionDataUpdate(); 	//处理视觉数据
        GimbalCtrlModeSwitch();
        GimbalRampInit();
        GimbalRampUpdate();
        switch (gimbal_handle.ctrl_mode)
        {
            case GIMBAL_INIT:
            {
                GimbalInitMode();           //云台初始化
            }break;

            case GIMBAL_GYRO:
            {
                GimbalGyroAngleMode();     //云台陀螺仪模式
            }break;

            case GIMBAL_RELATIVE:           //云台分离模式
            {
                GimbalRelativeAngleMode();
            }break;

            case GIMBAL_NORMAL:             //云台跟随模式
            {
                GimbalNormalMode();
            }break;
            case GIMBAL_VISION:         //云台视觉跟踪模式
            {
                GimbalVisionAimMode();
            }break;
            default:
                break;
        }

//				 GimbalVisionAimMode();
        GimbalMotorControl(&gimbal_handle.yaw_motor);
        GimbalMotorControl(&gimbal_handle.pitch_motor);

        if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
        {
            pid_clear(&gimbal_handle.yaw_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.yaw_motor.pid.inter_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.outer_pid);
            pid_clear(&gimbal_handle.pitch_motor.pid.inter_pid);
            gimbal_handle.yaw_motor.current_set = 0;
            gimbal_handle.pitch_motor.current_set = 0;
            lock_motor(gimbal_handle.gimbal_can,CAN_ID);

        }

        GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);
//        GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
//                               (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);

				float pitch_pos = gimbal_handle.pitch_motor.motor_info->given_value;
        float pitch_vel=calculate_derivative(gimbal_handle.pitch_motor.motor_info->given_value,BSP_GetTime_ms());
        if(gimbal_handle.ctrl_mode == GIMBAL_INIT)  //初始化模式时对pitch进行限速
				{
					VAL_LIMIT(pitch_vel,0,0.02);
					VAL_LIMIT(pitch_pos,gimbal_dmmotor_pitch.pos-0.05,gimbal_dmmotor_pitch.pos+0.05);
				}
        GimbalMotorSendPos(pitch_pos,
                           pitch_vel);//后续需要添加达妙信息结构体
        osDelay(GIMBAL_CTRL_TASK_PERIOD);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void GimbalTaskInit(void)
{
    ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_CTRL_TASK_PERIOD);
    ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_CTRL_TASK_PERIOD);
	gimbal_handle.last_ctrl_mode = GIMBAL_ZARO;											  //无意义，仅用于代指未上电时的上一控制模式
//	  ESO_init();

    osThreadDef(gimbal_task, GimbalTask, osPriorityNormal, 0, 256);
    GimbalTaskHandle = osThreadCreate(osThread(gimbal_task), NULL);
}