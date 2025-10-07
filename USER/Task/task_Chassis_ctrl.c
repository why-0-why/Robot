
#include "task_Chassis_ctrl.h"
#include "mdl_Chassis.h"
#include "cmsis_os.h"
#include "dvc_Supercapacity.h"
#include "robot_info.h"
/* 任务 */
osThreadId ChassisTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t chassis_task_stack = 0;
#endif

void ChassisTask(void const *argument)
{
    for(;;)
    {
        ChassisSensorUpdata();
        ChassisCtrlModeSwitch();
        ChassisSuperCapTest();
        switch (chassis_handle.ctrl_mode)
        {
            case CHASSIS_STOP:
            {
                ChassisStopMode();
            }break;
            case CHASSIS_FOLLOW_GIMBAL:
            {
                ChassisFollowGimbalMode();
            }break;
            case CHASSIS_SEPARATE_GIMBAL:
            {
                ChassisSeparateGimbalMode();
            }break;
            case CHASSIS_SPIN:
            {
                ChassisSpinMode();
            }break;
            case CHASSIS_ISLAND:
            {
                ChassisIslandMode();
            }break;
            default:
                break;
        }

        Chassis_ControlCalc(&chassis_handle);

        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_handle.chassis_motor[i].given_speed = chassis_handle.wheel_rpm[i];

            chassis_handle.chassis_motor[i].current_set = pid_calc(&chassis_handle.chassis_motor[i].pid,
                                                                   chassis_handle.chassis_motor[i].motor_info->speed_rpm,
                                                                   chassis_handle.chassis_motor[i].given_speed);

        }

      Chassis_LimitPower(&chassis_handle);
//        Chassis_LimitCap(&chassis_handle);

        if(chassis_handle.ctrl_mode == CHASSIS_RELAX)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                chassis_handle.chassis_motor[i].current_set = 0;
            }
        }

        SuperCap_SendMessage(&can1_obj);

        ChassisMotorSendCurrent(chassis_handle.chassis_motor[0].current_set,
                                chassis_handle.chassis_motor[1].current_set,
                                chassis_handle.chassis_motor[2].current_set,
                                chassis_handle.chassis_motor[3].current_set);

        osDelay(CHASSIS_CTRL_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark                           //freertos标准格式
        chassis_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif

    }
}

void ChassisTaskInit(void)
{
    osThreadDef(chassis_task, ChassisTask, osPriorityNormal, 0, 256);
    ChassisTaskHandle = osThreadCreate(osThread(chassis_task), NULL);
}