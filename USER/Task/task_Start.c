#include "task_Start.h"
#include "main.h"
#include "cmsis_os.h"



#include "task_SoftwareTimer_Check.h"
#include "task_IMU_Update.h"
#include "mdl_Chassis.h"

#include "task_Chassis_ctrl.h"
#include "task_Gimbal_ctrl.h"
#include "task_Console.h"
#include "task_Communicate.h"
#include "task_Detect.h"
#include "robot_info.h"
// 定时器
#include "drv_timer.h"
// 直驱IO设备
#include "dvc_app_gpio.h"
#include "dvc_led.h"
#include "dvc_heater.h"
#include "dvc_buzzer.h"
#include "dvc_laser.h"
// 通讯IO
#include "drv_can.h"
#include "drv_uart.h"

AppType_e app_type;

void StartMusic(void);

void TaskStart(void const *argument)
{


    BSP_DelayInit();// TODO：初始化
    /* IO设备初始化 */
    APP_GPIO_Init();// 初始化跳线帽GPIO口判断是云台还是底盘
    LED_Init();
    Heater_Init();
    Buzzer_Init();
    LASER_Init();
    CAN_Init();
    COM_Init();

    if (BSP_GPIO_ReadPin(&app_gpio))  //读云台短路帽的电平
    {
        app_type = GIMBAL_APP;
    }
    else
    {
        app_type = CHASSIS_APP;
    }

    if (app_type == GIMBAL_APP)
    {
        HAL_Delay(1000);
    }
    StartMusic();              //开机音效

    SoftwareTimerTaskInit();   //软时钟初始化
    IMU_TaskInit();            //陀螺仪初始化
    ConsoleTaskInit();         //控制初始化
    Comm_TaskInit();           //通讯初始化
    DetectTaskInit();          //?报错检查
    // ShootTaskInit();           //射击任务初始化
    // if (app_type == GIMBAL_APP)
    // {
    //     GimbalAppConfig();
    //     GimbalTaskInit();
    // }
    // else if (app_type == CHASSIS_APP)
    // {
    ChassisAppConfig();
    ChassisTaskInit();// TODO:ChassisAppConfig在mdl层，应该合并到task层
    //			  UiTask_Init();
    //        IslandTaskInit();
    // }

    //    Calibrate_Init();
    for(;;)
    {
        osDelay(START_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        TaskStart_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

AppType_e GetAppType(void)
{
    return app_type;
}

void StartMusic(void)
{
    Buzzer_SetBeep(DO, 150);   //蜂鸣器
    HAL_Delay(100);            //延时
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(DO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
    Buzzer_SetBeep(DO, 150);
    HAL_Delay(100);
    Buzzer_SetBeep(0, 0);
    HAL_Delay(100);
}
