#include "task_Start.h"

AppType_e app_type;

void  AppInit(void);
void StartMusic(void);

void TaskStart(void const *argument)
{
    AppInit();
    for(;;)
    {
        osDelay(START_TASK_PERIOD);
#if INCLUDE_uxTaskGetStackHighWaterMark
        TaskStart_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
void AppInit() {

    BSP_DelayInit();
    APP_GPIO_Init();
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
    //    app_type = CHASSIS_APP;
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
        ChassisTaskInit();
        //			  UiTask_Init();
        //        IslandTaskInit();
    // }

    //    Calibrate_Init();
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
