#ifndef TASK_START_H
#define TASK_START_H

#include "main.h"
#include "cmsis_os.h"
#include "drv_timer.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "dvc_dt7.h"
#include "dvc_led.h"
#include "dvc_heater.h"
#include "dvc_buzzer.h"
#include "dvc_laser.h"
#include "dvc_app_gpio.h"
#include "task_SoftwareTimer_Check.h"
#include "task_IMU_Update.h"
#include "task_Chassis_ctrl.h"
#include "task_Console.h"
#include "task_Communicate.h"
#include "task_Detect.h"

#define START_TASK_PERIOD           100

typedef enum
{
    CHASSIS_APP = 1,
    GIMBAL_APP,
}AppType_e;

#endif //TASK_START_H