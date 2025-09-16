#ifndef TASK_GIMBAL_CTRL_H
#define TASK_GIMBAL_CTRL_H

#include "cmsis_os.h"
#include "mdl_Gimbal.h"
#include <user_lib.h>
#include "robot_info.h"

#define GIMBAL_CTRL_TASK_PERIOD          1
#define GIMBAL_UPLOAD_TIMER_PERIOD  20

extern GimbalHandle_t gimbal_handle;
#endif //TASK_GIMBAL_CTRL_H
