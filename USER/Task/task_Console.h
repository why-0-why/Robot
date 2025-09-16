#ifndef TASK_CONSOLE_H
#define TASK_CONSOLE_H

#include "alg_ramp.h"
#include "cmsis_os.h"
#include "drv_timer.h"
#include "dvc_dt7.h"
#include "robot_info.h"
#include "task_Detect.h"
#include "mdl_comm.h"
#include "mdl_Console.h"

#define CONSOLE_TASK_PERIOD         20

void ConsoleTaskInit(void);

#endif //TASK_CONSOLE_H
