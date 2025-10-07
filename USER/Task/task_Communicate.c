/* 包含头文件 ----------------------------------------------------------------*/
#include "task_Communicate.h"
#include "cmsis_os.h"
#include "mdl_comm.h"
#include "robot_info.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId CommTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t comm_task_stack = 0;
#endif

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void Comm_Task(void const *argument)
{
    for(;;)
    {
        Comm_ReceiveDataHandler();
        Comm_TransmitDataHandler();
        osDelay(COMMUNICATE_TASK_PERIOD);                             //?

#if INCLUDE_uxTaskGetStackHighWaterMark
        comm_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void Comm_TaskInit(void)
{
    osThreadDef(comm_task, Comm_Task, osPriorityNormal, 0, 256);     //通讯任务开始
    CommTaskHandle = osThreadCreate(osThread(comm_task), NULL);      //通讯任务开始
}
