//
// Created by Administrator on 25-7-30.
//


/* 包含头文件 ----------------------------------------------------------------*/
#include "task_SoftwareTimer_Check.h"
#include "dvc_soft_timer.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId TimerTaskHandle;
#if INCLUDE_uxTaskGetStackHighWaterMark
static uint32_t timer_task_stack = 0;
#endif

SoftwareTimer_t soft_timer[TIMER_ELEMENT_NUM_MAX + 1];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void Task_SoftwareTimer_Check(void const *argument)
{
    uint32_t period = osKernelSysTick();                //赋软件接口文件里的值

    for(;;)                                           //一直循环 某个地方跑出来
    {
			TimerISR_Hook();                                //时钟中断服务

			for (int i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
        {
				  	if ((soft_timer[i].id != 0) && (soft_timer[i].callback != NULL))   //软时钟有ID且回调不等于空。
            {
                if (soft_timer_check(soft_timer[i].id) == SOFT_TIMER_TIMEOUT)  //软时钟ID检查
                {
                    soft_timer[i].callback(soft_timer[i].argc);                //软时钟回调

                    soft_timer_update(soft_timer[i].id, soft_timer[i].ticks);  //软时钟更新
                }
            }
        }
        osDelayUntil(&period, 1);                                                //？
#if INCLUDE_uxTaskGetStackHighWaterMark
        timer_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void SoftwareTimerTaskInit(void)
{
    soft_timer_init();                                                            //软时钟初始化
    osThreadDef(Task_SoftwareTimer_Check, Task_SoftwareTimer_Check, osPriorityNormal, 0, 1024);        //？时钟任务开始
    TimerTaskHandle = osThreadCreate(osThread(Task_SoftwareTimer_Check), NULL);                 //？时钟任务开始
}

int32_t SoftwareTimerRegister(SoftTimer_Callback_t callback, void *argc, uint32_t ticks)
{
    for (int i = 1; i < TIMER_ELEMENT_NUM_MAX + 1; i++)
    {
        if (soft_timer[i].id == 0)
        {
            soft_timer[i].id = soft_timer_req(ticks);
            soft_timer[i].ticks = ticks;
            soft_timer[i].argc = argc;
            soft_timer[i].callback = callback;
            return i;
        }
    }
    return 0;
}

