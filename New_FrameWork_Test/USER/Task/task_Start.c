
#include "task_Start.h"

void Task_Start(void const *argument)
{   
    for(;;)
    {
       

#if INCLUDE_uxTaskGetStackHighWaterMark
        Task_Start_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
