#include "TestTask.h"
void TestTask(void const *argument)
{   
    for(;;)
    {
       

#if INCLUDE_uxTaskGetStackHighWaterMark
        Test_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
