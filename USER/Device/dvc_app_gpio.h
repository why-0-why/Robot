#ifndef DVC_APP_GPIO_H
#define DVC_APP_GPIO_H

#include "drv_gpio.h"

/* 自定义数据类型 */
typedef enum
{
    CHASSIS_APP = 1,
    GIMBAL_APP,
}AppType_e;// 应用层为底盘还是云台
/* IO对象结构 */

extern GPIO_Object_t app_gpio;


/* 函数声明 */

void APP_GPIO_Init(void);

#endif //DVC_APP_GPIO_H
