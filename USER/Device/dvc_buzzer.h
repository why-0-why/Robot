
#ifndef DVC_BUZZER_H
#define DVC_BUZZER_H

#include "drv_gpio.h"

/* IO对象结构 */

extern GPIO_Object_t buzzer_gpio;

/* 函数声明 */

void BUZZER_Init(void);
#endif //DVC_BUZZER_H
