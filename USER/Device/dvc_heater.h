#ifndef DVC_GPIO_H
#define DVC_GPIO_H

#include "drv_gpio.h"

/* IO对象结构 */
extern GPIO_Object_t temp_pwm_gpio;

/* 函数声明 */

void Heater_Init(void);

#endif  // DVC_GPIO_H