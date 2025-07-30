#ifndef DVC_GPIO_H
#define DVC_GPIO_H

#include "drv_gpio.h"

/* IO对象结构 */
extern GPIO_Object_t app_gpio;
extern GPIO_Object_t led_r_gpio;
extern GPIO_Object_t led_g_gpio;
extern GPIO_Object_t led_b_gpio;
extern GPIO_Object_t temp_pwm_gpio;
extern GPIO_Object_t buzzer_gpio;
extern GPIO_Object_t laser_gpio;

void GPIO_Init(void);

#endif  // DVC_GPIO_H