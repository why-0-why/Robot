#include "dvc_heater.h"

/* IO对象结构 */

GPIO_Object_t temp_pwm_gpio;

/* 函数体 --------------------------------------------------------------------*/

void Heater_Init(void) {
    BSP_GPIO_NewObject_PWM(&temp_pwm_gpio, &htim10, TIM_CHANNEL_1);
}