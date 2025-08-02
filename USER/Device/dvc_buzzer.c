#include "dvc_buzzer.h"

/* IO对象结构 */
GPIO_Object_t buzzer_gpio;

/* 函数体 --------------------------------------------------------------------*/

void BUZZER_Init(void) {
    BSP_GPIO_NewObject_PWM(&buzzer_gpio, &htim4, TIM_CHANNEL_3);
}
