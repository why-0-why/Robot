#include  "dvc_laser.h"

/* IO对象结构 */
GPIO_Object_t laser_gpio;

/* 函数体 --------------------------------------------------------------------*/

void LASER_Init(void) {
    BSP_GPIO_NewObject_PWM(&laser_gpio, &htim3, TIM_CHANNEL_3);
}