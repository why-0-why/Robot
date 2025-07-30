#include "dvc_gpio.h"

/* IO对象结构 */
GPIO_Object_t app_gpio;
GPIO_Object_t led_r_gpio;
GPIO_Object_t led_g_gpio;
GPIO_Object_t led_b_gpio;
GPIO_Object_t temp_pwm_gpio;
GPIO_Object_t buzzer_gpio;
GPIO_Object_t laser_gpio;

/* 函数体 --------------------------------------------------------------------*/

void GPIO_Init(void) {
    BSP_GPIO_NewObject(&app_gpio, GPIO_INPUT_DEV, APP_CONFIG_GPIO_Port, APP_CONFIG_Pin);
    BSP_GPIO_NewObject_PWM(&led_r_gpio, &htim5, TIM_CHANNEL_3);
    BSP_GPIO_NewObject_PWM(&led_g_gpio, &htim5, TIM_CHANNEL_2);
    BSP_GPIO_NewObject_PWM(&led_b_gpio, &htim5, TIM_CHANNEL_1);
    BSP_GPIO_NewObject_PWM(&temp_pwm_gpio, &htim10, TIM_CHANNEL_1);
    BSP_GPIO_NewObject_PWM(&buzzer_gpio, &htim4, TIM_CHANNEL_3);
    BSP_GPIO_NewObject_PWM(&laser_gpio, &htim3, TIM_CHANNEL_3);
}