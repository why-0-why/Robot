#include "dvc_led.h"

/* IO对象结构 */
GPIO_Object_t led_r_gpio;
GPIO_Object_t led_g_gpio;
GPIO_Object_t led_b_gpio;

/* 函数体 --------------------------------------------------------------------*/

void LED_Init(void) {
    BSP_GPIO_NewObject_PWM(&led_r_gpio, &htim5, TIM_CHANNEL_3);
    BSP_GPIO_NewObject_PWM(&led_g_gpio, &htim5, TIM_CHANNEL_2);
    BSP_GPIO_NewObject_PWM(&led_b_gpio, &htim5, TIM_CHANNEL_1);
}

void LED_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    BSP_GPIO_SetPwmValue(&led_r_gpio, red);
    BSP_GPIO_SetPwmValue(&led_g_gpio, green);
    BSP_GPIO_SetPwmValue(&led_b_gpio, blue);
}